"""El banco de agarre en el robot: un objeto, N ensayos de coger -> levantar -> llevarlo sobre su bandeja -> sostenerlo
-> devolverlo a su sitio, con los dos brazos intercalados: "regulador" (agarre/lazo.py) y "firmware" (la Mano del reto
tal cual). El protocolo de comparacion del plan del regulador (§8): una sola tanda, brazos intercalados, aleatorizados
por bloques equilibrados y sin rachas largas; nunca contra una tanda de otro dia.

SupervisorBanco es el Supervisor del reto con otro `_niveles_objetos`: la entrada, la observacion, la salida y la
salida segura son las suyas, sin tocarlas. Cada ensayo es el `_coger` del reto hasta llegar sobre la bandeja (con los
mismos estados, el registro y las fotos); alli, en vez de soltarlo, lo SOSTIENE `sostener_s`, vuelve por el mismo
camino (lo comprobado al planificar, al reves), lo baja por la recta de levantar hasta donde lo cogio y lo suelta.

Su sitio (`recolocar`, por defecto): cada ensayo empieza con el objeto en el mismo sitio, sobre la cruz de cinta del
tablero. Devolverlo lo mueve (~15 mm y ~10 grados en el cubo, 2026-11-25) y el agarre cambia con donde y como esta, asi
que se recoloca antes de cada ensayo. Al empezar se miran las cruces (con el objeto fuera de la suya) y se pide ponerlo
encima: donde quede la primera vez es su sitio, y lo apartado de la cruz que se vio, una medida de la percepcion. Tras
cada ensayo, la observacion de despues (tambien tras el ultimo) mide lo que se movio; si no esta en su sitio
(`tol_sitio_mm`, `tol_sitio_deg`), se pide recolocarlo diciendo hacia donde, y se comprueba otra vez con la camara.
Sin `recolocar`, el siguiente lo encuentra donde quedo y solo se pide recolocarlo si se cae o no se agarra.
"""

import csv
import math
import os
import random
import time

import numpy as np

from manip.cinematica import T_punto
from manip.geometria import aplicar
from manip.mensajes import PuntoPaso
from manip.planificacion import centro_agarre
from manip.supervisor import Supervisor

BRAZOS = ("regulador", "firmware")


def calendario(n, brazos=BRAZOS, semilla=None, racha_max=2):
    """El brazo de cada ensayo: bloques con cada brazo una vez, en orden aleatorio, sin mas de racha_max seguidos del
    mismo (con bloques de dos no pasa de 2: AB|BA)."""
    rng = random.Random(semilla)
    sal = []
    while len(sal) < n:
        for _ in range(50):
            b = list(brazos)
            rng.shuffle(b)
            prueba = sal + b
            racha, mx = 1, 1
            for i in range(1, len(prueba)):
                racha = racha + 1 if prueba[i] == prueba[i - 1] else 1
                mx = max(mx, racha)
            if mx <= racha_max:
                break
        sal = prueba
    return sal[:n]


def desde_sitio(sitio, xy, yaw_deg, simetria_deg):
    """Lo apartado del objeto de su sitio. dx_mm y dy_mm: hacia donde moverlo (sitio - objeto, en el torso: x adelante,
    y a la izquierda del robot); giro_deg: objeto - sitio, modulo la simetria (None si no tiene orientacion)."""
    dx = 1000.0 * (float(sitio["x"]) - float(xy[0]))
    dy = 1000.0 * (float(sitio["y"]) - float(xy[1]))
    s = float(simetria_deg or 0)
    giro = None if s <= 0 else (float(yaw_deg) - float(sitio["yaw_deg"]) + s / 2) % s - s / 2
    return {"dx_mm": dx, "dy_mm": dy, "mm": math.hypot(dx, dy), "giro_deg": giro}


def texto_correccion(d):
    """Que hacer para ponerlo en su sitio, dicho desde el robot (el yaw, antihorario visto desde arriba)."""
    partes = []
    if abs(d["dx_mm"]) >= 2:
        partes.append(f"{abs(d['dx_mm']):.0f} mm " + ("hacia delante (lejos del robot)" if d["dx_mm"] > 0
                                                     else "hacia el robot"))
    if abs(d["dy_mm"]) >= 2:
        partes.append(f"{abs(d['dy_mm']):.0f} mm hacia la " + ("izquierda" if d["dy_mm"] > 0 else "derecha")
                      + " del robot")
    txt = ("muevelo " + " y ".join(partes)) if partes else ""
    g = d["giro_deg"]
    if g is not None and abs(g) >= 1:
        txt += ("; " if txt else "") + f"giralo {abs(g):.0f} grados en sentido " + ("horario" if g > 0 else "antihorario") \
            + " (visto desde arriba)"
    return txt or "ya esta en su sitio"


# columnas de ensayos.csv (el resto, anidado, en resumen.json)
COLUMNAS = ["n", "brazo", "objeto", "mano", "resultado", "fase_escape", "t_cerrar_s", "t_establecido_s", "consigna_gf",
            "consigna_final_gf", "pico_apriete_cierre_gf", "apriete_al_cerrar_gf", "apriete_transporte_gf",
            "apriete_transporte_min_gf", "apriete_sostener_gf", "apriete_sostener_min_gf", "caida_sostener_pct",
            "resbalones", "ceden_breves", "escapes_detectados", "guardas", "desplazamiento_mm", "giro_deg",
            "mano_desde_el_plan_mm", "objeto_x", "objeto_y", "sitio_mm", "sitio_deg"]


class SupervisorBanco(Supervisor):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.banco = {"objeto": None, "ensayos": 20, "brazos": BRAZOS, "semilla": None, "racha_max": 2,
                      "sostener_s": 3.0, "recolocar": True, "tol_sitio_mm": 5.0, "tol_sitio_deg": 5.0,
                      "intentos_sitio": 3}
        self.ensayos = []
        self.esperar_usuario = None      # f(texto) -> True para seguir (el objeto recolocado), False para acabar
        self.al_ensayo = []              # f(ensayo) al acabar cada uno, tras la observacion de despues (pruebas)
        self.sitio = None                # donde empieza cada ensayo: {x, y, yaw_deg, clase} (la primera colocacion)
        self._cruces = []

    # --- el bucle de ensayos ------------------------------------------------------------------------------
    def _niveles_objetos(self):
        b = self.banco
        grupo = self._grupo(b["objeto"])
        clases = [c for c in self.asignacion if self._grupo(c) == grupo]
        if not clases:
            self._a("BANCO", f"'{grupo}' no tiene bandeja en la asignacion: nada que hacer")
            return
        bandeja = self.asignacion[clases[0]]
        lado = self.pl.lado_de_bandeja(bandeja)
        orden = calendario(int(b["ensayos"]), tuple(b["brazos"]), b["semilla"], int(b["racha_max"]))
        b["calendario"] = orden
        self._a("BANCO", f"{grupo}: {len(orden)} ensayos con la mano {lado}, sostenido {b['sostener_s']:.0f} s sobre la "
                         f"bandeja {bandeja} y devuelto a su sitio", {"calendario": orden, "semilla": b["semilla"]})
        vel = self.pl.p.setdefault("vel_recta_m_s", {})
        vel.setdefault("devolver", vel.get("agarrar", 0.01))       # la bajada para devolverlo, como la del agarre
        c_min = self.cfg["percepcion"]["confianza_min"]
        recolocar = bool(b.get("recolocar", True))
        fin = False
        o, f = self._ver(grupo, c_min)
        if recolocar:
            self._mirar_cruces(f)
            o, f, fin = self._a_su_sitio(grupo, c_min, f"Pon el {grupo} centrado sobre la cruz{self._recto(grupo)}")
        k = 0
        while not fin and k < len(orden):
            plan = self.pl.coger(o, bandeja, self.q(lado), lado) if o is not None else None
            if plan is None or not plan.comprobado:
                motivo = (f"no veo el {grupo} con confianza >= {c_min}" if o is None
                          else f"{o.clase} sin plan: {plan.motivo}")
                if recolocar:
                    o, f, fin = self._a_su_sitio(grupo, c_min, f"{motivo}. Ponlo sobre la cruz")
                else:
                    fin = not self._pedir_recolocar(f"{motivo}. Recolocalo en la zona de recogida")
                    if not fin:
                        o, f = self._ver(grupo, c_min)
                continue
            brazo = orden[k]
            self.manos[lado].regular = brazo == "regulador"
            r = self._ensayo(plan, f, k + 1, brazo)
            if self.sitio is not None:
                d = desde_sitio(self.sitio, o.p[:2], math.degrees(o.yaw), self._simetria(o.clase))
                r["sitio_mm"] = round(d["mm"], 1)
                r["sitio_deg"] = None if d["giro_deg"] is None else round(d["giro_deg"], 1)
            self.ensayos.append(r)
            self.resultados.append({c: r.get(c) for c in ("n", "brazo", "resultado", "fase_escape")})
            k += 1
            # la observacion de despues (tambien tras el ultimo): donde quedo, lo que se movio en la mano o al soltarlo
            o, f = self._ver(grupo, c_min)
            self._desplazamiento(r, o)
            self._guardar_csv()
            for cb in self.al_ensayo:
                cb(r)
            if k >= len(orden):
                break
            pre = f"ensayo {k} ({brazo}): {r['resultado']}. "
            if recolocar:
                if r["resultado"] == "sostenido" and self._en_sitio(o):
                    continue
                texto = pre + f"Pon el {grupo} sobre la cruz{self._recto(grupo)}"
                if o is not None and self._en_zona(o):
                    texto += f" ({texto_correccion(desde_sitio(self.sitio, o.p[:2], math.degrees(o.yaw), self._simetria(o.clase)))})"
                o, f, fin = self._a_su_sitio(grupo, c_min, texto)
            elif r["resultado"] != "sostenido":
                fin = not self._pedir_recolocar(pre + f"Recoloca el {grupo} en su sitio")
                if not fin:
                    o, f = self._ver(grupo, c_min)
        self._a("BANCO", "fin: " + "; ".join(
            f"{x}: {sum(1 for r in self.ensayos if r['brazo'] == x and r['resultado'] == 'sostenido')} de "
            f"{sum(1 for r in self.ensayos if r['brazo'] == x)} sostenidos" for x in b["brazos"]))

    def _pedir_recolocar(self, texto):
        """Con los brazos en la observacion: el usuario recoloca el objeto (Enter) o se acaba el banco."""
        self._a("RECOLOCAR", texto)
        if self.esperar_usuario is None:
            return False
        return bool(self.esperar_usuario(texto))

    # --- el sitio del objeto ------------------------------------------------------------------------------
    def _ver(self, grupo, c_min):
        """Observa: (el objeto del banco con mas confianza o None, el fotograma)."""
        esc, f = self.observar()
        vistos = [o for o in esc.objetos if self._grupo(o.clase) == grupo and o.confianza >= c_min]
        return (max(vistos, key=lambda x: x.confianza) if vistos else None), f

    def _simetria(self, clase):
        return float((self.pl.objetos["objetos"].get(clase) or {}).get("simetria_deg", 0) or 0)

    def _recto(self, grupo):
        if self._simetria(grupo) <= 0:
            return ""
        return " y girado como la primera vez" if self.sitio else ", recto (caras paralelas a los bordes de la mesa)"

    def _en_zona(self, o):
        z = self.pl.montaje["zona_recogida"]
        return z["x"][0] <= o.p[0] <= z["x"][1] and z["y"][0] <= o.p[1] <= z["y"][1]

    def _dentro(self, d):
        b = self.banco
        return d["mm"] <= float(b["tol_sitio_mm"]) and (d["giro_deg"] is None or abs(d["giro_deg"]) <= float(b["tol_sitio_deg"]))

    def _en_sitio(self, o):
        if o is None or self.sitio is None or o.clase != self.sitio["clase"]:
            return False
        return self._dentro(desde_sitio(self.sitio, o.p[:2], math.degrees(o.yaw), self._simetria(o.clase)))

    def _mirar_cruces(self, f):
        """Las cruces de cinta a la vista (el detector de dianas del nivel 1, sin profundidad: §79), antes de poner el
        objeto encima. Solo es una medida: si falla o no hay, el sitio es donde se ponga."""
        self._cruces = []
        if self.detector_dianas is None or f is None:
            return
        try:
            cruces = [np.asarray(c, float)[:2] for c in self.detector_dianas(f)]
        except Exception as e:                       # noqa: BLE001  (una medida de mas: no para el banco)
            self._a("OBSERVAR", f"cruces: el detector fallo ({e})")
            return
        self._cruces = cruces
        self._a("OBSERVAR", f"{len(cruces)} cruces a la vista", {"cruces": [np.round(c, 4).tolist() for c in cruces]})

    def _fijar_sitio(self, o):
        b = self.banco
        self.sitio = {"x": float(o.p[0]), "y": float(o.p[1]), "yaw_deg": float(math.degrees(o.yaw)), "clase": o.clase}
        b["sitio"] = {"x": round(self.sitio["x"], 4), "y": round(self.sitio["y"], 4),
                      "yaw_deg": round(self.sitio["yaw_deg"], 1), "clase": o.clase}
        cerca = [c for c in self._cruces if np.linalg.norm(c - o.p[:2]) <= 0.04]
        cruz = min(cerca, key=lambda c: np.linalg.norm(c - o.p[:2])) if cerca else None
        b["cruz"] = None if cruz is None else np.round(cruz, 4).tolist()
        b["sitio_desde_cruz_mm"] = None if cruz is None else np.round(1000 * (o.p[:2] - cruz), 1).tolist()
        texto = (f"sitio del {o.clase}: ({o.p[0]:.3f}, {o.p[1]:.3f}) a {math.degrees(o.yaw):.0f} grados; "
                 + (f"visto a ({b['sitio_desde_cruz_mm'][0]:+.0f}, {b['sitio_desde_cruz_mm'][1]:+.0f}) mm de la cruz"
                    if cruz is not None else "sin la cruz vista antes de ponerlo (tapada o fuera): el sitio es donde se puso"))
        self._a("BANCO", texto, {"sitio": b["sitio"], "cruz": b["cruz"], "sitio_desde_cruz_mm": b["sitio_desde_cruz_mm"]})

    def _a_su_sitio(self, grupo, c_min, texto):
        """Pide poner el objeto en su sitio y lo comprueba con la camara; la primera vez, donde lo pongan es su sitio.
        Tras `intentos_sitio` colocaciones fuera de tolerancia se acepta (queda en sitio_mm). (objeto, fotograma, fin)."""
        fuera = 0
        while True:
            if not self._pedir_recolocar(texto):
                return None, None, True
            o, f = self._ver(grupo, c_min)
            if o is None:
                texto = f"No veo el {grupo} (confianza >= {c_min}): ponlo sobre la cruz"
                continue
            if self.sitio is None:
                self._fijar_sitio(o)
                return o, f, False
            if o.clase != self.sitio["clase"]:
                texto = f"Lo veo como {o.clase}, no como {self.sitio['clase']}: ponlo como la primera vez"
                continue
            d = desde_sitio(self.sitio, o.p[:2], math.degrees(o.yaw), self._simetria(o.clase))
            if self._dentro(d):
                return o, f, False
            fuera += 1
            giro = "" if d["giro_deg"] is None else f" y {abs(d['giro_deg']):.0f} grados"
            if fuera >= int(self.banco["intentos_sitio"]):
                self._a("RECOLOCAR", f"aceptado a {d['mm']:.0f} mm{giro} de su sitio, tras {fuera} intentos",
                        {k: (None if v is None else round(v, 1)) for k, v in d.items()})
                return o, f, False
            texto = f"El {grupo} esta a {d['mm']:.0f} mm{giro} de su sitio: {texto_correccion(d)}"

    def _desplazamiento(self, r, o):
        """Lo que se movio el objeto en el ensayo: de la observacion de antes a la de despues."""
        if o is None or not self._en_zona(o):
            return
        r["desplazamiento_mm"] = round(1000 * math.hypot(o.p[0] - r["_xy"][0], o.p[1] - r["_xy"][1]), 1)
        s = self._simetria(o.clase)
        r["giro_deg"] = round((math.degrees(o.yaw) - r["_yaw"] + s / 2) % s - s / 2, 1) if s > 0 else None

    def _guardar_csv(self):
        ruta = os.path.join(self.reg.carpeta, "ensayos.csv")
        with open(ruta, "w", newline="") as fh:
            w = csv.DictWriter(fh, COLUMNAS, extrasaction="ignore")
            w.writeheader()
            for r in self.ensayos:
                w.writerow({c: r.get(c) for c in COLUMNAS})

    # --- un ensayo ------------------------------------------------------------------------------------
    def _ensayo(self, plan, f, n, brazo):
        lado, mano, ag = plan.mano, self.manos[plan.mano], plan.agarre
        clase = plan.objeto.clase
        self._plan_activo = plan
        r = {"n": n, "brazo": brazo, "objeto": clase, "mano": lado, "resultado": None, "fase_escape": None,
             "objeto_x": round(float(plan.objeto.p[0]), 4), "objeto_y": round(float(plan.objeto.p[1]), 4),
             "_xy": [float(plan.objeto.p[0]), float(plan.objeto.p[1])], "_yaw": math.degrees(plan.objeto.yaw)}
        self._a("PLANIFICAR", f"ensayo {n} ({brazo}): {clase} con la mano {lado}", plan.detalle)
        foto = f"ensayo{n:02d}_{brazo}"
        if f is not None:
            self.reg.imagen(f"{foto}_antes", f.color)
        n_agarres = len(getattr(mano, "agarres", []))
        mano.preforma(ag.preforma, vigilar=self.comprobar)
        punto = T_punto(centro_agarre(self.pl.objetos, clase, plan.detalle["agarre"]), lado)
        tramos = self._tramos(plan.puntos_de_paso)
        nombres = [x for x, _ in tramos]
        k = nombres.index("agarrar")

        def desde(i):
            return tramos[i - 1][1][-1].q
        self._a("MOVER", f"a {clase}")
        for _, pts in tramos[:k]:
            for p in pts:
                self._mover(lado, p)
        self._a("DESCENSO", "al agarre, guiado")
        g = self._recta_guiada(lado, desde(k), tramos[k][1], punto, ajuste_inicio=True, ajuste_fin=True)
        q_ag = tramos[k][1][-1].q
        fot = self.fuente.siguiente(0.5) if hasattr(self.fuente, "siguiente") else None
        if fot is not None:
            self.reg.imagen(f"{foto}_bajado", fot.color)
        self.vig.manos_vigiladas = {lado}
        self._a("CERRAR", f"{brazo}: " + ("regulador de fuerza" if brazo == "regulador" else f"FORCE_SET {ag.force_set}"))
        t0 = time.monotonic()
        e = mano.cerrar(ag.force_set, vigilar=self._sostener(lado, g), speed=ag.speed)
        r["t_cerrar_s"] = round(time.monotonic() - t0, 2)
        ok, det = mano.agarre(e, ag.force_set)
        en_agarre = np.round(1000 * (aplicar(self.pl.cin.fk(lado, self.q(lado), "mano"), punto)
                                     - aplicar(self.pl.cin.fk(lado, q_ag, "mano"), punto)), 1)
        r["mano_desde_el_plan_mm"] = round(float(np.linalg.norm(en_agarre)), 1)
        r["agarre"] = {int(k_): v for k_, v in det.items()}
        cero = getattr(mano, "cero", np.zeros(6))
        self._a("CERRAR", "agarre comprobado" if ok else "agarre fallido",
                {"dedos": r["agarre"], "fuerza_gf": (e.fuerza - cero).round(0).tolist(), "angulo": e.angulo.round(0).tolist(),
                 "mano_desde_el_plan_mm": en_agarre.tolist()})
        fot = self.fuente.siguiente(0.5) if hasattr(self.fuente, "siguiente") else None
        if fot is not None:
            self.reg.imagen(f"{foto}_cerrado", fot.color)
        k_lev = nombres.index("levantar")
        lev = tramos[k_lev][1]
        if not ok:
            mano.abrir(vigilar=self.comprobar)
            self.vig.manos_vigiladas = set()
            self._a("VOLVER", "agarre fallido: abrir, subir y a la observacion")
            self._recta_guiada(lado, desde(k_lev), lev, punto, guia_previa=g)
            self._mover(lado, plan.puntos_de_paso[-1])
            r["resultado"] = "agarre_fallido"
            return self._cerrar_ensayo(r, mano, n_agarres)
        escape = {"seguidas": 0, "en": None}

        def vigilar_escape(e2):
            if escape["en"] is not None:
                return
            escape["seguidas"] = escape["seguidas"] + 1 if mano.escapado(e2, ag.force_set) else 0
            if escape["seguidas"] >= 2:
                escape["en"] = self.estado
                self._a(self.estado, f"{clase} se escapa de la mano",
                        {"angulo": e2.angulo.round(0).tolist(), "fuerza_gf": (e2.fuerza - cero).round(0).tolist()})
        t_sujetar, t_leer = [time.monotonic()], [time.monotonic()]
        cada_leer = min(0.2, self.vig.mano_max_s / 2)

        def sujetar():
            ahora = time.monotonic()
            if ag.reapretar and ahora - t_sujetar[0] > 1.0:
                mano.sujetar(ag.force_set)
                t_sujetar[0] = ahora
            if ahora - t_leer[0] > cada_leer:
                t_leer[0] = ahora
                vigilar_escape(mano.leer())
        mano.leer()
        self._a("TRANSPORTE", f"levantar y sobre la bandeja {plan.bandeja}")
        g = self._recta_guiada(lado, desde(k_lev), lev, punto, guia_previa=g, extra=sujetar)
        vigilar_escape(mano.leer())
        if escape["en"] is None:
            vigilar_escape(mano.leer())
        k_baj = nombres.index("bajar_a_bandeja")
        ida = [p for _, pts in tramos[k_lev + 1:k_baj] for p in pts]
        if escape["en"] is None:
            for p in ida:
                self._mover(lado, p, extra=sujetar)
        if escape["en"] is None:
            s = float(self.banco["sostener_s"])
            self._a("SOSTENER", f"{s:.0f} s sobre la bandeja {plan.bandeja}")
            t1 = time.monotonic()
            while time.monotonic() - t1 < s and escape["en"] is None:
                self.comprobar()
                sujetar()
                time.sleep(0.02)
        if escape["en"] is not None:
            # se cayo (o se va cayendo): abrir y a la observacion; el ensayo cuenta como escapado
            mano.abrir(vigilar=self.comprobar)
            self.vig.manos_vigiladas = set()
            self._a("VOLVER", f"{clase} se escapo en {escape['en']}: abrir y a la observacion")
            self._mover(lado, plan.puntos_de_paso[-1])
            r["resultado"], r["fase_escape"] = "escapado", escape["en"]
            return self._cerrar_ensayo(r, mano, n_agarres)
        # devolverlo: por el mismo camino (al reves) hasta lo alto de la recta de levantar, y por ella hasta el agarre
        self._a("DEVOLVER", f"{clase} a su sitio")
        self._mover(lado, PuntoPaso(q=lev[-1].q, duracion=sum(p.duracion for p in ida) or lev[-1].duracion,
                                    nombre="devolver"), extra=sujetar)
        bajada = [PuntoPaso(q=p.q, duracion=p.duracion, nombre="devolver") for p in reversed(lev[:-1])]
        bajada.append(PuntoPaso(q=np.asarray(q_ag, float), duracion=lev[0].duracion, nombre="devolver"))
        g = self._recta_guiada(lado, lev[-1].q, bajada, punto, guia_previa=g, seguida=False, extra=sujetar,
                               ajuste_inicio=True)
        vigilar_escape(mano.leer())
        self._a("SOLTAR", f"{clase} en su sitio" if escape["en"] is None else f"{clase} se escapo al devolverlo")
        fs, pre = ag.force_set, ag.preforma
        sujetan = [k_ for k_ in range(5) if fs[k_] is not None and fs[k_] > 0]
        recogidos = [k_ for k_ in range(5) if k_ not in sujetan and pre[k_] is not None and 0 <= pre[k_] < 500]
        mano.abrir(vigilar=self.comprobar, dedos=sujetan if recogidos else None)
        self.vig.manos_vigiladas = set()
        self._a("VOLVER", "subir y a la observacion")
        self._recta_guiada(lado, q_ag, lev, punto, guia_previa=g, seguida=False)
        if recogidos:
            mano.abrir(vigilar=self.comprobar)
        self._mover(lado, plan.puntos_de_paso[-1])
        self._plan_activo = None
        if escape["en"] is None:
            r["resultado"] = "sostenido"
        else:
            r["resultado"], r["fase_escape"] = "escapado", escape["en"]
        return self._cerrar_ensayo(r, mano, n_agarres)

    def _cerrar_ensayo(self, r, mano, n_agarres):
        """Las medidas del lazo de este ensayo (regulando u observando), al resumen y a las columnas del CSV."""
        self._plan_activo = None
        nuevos = getattr(mano, "agarres", [])[n_agarres:]
        lz = nuevos[-1] if nuevos else None
        r["lazo"] = lz
        if lz:
            fases = lz.get("apriete_por_fase") or {}
            picos = (lz.get("picos_por_fase") or {}).get("CERRAR", {})
            cer, tr, so = fases.get("CERRAR", {}), fases.get("TRANSPORTE", {}), fases.get("SOSTENER", {})
            r.update({"t_establecido_s": lz.get("t_establecido_s"), "consigna_gf": lz.get("consigna_gf"),
                      "consigna_final_gf": lz.get("consigna_final_gf") if lz.get("regulado") else None,
                      "pico_apriete_cierre_gf": cer.get("max"), "apriete_al_cerrar_gf": cer.get("final"),
                      "apriete_transporte_gf": tr.get("mediana"), "apriete_transporte_min_gf": tr.get("min"),
                      "apriete_sostener_gf": so.get("mediana"), "apriete_sostener_min_gf": so.get("min"),
                      "picos_cierre_gf": picos})
            if so.get("inicio"):
                r["caida_sostener_pct"] = round(100.0 * (so["inicio"] - so["final"]) / so["inicio"], 1)
            tipos = [e["tipo"] for e in lz.get("eventos", [])]
            r.update({"resbalones": tipos.count("resbalon"), "ceden_breves": tipos.count("cede_breve"),
                      "escapes_detectados": tipos.count("escape"),
                      "guardas": tipos.count("guarda")})
        return r
