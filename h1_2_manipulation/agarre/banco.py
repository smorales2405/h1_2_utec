"""El banco de agarre en el robot: un objeto, N ensayos de coger -> levantar -> llevarlo sobre su bandeja -> sostenerlo
-> devolverlo a su sitio, con los dos brazos intercalados: "regulador" (agarre/lazo.py) y "firmware" (la Mano del reto
tal cual). El protocolo de comparacion del plan del regulador (§8): una sola tanda, brazos intercalados, aleatorizados
por bloques equilibrados y sin rachas largas; nunca contra una tanda de otro dia.

SupervisorBanco es el Supervisor del reto con otro `_niveles_objetos`: la entrada, la observacion, la salida y la
salida segura son las suyas, sin tocarlas. Cada ensayo es el `_coger` del reto hasta llegar sobre la bandeja (con los
mismos estados, el registro y las fotos); alli, en vez de soltarlo, lo SOSTIENE `sostener_s`, vuelve por el mismo
camino (lo comprobado al planificar, al reves), lo baja por la recta de levantar hasta donde lo cogio y lo suelta.
Si se cae o no se agarra, se pide recolocarlo (Enter) antes del siguiente.
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


# columnas de ensayos.csv (el resto, anidado, en resumen.json)
COLUMNAS = ["n", "brazo", "objeto", "mano", "resultado", "fase_escape", "t_cerrar_s", "t_establecido_s", "consigna_gf",
            "consigna_final_gf", "pico_apriete_cierre_gf", "apriete_al_cerrar_gf", "apriete_transporte_gf",
            "apriete_transporte_min_gf", "apriete_sostener_gf", "apriete_sostener_min_gf", "caida_sostener_pct",
            "resbalones", "escapes_detectados", "guardas", "desplazamiento_mm", "giro_deg", "mano_desde_el_plan_mm",
            "objeto_x", "objeto_y"]


class SupervisorBanco(Supervisor):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.banco = {"objeto": None, "ensayos": 20, "brazos": BRAZOS, "semilla": None, "racha_max": 2,
                      "sostener_s": 3.0}
        self.ensayos = []
        self.esperar_usuario = None      # f(texto) -> True para seguir (el objeto recolocado), False para acabar
        self.al_ensayo = []              # f(ensayo) al acabar cada uno (pruebas)

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
        k, previo = 0, None
        while k < len(orden):
            esc, f = self.observar()
            vistos = [o for o in esc.objetos if self._grupo(o.clase) == grupo and o.confianza >= c_min]
            o = max(vistos, key=lambda x: x.confianza) if vistos else None
            if previo is not None:
                # donde quedo el que se devolvio: lo que se movio en la mano (o al soltarlo)
                if o is not None:
                    previo["desplazamiento_mm"] = round(1000 * float(np.linalg.norm(o.p[:2] - np.asarray(previo["_xy"]))), 1)
                    s = (self.pl.objetos["objetos"].get(o.clase) or {}).get("simetria_deg", 0) or 360
                    d = (math.degrees(o.yaw) - previo["_yaw"] + s / 2) % s - s / 2
                    previo["giro_deg"] = round(d, 1) if s < 360 or o.clase != "esfera" else None
                self._guardar_csv()
                previo = None
            plan = self.pl.coger(o, bandeja, self.q(lado), lado) if o is not None else None
            if plan is None or not plan.comprobado:
                motivo = (f"no veo el {grupo} con confianza >= {c_min}" if o is None
                          else f"{o.clase} sin plan: {plan.motivo}")
                if not self._pedir_recolocar(f"{motivo}. Recolocalo en la zona de recogida"):
                    break
                continue
            brazo = orden[k]
            self.manos[lado].regular = brazo == "regulador"
            r = self._ensayo(plan, f, k + 1, brazo)
            self.ensayos.append(r)
            self.resultados.append({c: r.get(c) for c in ("n", "brazo", "resultado", "fase_escape")})
            self._guardar_csv()
            for cb in self.al_ensayo:
                cb(r)
            k += 1
            if r["resultado"] == "sostenido":
                previo = r
            elif k < len(orden):
                if not self._pedir_recolocar(f"ensayo {k} ({brazo}): {r['resultado']}. Recoloca el {grupo} en su sitio"):
                    break
        self._a("BANCO", "fin: " + "; ".join(
            f"{x}: {sum(1 for r in self.ensayos if r['brazo'] == x and r['resultado'] == 'sostenido')} de "
            f"{sum(1 for r in self.ensayos if r['brazo'] == x)} sostenidos" for x in b["brazos"]))

    def _pedir_recolocar(self, texto):
        """Con los brazos en la observacion: el usuario recoloca el objeto (Enter) o se acaba el banco."""
        self._a("RECOLOCAR", texto)
        if self.esperar_usuario is None:
            return False
        return bool(self.esperar_usuario(texto))

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
            r.update({"resbalones": tipos.count("resbalon"), "escapes_detectados": tipos.count("escape"),
                      "guardas": tipos.count("guarda")})
        return r
