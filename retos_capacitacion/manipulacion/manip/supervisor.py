"""Supervisor (seccion 5 del PDF): maquina de estados explicita, unico que cambia el peso de arm_sdk y el unico
que da un objeto por terminado; ejecuta la salida segura.

    ESPERA -> ENTRADA (rampa de 12 s) -> A_OBSERVACION (los dos brazos, en espejo)
      -> por cada diana u objeto: OBSERVAR -> PLANIFICAR -> MOVER -> DESCENSO -> (TOCAR | CERRAR -> TRANSPORTE
         -> SOLTAR) -> VOLVER
      -> SALIDA (observacion -> reposo, peso a 0 en >= 5 s) -> FIN
    cualquier estado con peso -> SALIDA_SEGURA: congelar, abrir la mano, subir a la altura de seguridad, salir
      por la pose de observacion al reposo y solo entonces bajar el peso.
La salida por la observacion es una ampliacion de la del PDF ("subirlo a la altura de seguridad y solo
entonces bajar el peso a 0"): en el gemelo, ir al reposo desde encima de la mesa barre su borde, y con el peso
en 0 la politica devuelve el brazo a su postura por donde quiera (lo advierte el propio PDF). Se puede quitar
con supervisor.salida_por_observacion: false.

Percepcion nueva antes de cada agarre (nivel 3: el plan no se calcula una sola vez). En simulacro el brazo no
publica en rt/arm_sdk (brazo.py) y la mano abre y cierra en el aire.
"""

import math
import time

import numpy as np

from .cinematica import TareaIK, Banda
from .mensajes import JUNTAS_BRAZO, LADOS, Parada, Plan, PuntoPaso


# puntos de paso a la altura de seguridad o de transporte: su tramo desde la observacion (o hacia ella) es limpio
PASOS_ALTOS = ("sobre", "subir", "levantar", "sobre_bandeja", "retirar_de_bandeja")


class Contacto(Exception):
    """La yema toco la diana: se para el descenso."""


class Supervisor:
    def __init__(self, cfg, robot, brazo, manos, percepcion, planificador, fuente, vigilante, registro,
                 nivel=1, simulacro=False, dianas=None, asignacion=None, detector_dianas=None, log=print):
        self.cfg, self.robot, self.brazo, self.manos = cfg, robot, brazo, manos
        self.per, self.pl, self.fuente, self.vig, self.reg = percepcion, planificador, fuente, vigilante, registro
        self.nivel, self.simulacro = nivel, simulacro
        self.dianas = dianas or []
        self.asignacion = asignacion or {}
        self.detector_dianas = detector_dianas
        self.log = log
        self.estado = "ESPERA"
        self.motivo_estado = ""
        self._plan_activo = None      # el plan que se esta ejecutando: la salida segura sale por sus puntos altos
        self.resultados = []
        self.c = cfg["contacto"]
        self.s = cfg["supervisor"]
        self._ultimo_fotograma = None

    # --- utilidades -----------------------------------------------------------------------------------
    def _a(self, estado, motivo="", detalle=None):
        self.estado, self.motivo_estado = estado, motivo
        self.reg.evento(estado, motivo, detalle)

    def q(self, lado):
        return self.robot.q()[JUNTAS_BRAZO[lado]]

    def comprobar(self):
        self.vig.comprobar()

    def contexto_registro(self):
        return {"estado": self.estado, "peso": self.brazo.peso, "obj": self.brazo.consigna()}

    def _mover(self, lado, punto, extra=None):
        """Un punto de paso con el brazo `lado`; `extra` se llama en cada ciclo (contacto, sujetar...)."""
        def chequeo():
            self.comprobar()
            if extra:
                extra()
        self.brazo.ir_a(self.brazo.pose_brazo(lado, punto.q), punto.duracion, chequeo)

    def _ambos(self, planes):
        """Mueve los dos brazos a la vez a sus puntos finales (entrada y salida en espejo)."""
        pose, dur = {}, 0.0
        for p in planes:
            pose.update(self.brazo.pose_brazo(p.mano, p.puntos_de_paso[-1].q))
            dur = max(dur, sum(x.duracion for x in p.puntos_de_paso))
        self.brazo.ir_a(pose, dur, self.comprobar)

    # --- percepcion -----------------------------------------------------------------------------------
    def observar(self):
        f = self.fuente.fresco(timeout_s=3.0)
        if f is None:
            raise Parada("camara sin fotogramas al observar")
        self._ultimo_fotograma = f
        q = self.robot.q()
        esc = self.per.procesar(f, {l: q[JUNTAS_BRAZO[l]] for l in LADOS})
        self.pl.poner_escena(esc)
        self._a("OBSERVAR", f"{len(esc.objetos)} objetos", {"objetos": [(o.clase, round(o.confianza, 2), np.round(o.p, 3).tolist())
                                                                          for o in esc.objetos], "ms": round(esc.ms, 1)})
        return esc, f

    # --- tirada ---------------------------------------------------------------------------------------
    def ejecutar(self):
        motivo_fin = "completado"
        try:
            self.vig.iniciar()
            self._a("ENTRADA", f"peso 0 -> 1 en {self.brazo.t_entrada:.0f} s siguiendo la postura medida")
            self.brazo.rampa_peso(1.0, comprobar=self.comprobar)
            planes = [self.pl.entre_reposo_y_observacion(l, True) for l in LADOS]
            malos = [p.motivo for p in planes if not p.comprobado]
            if malos:
                raise Parada(f"reposo -> observacion no pasa la comprobacion: {malos}")
            self._a("A_OBSERVACION", "los dos brazos a la pose de observacion")
            self._ambos(planes)
            if self.nivel == 1:
                self._nivel1()
            else:
                self._niveles_objetos()
            self._salida()
        except Parada as e:
            motivo_fin = f"salida segura: {e}"
            self.salida_segura(str(e))
        except KeyboardInterrupt:
            motivo_fin = "salida segura: Ctrl+C"
            self.salida_segura("Ctrl+C")
        return motivo_fin

    def _salida(self):
        self._a("SALIDA", "observacion -> reposo")
        planes = [self.pl.entre_reposo_y_observacion(l, False) for l in LADOS]
        if all(p.comprobado for p in planes):
            self._ambos(planes)
        self.brazo.rampa_peso(0.0)
        self._a("FIN", "peso 0.00")

    # --- nivel 1 --------------------------------------------------------------------------------------
    def _nivel1(self):
        dianas = list(self.dianas)
        if not dianas and self.detector_dianas is not None:
            _, f = self.observar()
            dianas = self.detector_dianas(f)
            self._a("OBSERVAR", f"{len(dianas)} dianas", {"dianas": [np.round(d, 3).tolist() for d in dianas]})
        for i, xy in enumerate(dianas):
            preferida = "izq" if xy[1] >= 0 else "der"
            plan = None
            for lado in (preferida, "der" if preferida == "izq" else "izq"):
                p = self.pl.tocar(lado, xy, self.q(lado))
                if p.comprobado:
                    plan = p
                    break
                self._a("PLANIFICAR", f"diana {i + 1} con la mano {lado}: {p.motivo}")
            if plan is None:
                self.resultados.append({"diana": list(xy), "resultado": "sin plan"})
                continue
            self.resultados.append(self._tocar(i + 1, xy, plan))

    def _tocar(self, n, xy, plan):
        lado, mano = plan.mano, self.manos[plan.mano]
        self._plan_activo = plan
        self._a("PLANIFICAR", f"diana {n} con la mano {lado}", plan.detalle)
        mano.preforma(self.pl.objetos["preformas"]["senalar"], vigilar=self.comprobar)
        pts = {p.nombre: p for p in plan.puntos_de_paso}
        self._a("MOVER", f"sobre la diana {n}")
        self._mover(lado, pts["sobre"])
        self._mover(lado, pts["aproximar"])
        base = mano.linea_base(n=5)
        self.vig.manos_vigiladas = {lado}
        self._a("DESCENSO", "despacio hasta notar la mesa con la yema", {"base_gf": base.round(0).tolist()})
        contacto = {"t": None, "f": 0.0, "tactil": 0.0}

        def notar():
            e = mano.leer()
            f = float(e.fuerza[3] - e.base[3])
            t = mano.tactil_yema()
            if f > self.c["umbral_gf"] or t > self.c.get("umbral_tactil", 60):
                contacto.update(t=time.monotonic(), f=f, tactil=t)
                raise Contacto()
        try:
            self._mover(lado, pts["bajar"], extra=notar)
            resultado = "sin contacto"
        except Contacto:
            self.brazo.congelar()
            resultado = "tocada"
        q_contacto = self.q(lado)
        e = mano.leer()
        f_max = float(max(contacto["f"], e.fuerza[3] - e.base[3]))
        self._a("TOCAR", resultado, {"fuerza_gf": round(f_max), "tactil": contacto["tactil"]})
        if self._ultimo_fotograma is not None:
            self.reg.imagen(f"diana_{n}", self._ultimo_fotograma.color)
        self.vig.manos_vigiladas = set()
        self._a("VOLVER", "subir y a la observacion")
        self._mover(lado, pts["subir"])
        self._mover(lado, pts["observacion"])
        self._plan_activo = None
        return {"diana": list(xy), "mano": lado, "resultado": resultado, "fuerza_gf": round(f_max),
                "q_contacto": q_contacto.round(4).tolist()}

    # --- niveles 2 y 3 ----------------------------------------------------------------------------------
    def _niveles_objetos(self):
        pendientes = dict(self.asignacion)
        intentos = {k: 0 for k in pendientes}
        while pendientes:
            esc, f = self.observar()
            vistos = {o.clase: o for o in esc.objetos if o.clase in pendientes and o.confianza >= self.cfg["percepcion"]["confianza_min"]}
            if not vistos:
                self._a("PLANIFICAR", f"no veo ninguno de {sorted(pendientes)}")
                break
            hecho = False
            # primero los mas faciles: los que tienen plan; entre ellos, el mas cercano a su mano
            for clase, o in sorted(vistos.items(), key=lambda kv: kv[1].p[0]):
                bandeja = pendientes[clase]
                lado = self.pl.lado_de_bandeja(bandeja)
                plan = self.pl.coger(o, bandeja, self.q(lado), lado)
                if not plan.comprobado:
                    self._a("PLANIFICAR", f"{clase} -> {bandeja}: {plan.motivo}", {"rechazos": plan.detalle.get("rechazos", [])})
                    continue
                ok = self._coger(plan, f)
                intentos[clase] += 1
                self.resultados.append({"objeto": clase, "bandeja": bandeja, "mano": lado,
                                        "resultado": "colocado" if ok else "fallido", "intento": intentos[clase],
                                        "agarre": plan.detalle.get("agarre")})
                if ok or intentos[clase] >= self.s.get("intentos_por_objeto", 2):
                    pendientes.pop(clase)
                hecho = True
                break
            if not hecho:
                self._a("PLANIFICAR", f"ningun objeto con plan valido: {sorted(vistos)}")
                break

    def _coger(self, plan, f):
        lado, mano, ag = plan.mano, self.manos[plan.mano], plan.agarre
        clase = plan.objeto.clase
        self._plan_activo = plan
        self._a("PLANIFICAR", f"{clase} -> bandeja {plan.bandeja} con la mano {lado}", plan.detalle)
        if f is not None:
            self.reg.imagen(f"agarre_{clase}_antes", f.color)
        mano.preforma(ag.preforma, vigilar=self.comprobar)
        nombres = [p.nombre for p in plan.puntos_de_paso]
        k = nombres.index("agarrar")
        self._a("MOVER", f"a {clase}")
        for p in plan.puntos_de_paso[:k]:
            self._mover(lado, p)
        self._a("DESCENSO", "al agarre")
        self._mover(lado, plan.puntos_de_paso[k])
        self.vig.manos_vigiladas = {lado}
        self._a("CERRAR", f"FORCE_SET {ag.force_set}")
        e = mano.cerrar(ag.force_set, vigilar=self.comprobar)
        ok, det = mano.agarre(e, ag.force_set)
        self._a("CERRAR", "agarre comprobado" if ok else "agarre fallido",
                {"dedos": {int(k): v for k, v in det.items()}, "fuerza_gf": (e.fuerza - e.base).round(0).tolist(),
                 "angulo": e.angulo.round(0).tolist()})
        fot = self.fuente.siguiente(0.5) if hasattr(self.fuente, "siguiente") else None
        if fot is not None:
            self.reg.imagen(f"agarre_{clase}_cerrado", fot.color)
        if not ok:
            mano.abrir(vigilar=self.comprobar)
            self.vig.manos_vigiladas = set()
            self._a("VOLVER", "agarre fallido: abrir, subir y a la observacion")
            self._mover(lado, plan.puntos_de_paso[k + 1])
            self._mover(lado, plan.puntos_de_paso[-1])
            self._plan_activo = None
            return False
        # FORCE_SET y la consigna cerrada cada 1 s; la lectura, mas a menudo que mano_max_s del vigilante (con la
        # mano vigilada, una lectura mas vieja que eso es "mano sin respuesta")
        t_sujetar, t_leer = [time.monotonic()], [time.monotonic()]
        cada_leer = min(0.2, self.vig.mano_max_s / 2)

        def sujetar():
            ahora = time.monotonic()
            if ahora - t_sujetar[0] > 1.0:
                mano.sujetar(ag.force_set)
                t_sujetar[0] = ahora
            if ahora - t_leer[0] > cada_leer:
                t_leer[0] = ahora
                e2 = mano.leer()
                fn = (e2.fuerza - e2.base)[:5]
                if float(fn.sum()) < 0.3 * float((e.fuerza - e.base)[:5].sum()):
                    raise Parada(f"{clase} se escapa de la mano: {fn.round(0).tolist()} gf")
        mano.leer()                          # la foto y el registro de arriba han envejecido la ultima lectura
        self._a("TRANSPORTE", f"a la bandeja {plan.bandeja}")
        k_suelta = nombres.index("bajar_a_bandeja")
        for p in plan.puntos_de_paso[k + 1:k_suelta + 1]:
            self._mover(lado, p, extra=sujetar)
        self._a("SOLTAR", f"{clase} en la bandeja {plan.bandeja}")
        mano.abrir(vigilar=self.comprobar)
        self.vig.manos_vigiladas = set()
        self._a("VOLVER", "retirar y a la observacion")
        for p in plan.puntos_de_paso[k_suelta + 1:]:
            self._mover(lado, p)
        self._plan_activo = None
        return True

    # --- salida segura ----------------------------------------------------------------------------------
    def salida_segura(self, motivo):
        """Congelar, abrir la mano, subir a la altura de seguridad, (salir por la observacion al reposo) y solo
        entonces bajar el peso. Si algo grave pasa a mitad (lowstate o motor), directo a la rampa de bajada."""
        self._a("SALIDA_SEGURA", motivo)
        self.brazo.congelar()
        if self.brazo.peso <= 0.01:
            self._a("FIN", "salida segura sin peso: nada que mover")
            return
        for lado, mano in self.manos.items():
            try:
                mano.abrir()
            except Exception as e:                     # sigue: subir y bajar el peso no depende de la mano
                self._a("SALIDA_SEGURA", f"la mano {lado} no abre: {e}")

        def grave():
            if self.robot.edad_lowstate() > self.s["lowstate_max_s"]:
                raise Parada("rt/lowstate perdido en la salida segura")
            if self.robot.estado().motores_en_fallo:
                raise Parada("motor en fallo en la salida segura")
        try:
            for lado in LADOS:
                q = self.q(lado)
                M = self.pl.cin.fk(lado, q, "mano")
                if not self.pl.sobre_mesa(M[:3, 3], 0.05):
                    continue
                h = float(self.pl.plano.altura(M[:3, 3]))
                altura = self.cfg["planificacion"]["altura_segura_m"]
                if h >= altura:
                    continue
                R = M[:3, :3]
                t = TareaIK(posicion=M[:3, 3] + (altura - h + 0.02) * self.pl.plano.n, tol_pos=0.01,
                            bandas=[Banda(np.array([1.0, 0, 0]), R[:, 0], math.cos(math.radians(15)), 1.0),
                                    Banda(np.array([0, 1.0, 0]), R[:, 1], math.cos(math.radians(15)), 1.0)])
                r = self.pl.cin.ik(lado, t, q0=q)
                if r.ok:
                    self._a("SALIDA_SEGURA", f"subir la mano {lado} a {altura * 100:.0f} cm")
                    self.brazo.ir_a(self.brazo.pose_brazo(lado, r.q), None, grave)
            if self.s.get("salida_por_observacion", True):
                for lado in LADOS:
                    self._mover_seguro(lado, self.pl.observacion[lado], grave)
                # al reposo solo los brazos que llegaron a la observacion: desde otro sitio barrerian la mesa
                pose, dur = {}, 0.0
                for lado in LADOS:
                    if np.abs(self.q(lado) - self.pl.observacion[lado]).max() < 0.1:
                        p = self.pl.entre_reposo_y_observacion(lado, False)
                        if p.comprobado:
                            pose.update(self.brazo.pose_brazo(lado, p.puntos_de_paso[-1].q))
                            dur = max(dur, p.puntos_de_paso[-1].duracion)
                    else:
                        self._a("SALIDA_SEGURA", f"brazo {lado} fuera de la observacion: no va al reposo")
                if pose:
                    self._a("SALIDA_SEGURA", f"observacion -> reposo ({', '.join(sorted({l for l in LADOS if self.brazo.pose_brazo(l, self.q(l)).keys() & pose.keys()}))})")
                    self.brazo.ir_a(pose, dur, grave)
        except Parada as e:
            self._a("SALIDA_SEGURA", f"{e}: directo a bajar el peso")
        self._a("SALIDA_SEGURA", f"peso -> 0 en {self.brazo.t_salida:.0f} s")
        self.brazo.rampa_peso(0.0)
        self._a("FIN", "peso 0.00 tras la salida segura")

    def _mover_seguro(self, lado, q_obj, grave):
        """A la observacion solo por tramos que pasan la comprobacion: directo o, si no, pasando por un punto alto
        del plan en curso (sobre el objeto, la bandeja o la diana), el mas cercano primero. Directo, desde encima
        de la mesa, la muneca puede girar bajando la mano hasta la mesa. Si ninguno pasa, se queda arriba."""
        q = self.q(lado)
        if np.abs(q - q_obj).max() < 0.05:
            return
        vias = [None]
        if self._plan_activo is not None and self._plan_activo.mano == lado:
            altos = [p.q for p in self._plan_activo.puntos_de_paso if p.nombre in PASOS_ALTOS]
            vias += sorted(altos, key=lambda v: float(np.abs(v - q).max()))
        motivos = []
        for via in vias:
            pts, qa = [], q
            for nombre, qb in ([] if via is None else [("via", via)]) + [("observacion", q_obj)]:
                pts.append(PuntoPaso(q=qb, duracion=self.pl.duracion(qa, qb), nombre=nombre))
                qa = qb
            plan = self.pl.comprobar(Plan(objeto=None, mano=lado, tarea="observacion", puntos_de_paso=pts),
                                     q_inicio=q, sin_objetos=True)
            if plan.comprobado:
                self._a("SALIDA_SEGURA", f"brazo {lado} a la observacion" +
                        ("" if via is None else f" por un punto alto del plan (directo: {motivos[0]})"))
                for p in plan.puntos_de_paso:
                    self.brazo.ir_a(self.brazo.pose_brazo(lado, p.q), p.duracion, grave)
                return
            motivos.append(plan.motivo)
        self._a("SALIDA_SEGURA", f"brazo {lado}: no hay camino limpio a la observacion ({motivos[0]}); se queda")
