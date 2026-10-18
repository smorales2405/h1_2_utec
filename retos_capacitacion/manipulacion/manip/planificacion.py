"""Planificacion (seccion 5 del PDF): objeto + estado de los brazos -> mano, agarre, puntos de paso y suelta, en
articulares, dentro de los limites y comprobados contra el gemelo (sin robot).

Requisitos del PDF que se comprueban en cada plan:
  - ningun punto por debajo de la altura de seguridad sobre la mesa, salvo el descenso final (al objeto o a la
    bandeja): se mira el punto de agarre (o la yema) a lo largo de cada tramo, no solo en los puntos de paso
  - dentro de los limites del URDF (web n URDF, con margen) y sin colisiones en el gemelo: la mano, el brazo y
    el objeto que se lleva contra la mesa, el labio, las asas, las bandejas, el cuerpo y el otro brazo
  - duraciones con la regla de brazo.py (>= 8 s, pico de 0.5 rad/s), SIN la escala de tiempo: la aplica
    brazo.py, una sola vez (aplicarla tambien aqui duplicaba los tiempos: tramos de 32 s con la escala 2)
Cada objeto lo coge la mano del lado de su bandeja (la otra no llega a esas bandejas: mapa de alcance).

Agarres (objetos.yaml): "arriba" con la palma abajo; para cajas, el ancho de la mano (menique -> indice) a lo
largo del lado largo, asi los dedos cierran a lo ancho; "lado" (el cilindro de pie) con la palma vertical y
los dedos inclinados, bajando a `entrada_lado_m` del objeto y entrando en horizontal.
"""

import math

import numpy as np

from .brazo import duracion_movimiento
from .cinematica import (EJE_DEDO_YEMA, EJE_DEDOS, EJE_PALMA, Banda, TareaIK, dedos_inclinados, guinada,
                         palma_abajo, palma_vertical, T_punto)
from .geometria import Plano, aplicar
from .mensajes import Agarre, Plan, PuntoPaso

# tramos que forman el descenso final (se permiten bajo la altura de seguridad)
DESCENSO = {"aproximar", "entrar", "agarrar", "levantar", "bajar_a_bandeja", "retirar_de_bandeja", "bajar", "subir"}


class Planificador:
    def __init__(self, cfg, montaje, objetos, cin, gemelo=None, reposo=None, observacion=None, escala_tiempo=1.0):
        """gemelo: (sim.escena.Estado, sim.colisiones.Colisiones) o None (sin comprobar colisiones).
        reposo: {lado: q7} con los brazos colgando (la postura de la marcha, peso 0).
        observacion: {lado: q7} con las manos fuera de la mesa y de la vista (config/poses_brazos.yaml): cada
        plan de trabajo empieza y acaba en ella; reposo <-> observacion son planes aparte."""
        self.cfg, self.p, self.c = cfg, cfg["planificacion"], cfg["contacto"]
        self.montaje, self.objetos = montaje, objetos
        self.cin = cin
        self.gemelo = gemelo
        pl = montaje["mesa"]["plano"]
        self.plano = Plano(pl["n"], pl["d"])
        self.reposo = {k: np.asarray(v, float) for k, v in (reposo or {}).items()}
        self.observacion = {k: np.asarray(v, float) for k, v in (observacion or {}).items()}
        b = cfg["brazo"]
        self.t_min, self.vel_max = b["t_min_mov_s"], b["vel_max_rad_s"]
        self.escala = escala_tiempo     # solo para informar de lo que tardara el plan (la aplica brazo.py)

    # --- geometria ---------------------------------------------------------------------------------
    def punto(self, xy, altura):
        p = np.array([xy[0], xy[1], 0.0])
        z = -(self.plano.n[0] * p[0] + self.plano.n[1] * p[1] + self.plano.d) / self.plano.n[2]
        return np.array([p[0], p[1], z]) + altura * self.plano.n

    def sobre_mesa(self, pt, margen=0.05):
        """El punto (torso) cae en la vertical de la mesa (superficie util ampliada `margen`)."""
        from .geometria import inv
        T_mesa = np.array(self.montaje["mesa"]["T_torso_mesa"], float)
        x, y, _ = aplicar(inv(T_mesa), pt)
        sup = self.montaje["mesa"]["superficie"]
        return sup["x"][0] - margen <= x <= sup["x"][1] + margen and sup["y"][0] - margen <= y <= sup["y"][1] + margen

    def lado_de_bandeja(self, bandeja):
        from .montaje import bandeja_real
        return "izq" if bandeja_real(self.montaje["bandejas"][bandeja])[0][1] >= 0 else "der"

    def duracion(self, qa, qb):
        """La duracion base del tramo (>= t_min, pico de vel_max). brazo.ir_a la recibe y la multiplica por la
        escala de tiempo: aqui no se multiplica."""
        return duracion_movimiento(float(np.abs(np.asarray(qb) - np.asarray(qa)).max()), self.t_min, self.vel_max)

    # --- tareas ----------------------------------------------------------------------------------------
    def _tareas_agarre(self, lado, clase, centro_xy, yaw, altura, tipo):
        """Las variantes de orientacion para coger `clase` en (centro_xy, yaw) a `altura` con el agarre `tipo`."""
        o = self.objetos["objetos"][clase]
        punto = T_punto(self.objetos["agarres"][tipo]["centro"], lado)
        pos = self.punto(centro_xy, altura)
        tareas = []
        if tipo == "arriba":
            base = [palma_abajo(lado, self.p["palma_abajo_tol_deg"])]
            if o["forma"] == "caja":
                sim = o.get("simetria_deg", 180)
                n = max(1, int(round(360 / max(sim, 1))))
                # primero bien alineada; si no hay plan, hasta la tolerancia del objeto (a 30 grados, 4 cm de
                # ancho son 4.6 de cuerda: la mano sigue abarcandolo)
                tols = sorted({self.p["guinada_tol_deg"], o.get("guinada_tol_deg", self.p["guinada_tol_deg"])})
                for tol in tols:
                    for k in range(n * (2 if sim >= 180 else 1)):
                        a = yaw + k * math.radians(sim if sim < 180 else 180)
                        tareas.append(TareaIK(posicion=pos, punto=punto,
                                              bandas=base + [guinada([math.cos(a), math.sin(a)], tol)]))
            else:
                tareas.append(TareaIK(posicion=pos, punto=punto, bandas=base))
        else:
            lo, hi = self.p["dedos_abajo_deg"]
            tareas.append(TareaIK(posicion=pos, punto=punto,
                                  bandas=[palma_vertical(lado, 25), dedos_inclinados(lo, hi)]))
        return tareas

    def _ik(self, lado, tarea, q0):
        r = self.cin.ik(lado, tarea, q0=q0)
        return r.q if r.ok else None

    def _cadena(self, lado, q0, pasos, q_agarre=None):
        """pasos: [(nombre, TareaIK)] resueltos en orden, cada uno desde el anterior (continuidad). Con q_agarre,
        los pasos antes del agarre parten de el (misma rama de la IK) y el agarre es exactamente q_agarre."""
        puntos, q = [], np.asarray(q0, float)
        for nombre, t in pasos:
            semilla = q_agarre if (q_agarre is not None and nombre in ("sobre", "aproximar", "entrar")) else q
            qn = q_agarre.copy() if (q_agarre is not None and nombre == "agarrar") else self._ik(lado, t, semilla)
            if qn is None:
                return None, nombre
            puntos.append(PuntoPaso(q=qn, duracion=self.duracion(q, qn), nombre=nombre))
            q = qn
        return puntos, None

    # --- planes ----------------------------------------------------------------------------------------
    def tocar(self, lado, diana_xy, q0):
        """Nivel 1: la yema del indice sobre la diana. El descenso final lo hace el supervisor despacio, con
        deteccion de contacto, hasta `bajar` como mucho. Primero con el indice mas vertical (el resto de la mano
        queda mas alto sobre la mesa); si no hay plan, con el minimo de la configuracion."""
        self.mano_gemelo(lado, self.objetos["preformas"]["senalar"])
        plan = None
        for grados in (65, 50, self.p["yema_abajo_tol_deg"]):
            abajo = dedos_inclinados(grados, 90, EJE_DEDO_YEMA[lado])
            pasos = [("sobre", TareaIK(posicion=self.punto(diana_xy, self.p["altura_segura_m"]), marco="yema", bandas=[abajo])),
                     ("aproximar", TareaIK(posicion=self.punto(diana_xy, self.p["altura_aproximacion_m"]), marco="yema", bandas=[abajo])),
                     # hasta 2 cm bajo la mesa estimada: el descenso para al notar el contacto (si la mesa esta
                     # algo mas baja de lo calibrado, sin esto no llegaria a tocarla)
                     ("bajar", TareaIK(posicion=self.punto(diana_xy, -min(0.02, self.c["bajada_max_m"])), marco="yema", bandas=[abajo])),
                     ("subir", TareaIK(posicion=self.punto(diana_xy, self.p["altura_segura_m"]), marco="yema", bandas=[abajo]))]
            puntos, falla = self._cadena(lado, q0, pasos)
            plan = Plan(objeto=None, mano=lado, tarea="tocar", puntos_de_paso=puntos or [],
                        detalle={"diana": list(diana_xy), "indice_abajo_deg": grados})
            if puntos is None:
                plan.motivo = f"sin IK en '{falla}'"
                continue
            self._añadir_reposo(plan, marco="yema")
            plan = self.comprobar(plan, marco="yema", q_inicio=q0)
            if plan.comprobado:
                return plan
        return plan

    def coger(self, objeto, bandeja, q0, lado=None):
        """Niveles 2 y 3: coger `objeto` (mensajes.Objeto) y dejarlo en `bandeja`. Prueba los agarres del objeto
        (objetos.yaml, en orden), sus orientaciones y varias soluciones de la IK del agarre; devuelve el primer
        plan que pasa la comprobacion, o el ultimo rechazado con su motivo."""
        lado = lado or self.lado_de_bandeja(bandeja)
        o = self.objetos["objetos"][objeto.clase]
        tipos = o["agarre"] if isinstance(o["agarre"], list) else [o["agarre"]]
        alt = o.get("altura_agarre_m", 0.035)
        hs, ht = self.p["altura_segura_m"], self.p.get("altura_transporte_m", self.p["altura_segura_m"])
        ha, hsu = self.p["altura_aproximacion_m"], self.p["altura_suelta_m"]
        xy = objeto.p[:2]
        from .montaje import bandeja_real
        sobre_b = bandeja_real(self.montaje["bandejas"][bandeja])[0]     # la medida con [g], o el objetivo
        self.mano_gemelo(lado, self.objetos["preformas"][o["preforma"]])
        ultimo, intentos, rechazos = None, 0, []
        for tipo in tipos:
            for t_agarre in self._tareas_agarre(lado, objeto.clase, xy, objeto.yaw, alt, tipo):
                for q_ag in self._variantes(lado, t_agarre, q0):
                    intentos += 1
                    # misma orientacion de la mano en todo el plan: la del agarre
                    M = self.cin.fk(lado, q_ag, "mano")
                    fija = [Banda(EJE_PALMA[lado], M[:3, :3] @ EJE_PALMA[lado], math.cos(math.radians(12)), 1.0),
                            Banda(EJE_DEDOS, M[:3, :3] @ EJE_DEDOS, math.cos(math.radians(12)), 1.0)]
                    punto = t_agarre.punto

                    def t(xy_, h):
                        return TareaIK(posicion=self.punto(xy_, h), punto=punto, bandas=fija)
                    if tipo == "lado":
                        # entrar en horizontal desde fuera: retroceder a lo largo de la palma
                        d = -(M[:3, :3] @ EJE_PALMA[lado])[:2]
                        d = d / max(np.linalg.norm(d), 1e-6) * self.p["entrada_lado_m"]
                        pasos = [("sobre", t(xy + d, ht)), ("aproximar", t(xy + d, alt)), ("entrar", t(xy, alt))]
                    else:
                        pasos = [("sobre", t(xy, ht)), ("aproximar", t(xy, ha))]
                    pasos += [("agarrar", t(xy, alt)), ("levantar", t(xy, ht)), ("sobre_bandeja", t(sobre_b, ht)),
                              ("bajar_a_bandeja", t(sobre_b, hsu)), ("retirar_de_bandeja", t(sobre_b, ht))]
                    puntos, falla = self._cadena(lado, q0, pasos, q_agarre=q_ag)
                    if puntos is None:
                        ultimo = ultimo or Plan(objeto=objeto, mano=lado, tarea="coger", puntos_de_paso=[],
                                                bandeja=bandeja, motivo=f"{tipo}: sin IK en '{falla}'")
                        continue
                    k_ag = [p.nombre for p in puntos].index("agarrar")
                    agarre = Agarre(preforma=self.objetos["preformas"][o["preforma"]], force_set=o["force_set"],
                                    speed=self.cfg["mano"]["speed_cierre"], T_mano=self.cin.fk(lado, puntos[k_ag].q, "mano"))
                    plan = Plan(objeto=objeto, mano=lado, tarea="coger", puntos_de_paso=puntos, agarre=agarre,
                                bandeja=bandeja, detalle={"clase": objeto.clase, "agarre": tipo})
                    self._añadir_reposo(plan, punto=punto)
                    plan = self.comprobar(plan, sujeto_desde="levantar", sujeto_hasta="bajar_a_bandeja",
                                          objeto_ignorado=f"obj_{objeto.clase}", q_inicio=q0)
                    if plan.comprobado:
                        plan.detalle["intentos"] = intentos
                        return plan
                    plan.motivo = f"{tipo}: {plan.motivo}"
                    rechazos.append(plan.motivo)
                    ultimo = plan
        ultimo = ultimo or Plan(objeto=objeto, mano=lado, tarea="coger", puntos_de_paso=[], bandeja=bandeja,
                                motivo="sin agarre posible (IK)")
        ultimo.detalle["rechazos"] = rechazos
        return ultimo

    def _variantes(self, lado, tarea, q0, n=3):
        """Hasta n soluciones distintas de la IK (la mas cercana a q0 primero)."""
        sol = [r for r in self.cin.ik(lado, tarea, q0=q0, todas=True) if r.ok]
        sol.sort(key=lambda r: float(np.abs(r.q - q0).max()))
        distintas = []
        for r in sol:
            if all(np.abs(r.q - d).max() > 0.3 for d in distintas):
                distintas.append(r.q)
            if len(distintas) >= n:
                break
        return distintas

    def mano_gemelo(self, lado, angle_set):
        """La mano del gemelo en la preforma (ANGLE_SET), para comprobar colisiones con los dedos como iran."""
        if self.gemelo is None:
            return
        from sim.escena import angle_set_a_rad
        a = [1000 if v is None or v < 0 else v for v in angle_set]
        self.gemelo[0].poner_mano(lado, angle_set_a_rad(a))

    def poner_escena(self, escena):
        """Los objetos percibidos, en el gemelo: los demas objetos son obstaculos al planificar."""
        if self.gemelo is None:
            return
        est = self.gemelo[0]
        vistos = {o.clase: o for o in escena.objetos if o.clase in self.objetos["objetos"]}
        for clase in self.objetos["objetos"]:
            o = vistos.get(clase)
            if o is None:
                est.poner_objeto(clase)
            else:
                M = self._T_objeto(o)
                est.poner_objeto(clase, M[:3, 3], M[:3, :3])

    def _añadir_reposo(self, plan, marco="mano", punto=None):
        """Los planes de trabajo acaban en la pose de observacion (si se dio), desde donde se percibe otra vez."""
        if plan.mano in self.observacion and plan.puntos_de_paso:
            q = plan.puntos_de_paso[-1].q
            q_obs = self.observacion[plan.mano]
            plan.puntos_de_paso.append(PuntoPaso(q=q_obs, duracion=self.duracion(q, q_obs), nombre="observacion"))

    def entre_reposo_y_observacion(self, lado, a_observacion=True):
        """El plan reposo -> observacion (o al reves), comprobado como cualquier otro."""
        qa, qb = (self.reposo[lado], self.observacion[lado]) if a_observacion else (self.observacion[lado], self.reposo[lado])
        plan = Plan(objeto=None, mano=lado, tarea="observacion" if a_observacion else "reposo",
                    puntos_de_paso=[PuntoPaso(q=qb, duracion=self.duracion(qa, qb), nombre="observacion" if a_observacion else "reposo")])
        return self.comprobar(plan, q_inicio=qa)

    # --- comprobacion contra el modelo ------------------------------------------------------------------
    def comprobar(self, plan, marco="mano", sujeto_desde=None, sujeto_hasta=None, objeto_ignorado=None, q_inicio=None,
                  sin_objetos=False):
        """Recorre cada tramo (lineal en articulares, como lo hace brazo.py), desde q_inicio (donde esta el brazo)
        si se da, y mira limites, altura sobre la mesa y colisiones. sin_objetos: los objetos no cuentan (la salida
        segura: alguno puede estar en la mano)."""
        self._sin_objetos = sin_objetos
        lado, puntos = plan.mano, plan.puntos_de_paso
        hs = self.p["altura_segura_m"]
        n = self.p["muestras_tramo"]
        punto_ag = None
        if plan.tarea == "coger":
            punto_ag = T_punto(self.objetos["agarres"][plan.detalle["agarre"]]["centro"], lado)
        nombres = [p.nombre for p in puntos]
        k_desde = nombres.index(sujeto_desde) if sujeto_desde in nombres else None
        k_hasta = nombres.index(sujeto_hasta) if sujeto_hasta in nombres else None
        h_min_fuera, d_min = np.inf, np.inf
        q_prev = None if q_inicio is None else np.asarray(q_inicio, float)
        detalle = []
        for k, p in enumerate(puntos):
            if not self.cin.dentro_de_limites(lado, p.q, 1e-6):
                plan.motivo = f"'{p.nombre}' fuera de limites"
                return plan
            if q_prev is None:
                q_prev = p.q
                continue
            descenso = p.nombre in DESCENSO or (k > 0 and puntos[k - 1].nombre in DESCENSO and p.nombre in DESCENSO)
            sujeto = k_desde is not None and k_hasta is not None and k_desde <= k <= k_hasta
            for s in np.linspace(0, 1, n + 1)[1:]:
                q = q_prev + (p.q - q_prev) * s
                M = self.cin.fk(lado, q, "yema" if marco == "yema" else "mano")
                pt = M[:3, 3] if marco == "yema" else aplicar(M, punto_ag if punto_ag is not None else np.zeros(3))
                h = float(self.plano.altura(pt))
                if not descenso and h < hs - 0.01 and self.sobre_mesa(pt):
                    plan.motivo = f"tramo a '{p.nombre}' baja a {h * 100:.1f} cm de la mesa (seguridad {hs * 100:.0f})"
                    return plan
                if not descenso and self.sobre_mesa(pt):
                    h_min_fuera = min(h_min_fuera, h)
                if self.gemelo is not None:
                    d = self._colision(lado, q, M if sujeto else None, plan, objeto_ignorado, p.nombre)
                    d_min = min(d_min, d[0])
                    if d[0] < d[1]:
                        plan.motivo = f"tramo a '{p.nombre}': {d[2]}"
                        return plan
            detalle.append((p.nombre, round(p.duracion * self.escala, 1)))     # lo que tardara de verdad
            q_prev = p.q
        plan.comprobado = True
        plan.detalle.update({"tramos": detalle, "altura_min_fuera_del_descenso_m": round(float(h_min_fuera), 3),
                             "distancia_min_m": round(float(d_min), 4) if np.isfinite(d_min) else None,
                             "duracion_s": round(sum(p.duracion for p in puntos) * self.escala, 1)})
        return plan

    def _colision(self, lado, q, M_mano_sujeto, plan, objeto_ignorado, nombre):
        """(distancia minima, umbral, texto) con el brazo en q; con el objeto en la mano si M_mano_sujeto."""
        est, col = self.gemelo
        est.poner_brazo(lado, q)
        otro = "der" if lado == "izq" else "izq"
        quieto = self.observacion if plan.tarea in ("tocar", "coger") else self.reposo
        if otro in quieto:
            est.poner_brazo(otro, quieto[otro])
        ignorar = set()
        cuerpo_obj = None
        if plan.objeto is not None:
            cuerpo_obj = f"obj_{plan.objeto.clase}"
            if M_mano_sujeto is not None:
                # el objeto va en la mano: donde lo pone el agarre, y choca como parte del robot
                T_rel = np.linalg.inv(plan.agarre.T_mano) @ self._T_objeto(plan.objeto)
                M = M_mano_sujeto @ T_rel
                est.poner_objeto(plan.objeto.clase, M[:3, 3], M[:3, :3])
                col.como_robot(cuerpo_obj, True)
            else:
                est.poner_objeto(plan.objeto.clase, self._T_objeto(plan.objeto)[:3, 3], self._T_objeto(plan.objeto)[:3, :3])
                col.como_robot(cuerpo_obj, False)
        if objeto_ignorado and nombre in DESCENSO | {"sobre_bandeja", "observacion"}:
            ignorar.add(objeto_ignorado)
        umbral = self.p["holgura_agarre_m"] if nombre in {"aproximar", "entrar", "agarrar", "levantar", "bajar", "subir"} else self.p["holgura_m"]
        if M_mano_sujeto is not None and nombre in {"bajar_a_bandeja", "retirar_de_bandeja"}:
            umbral = self.p["holgura_agarre_m"]
        if getattr(self, "_sin_objetos", False):
            ignorar |= {f"obj_{c}" for c in self.objetos["objetos"]}
        h = col.revisar(ignorar=ignorar, mano_con=(cuerpo_obj if M_mano_sujeto is not None else None))
        if plan.tarea == "tocar" and nombre in ("bajar", "subir"):
            # el descenso del nivel 1 acaba con la yema en la mesa: ese contacto es el buscado
            yema = f"{'left' if lado == 'izq' else 'right'}_index_2"
            h = [x for x in h if {x[0], x[1]} != {yema, "mesa"}]
        if cuerpo_obj is not None:
            col.como_robot(cuerpo_obj, False)
        if not h:
            return col.margen, umbral, ""
        a, b, d = h[0]
        return d, umbral, f"{a} / {b} a {d * 1000:+.0f} mm"

    def _T_objeto(self, objeto):
        from .geometria import T, rpy_a_R
        R = np.column_stack(self.plano.base((math.cos(objeto.yaw), math.sin(objeto.yaw), 0.0))[1:] + (self.plano.n,))
        return T(R, objeto.p)
