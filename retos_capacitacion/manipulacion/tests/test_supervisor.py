"""Supervisor de punta a punta en el mundo simulado (sim/simulacion.py): estados, contacto, agarre, registro y
salida segura, con el mismo cableado que manipulacion.py. Tiempo acelerado; se salta sin OpenGL."""

import copy
import os
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from manip import config
from test_gemelo import gl

import manipulacion


def _args(tmp_path, **kw):
    a = SimpleNamespace(nivel=1, simulacro=False, sim=True, escala_tiempo=1.0, dianas=None, asignacion={},
                        colocacion={}, datos=str(tmp_path))
    a.__dict__.update(kw)
    return a


def _cfg():
    cfg = copy.deepcopy(config.cargar())
    cfg["mano"]["t_base_s"] = 0.0          # la mano simulada no tiene deriva del cero
    return cfg


@gl
def test_nivel1_toca_las_dianas(tmp_path):
    t = manipulacion.armar(_args(tmp_path, dianas=[(0.46, 0.05), (0.47, -0.06)]), _cfg(), log=lambda *x: None)
    motivo = manipulacion.correr(t, confirmar=False)
    assert motivo == "completado"
    assert [r["resultado"] for r in t.sup.resultados] == ["tocada", "tocada"]
    assert all(0 < r["fuerza_gf"] <= 300 for r in t.sup.resultados)
    assert t.brazo.peso == 0.0
    assert {"lowstate.csv", "manos.csv", "eventos.csv", "resumen.json"} <= set(os.listdir(t.reg.carpeta))


@gl
def test_medir_mesa_en_el_gemelo(tmp_path):
    """--medir-mesa (§53): un toque por mano hasta que lo nota la fuerza; en el gemelo la mesa esta en su plano, asi
    que la correccion sale ~0; en la simulacion no se guarda nada."""
    import manip.config as mc
    antes = open(mc.RUTA_MONTAJE).read()
    t = manipulacion.armar(_args(tmp_path, nivel=0), _cfg(), log=lambda *x: None)
    assert manipulacion.correr(t, confirmar=False) == "completado"
    m = t.sup.medida_mesa
    assert [(p["mano"], p["vale"]) for p in m["puntos"]] == [("izq", True), ("der", True)]
    assert abs(m["correccion_m"]) < 0.004 and m["diferencia_mm"] < 4
    assert open(mc.RUTA_MONTAJE).read() == antes


@gl
def test_nivel1_sin_fijar(tmp_path):
    """Con un arnes.py que no acepta fijar, el toque sigue sin guiar: la bajada y la vuelta de una vez (punto a punto,
    cada tramo duraba como poco t_min_mov_s: 3 min sobre la diana en el robot, §51) y se toca igual."""
    t = manipulacion.armar(_args(tmp_path, dianas=[(0.46, 0.05)]), _cfg(), log=lambda *x: None)
    movidas = []
    mover = t.sup._mover
    t.sup._mover = lambda lado, p, **kw: (movidas.append(p.nombre), mover(lado, p, **kw))[1]
    t.brazo.fijar = lambda pose: False
    motivo = manipulacion.correr(t, confirmar=False)
    assert motivo == "completado"
    assert [r["resultado"] for r in t.sup.resultados] == ["tocada"]
    assert movidas.count("bajar") == 1 and movidas.count("subir") == 1


@gl
def test_nivel2_coloca_en_sus_bandejas(tmp_path):
    a = _args(tmp_path, nivel=2, asignacion={"esfera": "A", "cubo": "C"},
              colocacion={"esfera": ((0.43, 0.05), 0.0), "cubo": ((0.47, -0.07), 0.3)})
    t = manipulacion.armar(a, _cfg(), log=lambda *x: None)
    motivo = manipulacion.correr(t, confirmar=False)
    assert motivo == "completado"
    final = {k: t.mundo.bandeja_de(k) for k in ("esfera", "cubo")}
    assert final == {"esfera": "A", "cubo": "C"}
    assert t.brazo.peso == 0.0


@gl
def test_coger_en_linea_recta(tmp_path):
    """§53: los tramos finos de coger (al agarre, levantar, bajar a la bandeja y retirarse) son rectas del punto de
    agarre con una postura por cm (entre dos IK sueltas se apartaba hasta 56 mm), y el agarre acaba justo en el."""
    from manip.cinematica import T_punto
    from manip.geometria import aplicar
    a = _args(tmp_path, nivel=2, asignacion={"cubo": "C"}, colocacion={"cubo": ((0.47, -0.07), 0.3)})
    t = manipulacion.armar(a, _cfg(), log=lambda *x: None)
    try:
        pl = t.sup.pl
        esc, _ = t.sup.observar()
        o = next(x for x in esc.objetos if x.clase == "cubo")
        plan = pl.coger(o, "C", pl.observacion["der"])
        assert plan.comprobado, plan.motivo
        punto = T_punto(pl.objetos["agarres"][plan.detalle["agarre"]]["centro"], "der")
        pts = plan.puntos_de_paso
        for nombre in ("agarrar", "levantar", "bajar_a_bandeja", "retirar_de_bandeja"):
            ks = [k for k, p in enumerate(pts) if p.nombre == nombre]
            assert len(ks) >= 2 and ks == list(range(ks[0], ks[-1] + 1)), nombre
            Q = [pts[ks[0] - 1].q] + [pts[k].q for k in ks]
            P = np.array([aplicar(pl.cin.fk("der", q, "mano"), punto) for q in Q])
            u = (P[-1] - P[0]) / np.linalg.norm(P[-1] - P[0])
            for a_, b_ in zip(Q[:-1], Q[1:]):
                for s_ in np.linspace(0, 1, 5):
                    e = aplicar(pl.cin.fk("der", a_ + s_ * (b_ - a_), "mano"), punto) - P[0]
                    assert np.linalg.norm(e - (e @ u) * u) < 0.0025, nombre
        k_ag = max(k for k, p in enumerate(pts) if p.nombre == "agarrar")
        assert np.allclose(pl.cin.fk("der", pts[k_ag].q, "mano"), plan.agarre.T_mano)
    finally:
        t.mundo.parar()


@gl
def test_salida_segura_por_giro(tmp_path):
    a = _args(tmp_path, nivel=2, asignacion={"esfera": "A"}, colocacion={"esfera": ((0.43, 0.05), 0.0)})
    t = manipulacion.armar(a, _cfg(), log=lambda *x: None)

    def girar_al_cerrar():
        while t.sup.estado not in ("CERRAR", "FIN"):
            time.sleep(0.01)
        t.robot.girar(6.0)
    threading.Thread(target=girar_al_cerrar, daemon=True).start()
    motivo = manipulacion.correr(t, confirmar=False)
    assert motivo.startswith("salida segura") and "giro" in motivo
    assert t.brazo.peso == 0.0
    estados = [e[1] for e in t.reg.resumen["eventos"]]
    assert "SALIDA_SEGURA" in estados and estados[-1] == "FIN"


@gl
def test_salida_segura_en_el_transporte_sale_por_la_observacion(tmp_path):
    """Parada con el objeto en la mano sobre la mesa: el brazo va a la observacion (por un punto alto del plan si
    el tramo directo no esta limpio) y al reposo antes de bajar el peso; ningun brazo se queda sobre la mesa."""
    a = _args(tmp_path, nivel=2, asignacion={"cilindro": "B"}, colocacion={"cilindro": ((0.42, 0.01), 0.0)},
              parada_en="TRANSPORTE")
    t = manipulacion.armar(a, _cfg(), log=lambda *x: None)
    motivo = manipulacion.correr(t, confirmar=False)
    assert motivo.startswith("salida segura") and "TRANSPORTE" in motivo
    motivos = [e[2] for e in t.reg.resumen["eventos"] if e[1] == "SALIDA_SEGURA"]
    assert any(m.startswith("brazo izq a la observacion") for m in motivos)
    assert any(m.startswith("observacion -> reposo") and "izq" in m for m in motivos)
    assert not any("se queda" in m or "fuera de la observacion" in m for m in motivos)
    assert t.brazo.peso == 0.0


@gl
def test_nivel2_agrupa_los_objetos_por_brazo(tmp_path):
    """Opcion b (2026-10-14): todos los objetos de un brazo y luego los del otro, no intercalados."""
    a = _args(tmp_path, nivel=2, asignacion=dict(manipulacion.ASIGNACION_SIM),
              colocacion=dict(manipulacion.COLOCACION_SIM))
    t = manipulacion.armar(a, _cfg(), log=lambda *x: None)
    assert manipulacion.correr(t, confirmar=False) == "completado"
    manos = [r["mano"] for r in t.sup.resultados]
    assert len(manos) == 4 and all(r["resultado"] == "colocado" for r in t.sup.resultados)
    assert manos in (["izq", "izq", "der", "der"], ["der", "der", "izq", "izq"])


@gl
def test_el_toque_con_el_indice_vertical(tmp_path):
    """El toque del nivel 1 (RESULTADOS §42): el indice extendido (senalar) y casi vertical, en toda la zona y con las
    dos manos; la bajada para por la yema medida."""
    import numpy as np
    from manip.cinematica import EJE_DEDO_YEMA, EJE_PALMA
    t = manipulacion.armar(_args(tmp_path, dianas=[(0.46, 0.05)]), _cfg(), log=lambda *x: None)
    pl = t.sup.pl
    for lado, xy in (("izq", (0.44, 0.05)), ("izq", (0.56, 0.20)), ("der", (0.40, -0.20)), ("der", (0.50, -0.02))):
        plan = pl.tocar(lado, xy, pl.observacion[lado])
        assert plan.comprobado, (lado, xy, plan.motivo)
        assert plan.detalle["preforma"] == "senalar"
        q_apr = next(p for p in plan.puntos_de_paso if p.nombre == "aproximar").q
        R = pl.cin_preforma("senalar").fk(lado, q_apr, "yema")[:3, :3]
        bajo = np.degrees(np.arcsin(-(R @ EJE_DEDO_YEMA[lado])[2]))
        assert 55 <= bajo <= 90, (lado, xy, bajo)
        palma = pl.cin_preforma("senalar").fk(lado, q_apr, "mano")[:3, :3] @ EJE_PALMA[lado]
        assert palma[0] < -0.6, (lado, xy, palma)                         # la palma al robot, el dorso al frente
        pl.mano_gemelo(lado, [1000] * 6)
    if t.mundo is not None:
        t.mundo.parar()


@gl
def test_la_bajada_del_nivel1_va_a_su_velocidad(tmp_path):
    """contacto.vel_descenso_m_s manda en la bajada (antes solo la duracion minima: 4-5 mm/s con la escala 2), ya con
    la escala de tiempo; y bajada_max_m limita hasta donde (probar_toque.py: 1 cm bajo la mesa)."""
    cfg = _cfg()
    cfg["contacto"].update(vel_descenso_m_s=0.003, bajada_max_m=0.01)
    cfg["planificacion"]["altura_aproximacion_m"] = 0.04
    t = manipulacion.armar(_args(tmp_path, dianas=[(0.46, 0.05)], escala_tiempo=2.0), cfg, log=lambda *x: None)
    pl = t.sup.pl
    plan = pl.tocar("izq", (0.46, 0.05), pl.observacion["izq"])
    assert plan.comprobado, plan.motivo
    bajada = [p for p in plan.puntos_de_paso if p.nombre == "bajar"]
    baja = 0.04 + 0.01 + cfg["contacto"]["margen_brazo_m"]            # aproximar, bajada y el margen del brazo
    assert abs(sum(p.duracion for p in bajada) * 2.0 - baja / 0.003) < 0.5      # a 3 mm/s, contada la escala
    cin_t = pl.cin_preforma(plan.detalle["preforma"])                   # con los dedos del toque
    yema = cin_t.fk("izq", bajada[-1].q, "yema")[:3, 3]
    assert abs(pl.plano.altura(yema) + 0.01 + cfg["contacto"]["margen_brazo_m"]) < 0.003   # bajo la mesa
    # en linea recta sobre la diana: una postura cada cm (con una sola, la yema se apartaba hasta 56 mm, 2026-11-12)
    assert len(bajada) >= 7
    en = pl.punto((0.46, 0.05), 0.0)
    Q = [p.q for p in plan.puntos_de_paso if p.nombre in ("aproximar", "bajar")]
    peor = max(float(np.linalg.norm((pl.plano.proyectar(cin_t.fk("izq", a + s * (b - a), "yema")[:3, 3]) - en)[:2]))
               for a, b in zip(Q[:-1], Q[1:]) for s in np.linspace(0, 1, 6))
    assert peor < 0.0025, peor
    if t.mundo is not None:
        t.mundo.parar()
