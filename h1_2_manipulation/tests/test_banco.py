"""De punta a punta en el mundo simulado del reto (sim/simulacion.py) con la mano de agarre/sim_mano.py: el banco con
los dos brazos intercalados y el nivel 2 con la mano regulada. Con el cableado de manipulacion_regulada.py (el armar y
el correr del reto); se salta sin OpenGL."""

import copy
import json
import os
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from manip import config

import manipulacion_regulada as mr
from agarre.banco import desde_sitio, texto_correccion


def _hay_gl():
    try:
        m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
        mujoco.Renderer(m, 8, 8).close()
        return True
    except Exception:
        return False


gl = pytest.mark.skipif(not _hay_gl(), reason="sin OpenGL")


def _cfg():
    cfg = copy.deepcopy(config.cargar())
    cfg["mano"]["t_base_s"] = 0.0
    return cfg


def _armar(tmp_path, argv):
    cfg, objetos = _cfg(), config.cargar_objetos()
    a, mios = mr.preparar(argv, cfg, objetos)
    a.escala_tiempo = 1.0
    return mr.armar(a, mios, cfg, objetos, log=lambda *x: None, datos=str(tmp_path))


@gl
def test_el_banco_intercala_los_brazos_y_devuelve_el_objeto(tmp_path):
    t = _armar(tmp_path, ["--banco", "cubo", "--ensayos", "2", "--sim", "--semilla", "1", "--sostener-s", "1"])
    xy0 = t.mundo.donde("cubo")[:3, 3].copy()
    assert mr.correr(t, confirmar=False) == "completado"
    e = t.sup.ensayos
    assert sorted(r["brazo"] for r in e) == ["firmware", "regulador"]
    assert [r["resultado"] for r in e] == ["sostenido", "sostenido"], e
    # devuelto a su sitio, no a la bandeja
    assert t.mundo.bandeja_de("cubo") is None
    assert np.linalg.norm(t.mundo.donde("cubo")[:2, 3] - xy0[:2]) < 0.02
    estados = [x[1] for x in t.reg.resumen["eventos"]]
    for s in ("SOSTENER", "DEVOLVER", "SOLTAR", "BANCO"):
        assert s in estados, s
    reg = next(r for r in e if r["brazo"] == "regulador")
    fw = next(r for r in e if r["brazo"] == "firmware")
    assert reg["lazo"]["regulado"] and reg["t_establecido_s"] is not None
    assert not fw["lazo"]["regulado"]                       # el firmware, medido por el lazo observador
    assert fw["apriete_sostener_gf"] and reg["apriete_sostener_gf"]
    # la observacion de despues, tambien tras el ultimo; y cada ensayo, desde su sitio
    assert all(r["desplazamiento_mm"] is not None and r["giro_deg"] is not None for r in e), e
    assert all(r["sitio_mm"] is not None and r["sitio_mm"] <= 5 for r in e), e
    assert estados.count("OBSERVAR") >= 4 and "RECOLOCAR" in estados
    c = t.reg.carpeta
    assert os.path.exists(os.path.join(c, "ensayos.csv"))
    with open(os.path.join(c, "resumen.json")) as fh:
        res = json.load(fh)
    assert len(res["ensayos"]) == 2 and res["banco"]["objeto"] == "cubo" and "regulador" in res
    assert np.linalg.norm(np.array([res["banco"]["sitio"]["x"], res["banco"]["sitio"]["y"]]) - xy0[:2]) < 0.01
    assert not any(k.startswith("_") for r in res["ensayos"] for k in r)
    assert sorted(f for f in os.listdir(c) if f.startswith("regulador_")) == \
        sorted([f"regulador_01_cubo_der{'_firmware' if e[0]['brazo'] == 'firmware' else ''}.csv",
                f"regulador_02_cubo_der{'_firmware' if e[1]['brazo'] == 'firmware' else ''}.csv"])
    assert t.brazo.peso == 0.0


@gl
def test_el_banco_pide_recolocar_el_que_se_cae(tmp_path):
    """Se cae sobre la bandeja mientras se sostiene: el ensayo es "escapado" (visto por el lazo o por el reto), el robot
    vuelve a la observacion, pide recolocarlo y sigue con el siguiente."""
    t = _armar(tmp_path, ["--banco", "cilindro", "--ensayos", "2", "--sim", "--semilla", "2", "--sostener-s", "2"])
    xy0, yaw0 = t.mundo.donde("cilindro")[:2, 3].copy(), 0.0
    hecho, pedidos = [], []

    def soltar(estado):
        if estado == "SOSTENER" and not hecho:
            hecho.append(1)
            with t.mundo.cerrojo:
                t.mundo._al_abrir("izq")                 # cae (en la bandeja)
    t.sup.al_estado.append(soltar)

    def esperar(texto):
        pedidos.append(texto)
        t.mundo.poner("cilindro", xy0, yaw0)             # el usuario lo recoloca
        return True
    t.sup.esperar_usuario = esperar
    assert mr.correr(t, confirmar=False) == "completado"
    e = t.sup.ensayos
    assert [r["resultado"] for r in e] == ["escapado", "sostenido"], e
    assert e[0]["fase_escape"] == "SOSTENER"
    # al empezar, sobre la cruz; tras el escape, otra vez
    assert len(pedidos) == 2 and "cruz" in pedidos[0] and "escapado" in pedidos[1], pedidos


@gl
def test_el_banco_comprueba_que_este_en_su_sitio(tmp_path):
    """Tras el ensayo 1 queda 15 mm mas lejos del robot: se pide recolocarlo. Puesto 12 mm a la izquierda del robot,
    la camara lo ve fuera de su sitio y se dice hacia donde moverlo; a la tercera, en su sitio."""
    t = _armar(tmp_path, ["--banco", "cubo", "--ensayos", "2", "--sim", "--semilla", "1", "--sostener-s", "1"])
    T0 = t.mundo.donde("cubo")
    xy0, yaw0 = T0[:2, 3].copy(), float(np.arctan2(T0[1, 0], T0[0, 0]))
    pedidos, puestos = [], [xy0, xy0 + [0.0, 0.012], xy0]

    def mover(estado):
        if estado == "VOLVER" and len(t.sup.ensayos) == 0:
            with t.mundo.cerrojo:
                t.mundo.poner("cubo", t.mundo.donde("cubo")[:2, 3] + [0.015, 0.0], yaw0)
    t.sup.al_estado.append(mover)

    def esperar(texto):
        pedidos.append(texto)
        with t.mundo.cerrojo:
            t.mundo.poner("cubo", puestos[min(len(pedidos), len(puestos)) - 1], yaw0)
        return True
    t.sup.esperar_usuario = esperar
    assert mr.correr(t, confirmar=False) == "completado"
    e = t.sup.ensayos
    assert len(e) == 2 and len(pedidos) == 3, pedidos
    assert "hacia el robot" in pedidos[1], pedidos             # quedo mas lejos: hay que acercarlo
    assert "derecha del robot" in pedidos[2], pedidos          # puesto a su izquierda
    assert 8 < e[0]["desplazamiento_mm"] < 30, e[0]
    assert e[1]["sitio_mm"] <= 5, e[1]


def test_lo_apartado_del_sitio():
    sitio = {"x": 0.47, "y": -0.07, "yaw_deg": -3.0}
    d = desde_sitio(sitio, (0.485, -0.06), -14.0, 90)
    assert d["dx_mm"] == pytest.approx(-15) and d["dy_mm"] == pytest.approx(-10)
    assert d["mm"] == pytest.approx(18.03, abs=0.01) and d["giro_deg"] == pytest.approx(-11)
    txt = texto_correccion(d)
    assert "15 mm hacia el robot" in txt and "10 mm hacia la derecha del robot" in txt and "11 grados en sentido antihorario" in txt
    # modulo la simetria: el cubo girado 88 grados esta a -2 de su sitio; la esfera no tiene giro
    assert desde_sitio(sitio, (0.47, -0.07), 85.0, 90)["giro_deg"] == pytest.approx(-2)
    assert desde_sitio(sitio, (0.47, -0.07), 85.0, 0)["giro_deg"] is None
    assert texto_correccion(desde_sitio(sitio, (0.47, -0.07), -3.5, 90)) == "ya esta en su sitio"


@gl
def test_nivel2_con_la_mano_regulada(tmp_path):
    t = _armar(tmp_path, ["--nivel", "2", "--sim", "--asignacion", "cubo=D", "cilindro=B",
                          "--colocacion", "cubo=0.47,-0.07,0.3", "cilindro=0.42,0.04,0"])
    assert mr.correr(t, confirmar=False) == "completado"
    assert {k: t.mundo.bandeja_de(k) for k in ("cubo", "cilindro")} == {"cubo": "D", "cilindro": "B"}
    agarres = [r for l in ("izq", "der") for r in t.manos[l].agarres]
    assert len(agarres) == 2 and all(r["regulado"] and r["t_establecido_s"] is not None for r in agarres), agarres
    assert os.path.basename(t.reg.carpeta).startswith("regulada_") and "nivel2_regulador_sim" in t.reg.carpeta
