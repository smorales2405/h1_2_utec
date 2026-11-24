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
    c = t.reg.carpeta
    assert os.path.exists(os.path.join(c, "ensayos.csv"))
    with open(os.path.join(c, "resumen.json")) as fh:
        res = json.load(fh)
    assert len(res["ensayos"]) == 2 and res["banco"]["objeto"] == "cubo" and "regulador" in res
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
    assert len(pedidos) == 1 and "escapado" in pedidos[0]


@gl
def test_nivel2_con_la_mano_regulada(tmp_path):
    t = _armar(tmp_path, ["--nivel", "2", "--sim", "--asignacion", "cubo=D", "cilindro=B",
                          "--colocacion", "cubo=0.47,-0.07,0.3", "cilindro=0.42,0.04,0"])
    assert mr.correr(t, confirmar=False) == "completado"
    assert {k: t.mundo.bandeja_de(k) for k in ("cubo", "cilindro")} == {"cubo": "D", "cilindro": "B"}
    agarres = [r for l in ("izq", "der") for r in t.manos[l].agarres]
    assert len(agarres) == 2 and all(r["regulado"] and r["t_establecido_s"] is not None for r in agarres), agarres
    assert os.path.basename(t.reg.carpeta).startswith("regulada_") and "nivel2_regulador_sim" in t.reg.carpeta
