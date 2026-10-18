"""Supervisor de punta a punta en el mundo simulado (sim/simulacion.py): estados, contacto, agarre, registro y
salida segura, con el mismo cableado que manipulacion.py. Tiempo acelerado; se salta sin OpenGL."""

import copy
import os
import threading
import time
from types import SimpleNamespace

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
def test_nivel2_coloca_en_sus_bandejas(tmp_path):
    a = _args(tmp_path, nivel=2, asignacion={"tenis": "A", "cubo": "C"},
              colocacion={"tenis": ((0.43, 0.05), 0.0), "cubo": ((0.47, -0.07), 0.3)})
    t = manipulacion.armar(a, _cfg(), log=lambda *x: None)
    motivo = manipulacion.correr(t, confirmar=False)
    assert motivo == "completado"
    final = {k: t.mundo.bandeja_de(k) for k in ("tenis", "cubo")}
    assert final == {"tenis": "A", "cubo": "C"}
    assert t.brazo.peso == 0.0


@gl
def test_salida_segura_por_giro(tmp_path):
    a = _args(tmp_path, nivel=2, asignacion={"tenis": "A"}, colocacion={"tenis": ((0.43, 0.05), 0.0)})
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
