"""Colocacion del montaje (manip/montaje.py y herramientas/marcar_montaje.py): la pose de la mesa y las bandejas
medidas en la camara virtual con desvios conocidos, el ajuste de una bandeja vista a medias, las indicaciones
para quien empuja el carrito y la asignacion de bandejas por color."""

import copy
import importlib.util
import math
import os

import numpy as np
import pytest

from manip import config
from manip import montaje as mt

OBJETIVO = {"x0_m": 0.275, "y_m": 0.0, "yaw_deg": 0.0, "tol_x_m": 0.01, "tol_y_m": 0.01, "tol_yaw_deg": 1.0}


def _hay_gl():
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
        mujoco.Renderer(m, 8, 8).close()
        return True
    except Exception:
        return False


gl = pytest.mark.skipif(not _hay_gl(), reason="sin OpenGL")


def _herramienta():
    ruta = os.path.join(config.RAIZ, "herramientas", "marcar_montaje.py")
    spec = importlib.util.spec_from_file_location("marcar_montaje", ruta)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_desvio_mesa_dice_que_hacer():
    d = mt.desvio_mesa({"x0": 0.29, "y": 0.02, "yaw_deg": 2.0}, OBJETIVO)
    assert not d["todo_ok"]
    # yaw + : el extremo izquierdo de la mesa queda mas cerca del robot
    assert d["pasos"] == ["acerca 1.5 cm", "2.0 cm a la derecha del robot", "aleja el extremo izquierdo 15 mm"]
    d = mt.desvio_mesa({"x0": 0.26, "y": -0.02, "yaw_deg": -2.0}, OBJETIVO)
    assert d["pasos"] == ["aleja 1.5 cm", "2.0 cm a la izquierda del robot", "aleja el extremo derecho 15 mm"]
    assert mt.desvio_mesa({"x0": 0.28, "y": -0.005, "yaw_deg": -0.5}, OBJETIVO)["todo_ok"]


def test_ajustar_bandeja_vista_a_medias():
    """Solo la pared lejana, la de la izquierda y medio fondo (la mano tapa el resto), con ruido de 1.5 mm."""
    rng = np.random.default_rng(0)
    c, yaw = np.array([0.56, 0.28]), math.radians(6.0)
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    a = 0.06 + 0.0015
    s = rng.uniform(-a, a, 150)
    paredes = np.concatenate([np.stack([np.full(150, a), s], 1), np.stack([s, np.full(150, a)], 1)])
    fondo = rng.uniform([-0.06, 0.0], [0.06, 0.06], (300, 2))
    local = np.concatenate([paredes, fondo])
    xy = local @ R.T + c + rng.normal(0, 0.0015, local.shape)
    h = np.concatenate([rng.uniform(0.015, 0.04, len(paredes)), np.full(len(fondo), 0.003)])
    c_est, t_est, rms = mt.ajustar_bandeja(xy, h, 0.12, 0.003, c + [0.03, -0.02], 0.0)
    assert np.linalg.norm(c_est - c) < 0.002
    assert abs(math.degrees(t_est) - 6.0) < 1.0
    assert rms < 0.003


def test_asignacion_por_color_o_nombre():
    mon = {"bandejas": {"A": {"color": "blanca"}, "B": {"color": "roja"}, "C": {"color": None}}}
    assert mt.resolver_asignacion({"tenis": "blanca", "cubo": "C", "cilindro": "roja"}, mon) == \
        {"tenis": "A", "cubo": "C", "cilindro": "B"}
    with pytest.raises(ValueError):
        mt.resolver_asignacion({"prisma": "verde"}, mon)


def test_asignacion_de_objetos_yaml_existe_en_el_montaje():
    """La asignacion fija (objetos.yaml) apunta a bandejas de montaje.yaml, una por objeto."""
    objetos, mon = config.cargar_objetos(), config.cargar_montaje()
    asign = mt.resolver_asignacion({k: v["bandeja"] for k, v in objetos["objetos"].items()}, mon)
    assert sorted(asign.values()) == sorted(mon["bandejas"])


@gl
def test_mesa_y_bandejas_en_el_gemelo():
    """La camara virtual con el carrito y las bandejas fuera de su sitio una cantidad conocida y los brazos a 0
    (las manos tapan parte de las bandejas): la herramienta mide los desvios."""
    from manip.calibracion import detector
    from manip.cinematica import Cinematica
    herr = _herramienta()
    cfg = config.cargar()
    mon = copy.deepcopy(config.cargar_montaje())
    mon["objetivo_mesa"] = dict(OBJETIVO)
    fs = herr.FuenteSim(cfg, mon, Cinematica.desde_config(cfg), [0.012, -0.015, 1.2])
    desvios = {"A": (0.02, -0.01, 0.0), "B": (-0.015, 0.0, 8.0), "C": (0.0, 0.0, 0.0), "D": (0.01, 0.02, -5.0)}
    for k, d in desvios.items():
        fs.mover_bandeja(k, *d)
    f = fs.siguiente()
    pose = mt.pose_mesa(f, fs.cal, mon, detector())
    assert abs(pose["x0"] - (OBJETIVO["x0_m"] + 0.012)) < 0.003
    assert abs(pose["y"] + 0.015) < 0.003
    assert abs(pose["yaw_deg"] - 1.2) < 0.3
    med = mt.medir_bandejas(f, fs.cal, pose["plano"], pose["T"], mon, cfg["montaje"]["hsv_bandejas"])
    for k, (dx, dy, dyaw) in desvios.items():
        assert med[k] is not None, k
        esperado = np.asarray(mon["bandejas"][k]["centro"]) + [dx, dy]
        assert np.linalg.norm(med[k]["centro"] - esperado) < 0.004, k
        # el yaw de la bandeja, respecto de la mesa (que esta girada 1.2 grados)
        assert abs(med[k]["yaw_deg"] - (dyaw - 1.2)) < 1.5, k


def test_guardar_bandejas_solo_las_vistas_y_en_su_sitio():
    """[g] en BANDEJAS: la mediana de las ultimas medidas; la que se ve a medias, la que esta fuera de tolerancia y
    la que se ha visto en pocos fotogramas se quedan sin `medida` (se usa el objetivo). Sin escribir nada."""
    herr = _herramienta()
    mon = {"bandejas": {"A": {"color": "azul", "centro": [0.40, 0.28]}, "B": {"color": "roja", "centro": [0.56, 0.28]},
                        "C": {"color": "blanca", "centro": [0.40, -0.28], "medida": {"centro": [0.4, -0.28]}},
                        "D": {"color": "naranja", "centro": [0.56, -0.28]}}}
    rng = np.random.default_rng(0)
    hist = []
    for k in range(15):
        hist.append({"A": {"centro": np.array([0.404, 0.276]) + rng.normal(0, 0.001, 2), "yaw_deg": 1.0, "a_medias": False},
                     "B": {"centro": np.array([0.58, 0.28]), "yaw_deg": 0.0, "a_medias": False},     # 2 cm lejos
                     "C": {"centro": np.array([0.43, -0.27]), "yaw_deg": -11.0, "a_medias": True},
                     "D": None if k < 10 else {"centro": np.array([0.56, -0.28]), "yaw_deg": 0.0, "a_medias": False}})
    nuevo, informe, carpeta = herr.guardar_bandejas(mon, hist, [], {}, 0.015, guardar=False)
    assert carpeta is None and "medida" in mon["bandejas"]["C"]          # el original no se toca
    b = nuevo["bandejas"]
    assert np.allclose(b["A"]["medida"]["centro"], [0.404, 0.276], atol=0.001) and b["A"]["medida"]["yaw_deg"] == 1.0
    assert all("medida" not in b[k] for k in "BCD")
    assert np.allclose(mt.bandeja_real(b["A"])[0], [0.404, 0.276], atol=0.001)
    assert np.allclose(mt.bandeja_real(b["B"])[0], [0.56, 0.28])          # sin medida: el objetivo
    assert len(informe) == 4


@gl
def test_el_gemelo_y_el_planificador_usan_la_bandeja_medida():
    from sim.escena import Estado, construir
    import mujoco
    mon = copy.deepcopy(config.cargar_montaje())
    mon["bandejas"]["B"]["medida"] = {"centro": [0.57, 0.27], "yaw_deg": 4.0}
    m, info = construir(montaje=mon, con_camaras=False)
    e = Estado(m, info)
    e.adelante()
    pos = e.d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "bandeja_B")]
    assert np.allclose(pos[:2], [0.57, 0.27], atol=0.002)
    R = e.d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "bandeja_B")].reshape(3, 3)
    T = np.array(mon["mesa"]["T_torso_mesa"])
    assert abs(math.degrees(math.atan2(R[1, 0], R[0, 0]) - math.atan2(T[1, 0], T[0, 0])) - 4.0) < 0.2


@gl
def test_comprobar_el_montaje_antes_de_la_tirada():
    """La mesa y las bandejas donde dice el montaje: sin problemas; la mesa 2 cm corrida o una bandeja 2 cm
    movida desde su `medida`: lo dice."""
    from manip.calibracion import detector
    from manip.cinematica import Cinematica
    herr = _herramienta()
    cfg = config.cargar()
    mon = copy.deepcopy(config.cargar_montaje())
    mon["objetivo_mesa"] = dict(OBJETIVO)
    hsv, det = cfg["montaje"]["hsv_bandejas"], detector()
    fs = herr.FuenteSim(cfg, mon, Cinematica.desde_config(cfg), [0.0, 0.0, 0.0])
    ref = herr.montaje_en_objetivo(mon)                      # la mesa del gemelo, como si fuera la del [g]
    ref["bandejas"]["D"]["medida"] = {"centro": list(ref["bandejas"]["D"]["centro"]), "yaw_deg": 0.0}
    fotos = [fs.siguiente() for _ in range(3)]
    problemas, informe = mt.comprobar(fotos, fs.cal, ref, det, hsv)
    assert problemas == [], problemas
    fs.mover(0.02, 0.0, 0.0)
    fs.mover_bandeja("D", 0.0, 0.02, 0.0)
    problemas, _ = mt.comprobar([fs.siguiente() for _ in range(3)], fs.cal, ref, det, hsv)
    assert any("mesa se ha movido" in p for p in problemas), problemas
    assert any("bandeja D" in p and "de donde se midio" in p for p in problemas), problemas
