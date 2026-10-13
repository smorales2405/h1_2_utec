"""Gemelo en MuJoCo: cinematica igual a la de pinocchio, camara virtual fiel al modelo pinhole y cierre de la
medida del montaje (medir sobre la imagen virtual lo que se uso para construirla). Las pruebas que
renderizan se saltan si no hay OpenGL (en el PC2, por ejemplo)."""

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from manip import calibracion as cb  # noqa: E402
from manip import config  # noqa: E402
from manip import montaje as mt  # noqa: E402
from manip.cinematica import PREFIJO, Cinematica  # noqa: E402
from manip.geometria import Intrinsecos, T, aplicar  # noqa: E402
from sim.escena import POSTURA_PIE, Estado, _camara, construir  # noqa: E402

T_COLOR_PROF = np.array([[1, 0, 0, 0.015], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def _hay_gl():
    try:
        m = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size='1'/></worldbody></mujoco>")
        mujoco.Renderer(m, 8, 8).close()
        return True
    except Exception:
        return False


gl = pytest.mark.skipif(not _hay_gl(), reason="sin OpenGL")


@pytest.fixture(scope="module")
def cin():
    return Cinematica.desde_config(config.cargar())


@pytest.fixture(scope="module")
def cal(cin):
    return cb.desde_urdf(cin, T_COLOR_PROF)


def test_fk_igual_que_pinocchio(cin, cal):
    m, info = construir(cal=cal, con_camaras=False)
    info["postura"] = [0.0 if i == 12 else v for i, v in enumerate(POSTURA_PIE)]   # cintura a 0, como cin
    e = Estado(m, info)
    rng = np.random.default_rng(0)
    for _ in range(100):
        for lado in ("izq", "der"):
            q = cin.lo[lado] + (cin.hi[lado] - cin.lo[lado]) * rng.random(7)
            e.poner_brazo(lado, q)
            e.adelante()
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{PREFIJO[lado]}_wrist_yaw_link")
            M = cin.fk(lado, q, "mano")
            assert np.linalg.norm(e.d.xpos[b] - M[:3, 3]) < 1e-4
            assert np.abs(e.d.xmat[b].reshape(3, 3) - M[:3, :3]).max() < 1e-4


@gl
def test_camara_virtual_es_pinhole():
    ic = Intrinsecos(604.11, 603.56, 321.33, 236.36, 640, 480)
    spec = mujoco.MjSpec()
    spec.worldbody.add_light().pos = [0, 0, 3]
    R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    _camara(spec.worldbody, "c", T(R, [0, 0, 0]), ic)
    pts = [(1.0, 0.0, 0.0), (1.0, 0.2, 0.1), (0.8, -0.25, -0.15), (1.2, 0.3, -0.2)]
    for p in pts:
        g = spec.worldbody.add_geom()
        g.type, g.size, g.pos = mujoco.mjtGeom.mjGEOM_SPHERE, [0.01, 0, 0], list(p)
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 480, 640)
    r.enable_segmentation_rendering()
    r.update_scene(d, "c")
    seg = r.render()[:, :, 0]
    r.disable_segmentation_rendering()
    r.enable_depth_rendering()
    r.update_scene(d, "c")
    z = r.render()
    for k, p in enumerate(pts):
        v, u = np.nonzero(seg == k)
        Pc = aplicar(np.linalg.inv(T(R, [0, 0, 0])), np.array(p))
        uv = ic.proyectar(Pc)[0]
        assert abs(u.mean() - uv[0]) < 0.3 and abs(v.mean() - uv[1]) < 0.3
        # profundidad planar (z), como la D435i: el frente de la esfera de 1 cm
        assert abs(z[int(round(v.mean())), int(round(u.mean()))] - (Pc[2] - 0.01)) < 0.002


@gl
def test_cierre_del_montaje(cin, cal):
    from sim.camara_virtual import CamaraVirtual
    mon = config.cargar_montaje()
    m, info = construir(cal=cal, con_bandejas=False)
    e = Estado(m, info)
    e.adelante()
    f = CamaraVirtual(m, info, cal).fotograma(e)
    assert {2, 3, 4, 5} <= set(cb.detectar(f.ir, cb.detector()))
    mesa, inf = mt.medir(f, cal, mon)
    T0, T1 = np.array(mon["mesa"]["T_torso_mesa"]), np.array(mesa["T_torso_mesa"])
    assert np.linalg.norm(T0[:3, 3] - T1[:3, 3]) < 0.003
    assert abs(mesa["yaw_deg"] - mon["mesa"]["yaw_deg"]) < 0.5
    assert abs(mesa["plano"]["d"] - mon["mesa"]["plano"]["d"]) < 0.002
