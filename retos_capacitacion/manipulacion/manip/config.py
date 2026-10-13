"""Carga de la configuracion: config/manipulacion.yaml, montaje.yaml, objetos.yaml y la calibracion.

La calibracion es la mas reciente de config/calibracion_<fecha>.yaml (o la que se pida). El resto del
sistema no lleva ninguna transformacion escrita en el codigo (seccion 5 del PDF).
"""

import glob
import os

import yaml

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(RAIZ, "config")
RUTA_CONFIG = os.path.join(CONFIG, "manipulacion.yaml")
RUTA_MONTAJE = os.path.join(CONFIG, "montaje.yaml")
RUTA_OBJETOS = os.path.join(CONFIG, "objetos.yaml")
RELATIVO_URDF = "ros_h1_2_ws/src/h1_2_inspire_description/urdf/h1_2_with_RH56DFTP_hands.urdf"


def _leer(ruta):
    with open(ruta) as f:
        return yaml.safe_load(f) or {}


def cargar(ruta=RUTA_CONFIG):
    return _leer(ruta)


def cargar_montaje(ruta=RUTA_MONTAJE):
    return _leer(ruta)


def cargar_objetos(ruta=RUTA_OBJETOS):
    return _leer(ruta)


def a_python(x):
    """Tipos de numpy -> tipos de Python, para yaml.safe_dump."""
    if isinstance(x, dict):
        return {a_python(k): a_python(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [a_python(v) for v in x]
    if hasattr(x, "tolist"):          # np.ndarray, np.float64, np.int64...
        return a_python(x.tolist())
    return x


def guardar_yaml(ruta, datos, cabecera=""):
    """Escribe en un temporal y lo renombra: un fallo a medias no deja el fichero roto."""
    texto = "".join(f"# {l}\n" if l else "#\n" for l in cabecera.strip().splitlines())
    texto += yaml.safe_dump(a_python(datos), sort_keys=False, allow_unicode=True, default_flow_style=None)
    with open(ruta + ".tmp", "w") as f:
        f.write(texto)
    os.replace(ruta + ".tmp", ruta)


def ruta_repo():
    """Raiz con ros_h1_2_ws/ y h1_2_mujoco/: el repo en la PC, o modelo/ en el PC2 (desplegar.sh)."""
    for base in (os.path.dirname(os.path.dirname(RAIZ)), os.path.join(RAIZ, "modelo")):
        if os.path.exists(os.path.join(base, RELATIVO_URDF)):
            return base
    raise FileNotFoundError(f"no esta {RELATIVO_URDF} ni en el repo ni en {RAIZ}/modelo (./desplegar.sh)")


def ruta_urdf():
    return os.path.join(ruta_repo(), RELATIVO_URDF)


def ruta_calibracion(ruta=None):
    """La pedida o la mas reciente de config/calibracion_*.yaml; None si no hay ninguna."""
    if ruta:
        return ruta
    rutas = sorted(glob.glob(os.path.join(CONFIG, "calibracion_*.yaml")))
    return rutas[-1] if rutas else None


def ruta_datos(cfg):
    carpeta = cfg.get("registro", {}).get("carpeta", "datos")
    return carpeta if os.path.isabs(carpeta) else os.path.join(RAIZ, carpeta)
