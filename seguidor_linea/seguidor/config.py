"""Carga de config/seguidor.yaml, con config/geometria.yaml (calibracion) por encima."""

import os

import yaml

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUTA_CONFIG = os.path.join(RAIZ, "config", "seguidor.yaml")
RUTA_GEOMETRIA = os.path.join(RAIZ, "config", "geometria.yaml")


def cargar(ruta=RUTA_CONFIG, ruta_geometria=RUTA_GEOMETRIA):
    with open(ruta) as f:
        cfg = yaml.safe_load(f)
    if ruta_geometria and os.path.exists(ruta_geometria):
        with open(ruta_geometria) as f:
            cfg.setdefault("geometria", {}).update(yaml.safe_load(f) or {})
    return cfg


def guardar_geometria(geometria, ruta=RUTA_GEOMETRIA):
    """La escribe calibrar_camara.py; seguidor.yaml no se toca (conserva sus comentarios)."""
    with open(ruta, "w") as f:
        f.write("# Escrito por herramientas/calibrar_camara.py: pisa la seccion `geometria` de seguidor.yaml\n")
        yaml.safe_dump(geometria, f, sort_keys=False, allow_unicode=True)


def ruta_datos(cfg):
    carpeta = cfg.get("registro", {}).get("carpeta", "datos")
    return carpeta if os.path.isabs(carpeta) else os.path.join(RAIZ, carpeta)
