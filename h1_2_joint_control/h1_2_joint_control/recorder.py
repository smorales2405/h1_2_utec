"""Guardado de ensayos a CSV, y el índice de ensayos.

Un ensayo = un CSV con la serie temporal + una línea en `logs/index.csv` con
sus métricas. Así se puede volver sobre cualquier medida meses después y
comparar ganancias sin repetir el ensayo.
"""
from __future__ import annotations

import csv
import datetime as _dt
import json
from pathlib import Path

from .config import LOG_DIR

INDEX = LOG_DIR / "index.csv"


def stamp() -> str:
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def save_samples(samples, controlled, path: Path) -> Path:
    """Vuelca la serie temporal. Una columna por magnitud y articulación."""
    from .joints import BY_INDEX
    path.parent.mkdir(parents=True, exist_ok=True)
    cols = ["t", "weight"]
    for i in controlled:
        n = BY_INDEX[i].name
        cols += [f"{n}.q_des", f"{n}.q", f"{n}.dq", f"{n}.tau"]
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for s in samples:
            row = [f"{s.t:.6f}", f"{s.weight:.4f}"]
            for i in controlled:
                row += [f"{s.q_des[i]:.6f}", f"{s.q[i]:.6f}",
                        f"{s.dq[i]:.6f}", f"{s.tau[i]:.4f}"]
            w.writerow(row)
    return path


def append_index(row: dict, path: Path = INDEX) -> None:
    """Añade una línea al índice de ensayos, creando la cabecera si hace falta."""
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    prev_cols = []
    if exists:
        with path.open(newline="", encoding="utf-8") as fh:
            prev_cols = next(csv.reader(fh), [])
    cols = prev_cols or list(row)
    for k in row:                       # columnas nuevas al final
        if k not in cols:
            cols.append(k)
    if cols != prev_cols and exists:    # hay que reescribir con la cabecera nueva
        with path.open(newline="", encoding="utf-8") as fh:
            old = list(csv.DictReader(fh))
        with path.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(old)
    with path.open("a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        if not exists:
            w.writeheader()
        w.writerow({k: row.get(k, "") for k in cols})


def save_meta(meta: dict, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return path
