#!/usr/bin/env python3
"""Reproduce un dataset (o una tirada) por percepcion -> estimacion -> control -> supervisor en
simulacro, con la IMU grabada, en la PC. No usa el robot.

1. Lazo: los estados por los que pasa el supervisor y las ordenes que habria mandado, con el mismo
   codigo que corre en el robot. El robot lo llevo el operador, asi que las ordenes no se cumplieron:
   sirve para ver signos, estados y que nada revienta con datos reales.
2. A ciegas (6.4.2): tapa la camara a ratos (--ciego s, varias veces) mientras se ve la linea y se
   anda, y compara la linea que predice la estimacion (IMU + avance a factor x vx) con la que se
   mide al volver: error en el desplazamiento lateral y en el angulo.

La vx del operador no se grabo (wasd.sh): se toma `--vx` (0.2 en los datasets) mientras el robot da
pasos (oscilacion de gz, seguidor/analisis.py).

    python3 herramientas/reproducir.py datos/dataset_<fecha>_<nombre> [--nivel 1] [--ciego 1 2] [--figura]

Escribe reproduccion.json (y reproduccion.png con --figura) en la carpeta.
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from seguidor import analisis, config, registro  # noqa: E402
from seguidor.control import Control  # noqa: E402
from seguidor.estimacion import Estimador, HistorialImu  # noqa: E402
from seguidor.fuentes import FuenteDataset  # noqa: E402
from seguidor.geometria import Intrinsecos  # noqa: E402
from seguidor.mensajes import Imu, Mando  # noqa: E402
from seguidor.percepcion import Percepcion  # noqa: E402
from seguidor.supervisor import Supervisor  # noqa: E402
from seguidor.vigilante import Vigilante  # noqa: E402


class Grabacion:
    """lowstate.csv como fuente de IMU (para HistorialImu) y como robot falso para el vigilante."""

    def __init__(self, ruta):
        with open(ruta) as f:
            filas = list(csv.DictReader(f))
        self.t = np.array([float(r["t"]) for r in filas])
        self.roll = np.radians([float(r["roll_deg"]) for r in filas])
        self.pitch = np.radians([float(r["pitch_deg"]) for r in filas])
        self.yaw = np.radians([float(r["yaw_deg"]) for r in filas])
        self.gz = np.array([float(r["gz"]) for r in filas])
        self.anda = analisis.andando(self.t, self.gz)
        self._i = 0
        self.ahora = self.t[0]
        self.marcha_habilitada = False

    def volcar(self, imu, hasta):
        while self._i < len(self.t) and self.t[self._i] <= hasta:
            i = self._i
            imu.agregar(self.t[i], self.roll[i], self.pitch[i], self.yaw[i])
            self._i += 1
        self.ahora = hasta

    def andando(self, t):
        return bool(self.anda[min(len(self.t) - 1, int(np.searchsorted(self.t, t)))])

    # interfaz de robot.Robot que usan el vigilante y el supervisor
    def _j(self):
        return max(0, min(len(self.t) - 1, int(np.searchsorted(self.t, self.ahora)) - 1))

    def edad_lowstate(self):
        return 0.0

    def motores_en_fallo(self):
        return []

    def imu(self):
        j = self._j()
        return Imu(t=self.t[j], tick=j, roll=self.roll[j], pitch=self.pitch[j], yaw=self.yaw[j], gx=0.0, gy=0.0,
                   gz=self.gz[j])

    def mando(self):
        return Mando(botones=0, lx=0.0, ly=0.0, rx=0.0, ry=0.0)


def pasada(carpeta, cfg, nivel, vx, ciego=0.0, cada_s=4.0):
    """Una pasada por el dataset. Con `ciego` > 0 tapa la camara `ciego` s cada `cada_s` s (solo
    mientras se anda) y devuelve los errores de la prediccion al volver."""
    with open(os.path.join(carpeta, "resumen.json")) as f:
        resumen = json.load(f)
    geo = resumen.get("geometria") or cfg["geometria"]
    fuente = FuenteDataset(carpeta)
    grab = Grabacion(os.path.join(carpeta, "lowstate.csv"))
    imu = HistorialImu(capacidad=1 << 17)
    per = Percepcion(cfg["percepcion"], Intrinsecos.desde_dict(fuente.meta["ir"]), geo)
    est = Estimador(cfg, imu, geo.get("x_puntera_m"), velocidad_externa=lambda t: vx if grab.andando(t) else 0.0)
    ctl = Control(cfg, 1.0)
    sup = Supervisor(cfg, grab, Vigilante(grab, cfg, joystick_para=False), ctl, est, nivel=nivel, simulacro=True)
    periodo = 1.0 / cfg["supervisor"]["hz_ordenes"]

    serie = {k: [] for k in ("t", "estado", "vx", "vyaw", "y", "theta", "conf", "edad", "dist_fin")}
    errores = []
    t_tick = None
    t_tapa = None          # inicio de la tapa en curso
    t_proxima = None
    ultimo_foto = None
    for f in fuente:
        grab.volcar(imu, f.t_rx)
        if t_tick is None:
            t_tick = f.t_rx
            t_proxima = f.t_rx + cada_s
            sup.empezar(f.t_rx)
        # ciclos de ordenes hasta este fotograma
        while t_tick <= f.t_rx:
            if sup.terminado:            # en un dataset se sigue mirando: se vuelve a empezar
                sup = Supervisor(cfg, grab, Vigilante(grab, cfg, joystick_para=False), ctl, est, nivel=nivel,
                                 simulacro=True)
                sup.empezar(t_tick)
            grab.ahora = t_tick
            dec = sup.paso(t_tick, t_tick - (ultimo_foto or t_tick))
            e = dec.detalle.get("est")
            for k, v in (("t", t_tick), ("estado", dec.estado), ("vx", dec.orden_enviada.vx),
                         ("vyaw", dec.orden_enviada.vyaw), ("y", e.y if e else math.nan),
                         ("theta", e.theta if e else math.nan), ("conf", e.confianza if e else math.nan),
                         ("edad", min(e.edad_s, 99) if e else math.nan),
                         ("dist_fin", e.dist_fin if (e and e.dist_fin is not None) else math.nan)):
                serie[k].append(v)
            t_tick += periodo
        roll, pitch = est.postura(f.t_rx)
        m = per.procesar(f, roll, pitch)
        ultimo_foto = f.t_rx
        if ciego > 0:
            if t_tapa is None and f.t_rx >= t_proxima and grab.andando(f.t_rx) and m.confianza >= 0.6 \
                    and est.estado(f.t_rx, 0.6).edad_s < 0.2:
                t_tapa = f.t_rx
            if t_tapa is not None:
                if f.t_rx - t_tapa < ciego:
                    continue                      # tapada: la medida no llega
                if m.confianza >= 0.6 and grab.andando(f.t_rx):
                    p = est.estado(f.t_rx - est.desfase, 0.6)
                    # la prediccion, en el marco del fotograma, frente a lo medido (en x = 0 y en angulo)
                    errores.append((t_tapa, 100 * (p.y - m.y * math.cos(m.theta)), math.degrees(p.theta - m.theta)))
                t_tapa = None
                t_proxima = f.t_rx + cada_s
        est.medida(m)
    serie = {k: np.array(v) for k, v in serie.items()}
    return serie, np.array(errores).reshape(-1, 3), est.rechazadas


def main():
    ap = argparse.ArgumentParser(description="Reproduce un dataset por todo el lazo, en simulacro (sin robot)")
    ap.add_argument("carpetas", nargs="+")
    ap.add_argument("--nivel", type=int, default=1)
    ap.add_argument("--vx", type=float, default=0.2, help="vx del operador mientras anda (wasd.sh --vx)")
    ap.add_argument("--ciego", type=float, nargs="*", default=[1.0, 2.0], help="s con la camara tapada")
    ap.add_argument("--figura", action="store_true")
    args = ap.parse_args()
    cfg = config.cargar()

    for carpeta in args.carpetas:
        nombre = os.path.basename(os.path.normpath(carpeta))
        serie, _, rechazadas = pasada(carpeta, cfg, args.nivel, args.vx)
        estados, cuenta = np.unique(serie["estado"], return_counts=True)
        periodo = 1.0 / cfg["supervisor"]["hz_ordenes"]
        sig = np.isin(serie["estado"], ["SEGUIMIENTO", "LINEA_PERDIDA"])
        out = {"dataset": nombre, "segundos_por_estado": {str(e): round(c * periodo, 1) for e, c in zip(estados, cuenta)},
               "medidas_rechazadas_por_la_puerta": int(rechazadas),
               "vyaw_max": float(np.max(np.abs(serie["vyaw"]))) if len(serie["vyaw"]) else math.nan,
               "suavidad_vyaw": float(np.sqrt(np.mean(np.diff(serie["vyaw"][sig]) ** 2))) if sig.sum() > 2 else math.nan}
        # signo: con la linea a la izquierda (y > 0) y de frente, la orden gira a la izquierda
        ok = sig & np.isfinite(serie["y"]) & (np.abs(serie["theta"]) < math.radians(5)) & (np.abs(serie["y"]) > 0.08)
        if ok.sum() > 10:
            out["signo_correcto"] = float(np.mean(np.sign(serie["vyaw"][ok] - cfg["control"]["vyaw_sesgo"]) == np.sign(serie["y"][ok])))
        print(f"{nombre}: " + ", ".join(f"{k} {v} s" for k, v in out["segundos_por_estado"].items())
              + f"; {rechazadas} medidas fuera de la puerta; vyaw max {out['vyaw_max']:.2f} rad/s, suavidad "
              f"{out['suavidad_vyaw']:.3f}" + (f"; signo bien en {100 * out['signo_correcto']:.0f} %" if "signo_correcto" in out else ""))
        for ciego in args.ciego:
            _, err, _ = pasada(carpeta, cfg, args.nivel, args.vx, ciego=ciego)
            if len(err):
                out[f"ciego_{ciego:g}s"] = {"n": int(len(err)), "y_cm_mediana_abs": float(np.median(np.abs(err[:, 1]))),
                                            "y_cm_max_abs": float(np.max(np.abs(err[:, 1]))),
                                            "y_cm_media": float(np.mean(err[:, 1])),
                                            "angulo_deg_mediana_abs": float(np.median(np.abs(err[:, 2]))),
                                            "angulo_deg_max_abs": float(np.max(np.abs(err[:, 2])))}
                o = out[f"ciego_{ciego:g}s"]
                print(f"   {ciego:g} s a ciegas ({o['n']} veces): y {o['y_cm_mediana_abs']:.1f} cm de mediana "
                      f"(max {o['y_cm_max_abs']:.1f}, sesgo {o['y_cm_media']:+.1f}), angulo {o['angulo_deg_mediana_abs']:.2f} "
                      f"grados (max {o['angulo_deg_max_abs']:.2f})")
        registro.guardar_json(os.path.join(carpeta, "reproduccion.json"), out)

        if args.figura:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            t = serie["t"] - serie["t"][0]
            fig, ejes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
            ejes[0].plot(t, 100 * serie["y"], label="y estimada (cm)")
            ejes[0].plot(t, np.degrees(serie["theta"]), label="angulo (grados)")
            ejes[0].legend(fontsize=8)
            ejes[1].plot(t, serie["vyaw"], label="vyaw que mandaria (rad/s)")
            ejes[1].plot(t, serie["vx"], label="vx (m/s)")
            ejes[1].legend(fontsize=8)
            nombres = ["ESPERA", "SEGUIMIENTO", "LINEA_PERDIDA", "GIRO_ESQUINA", "FIN", "PARADA"]
            ejes[2].plot(t, [nombres.index(e) for e in serie["estado"]], drawstyle="steps-post")
            ejes[2].set_yticks(range(len(nombres)))
            ejes[2].set_yticklabels(nombres, fontsize=7)
            ejes[2].set_xlabel("s")
            fig.suptitle(nombre)
            fig.tight_layout()
            fig.savefig(os.path.join(carpeta, "reproduccion.png"), dpi=90)


if __name__ == "__main__":
    main()
