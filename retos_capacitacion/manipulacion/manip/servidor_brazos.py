"""Los brazos del robot de pie en el arnes, para otro programa (PLAN, opcion B, 2026-11-10).

herramientas/arnes.py sostiene al robot por rt/lowcmd (manip/cuerpo.py, 500 Hz) y es el unico que publica. Este
servidor, en un hilo de arnes.py, deja que manipulacion.py (manip/brazo_arnes.py) mueva los brazos: el robot se
pone de pie una vez y se hacen varias tiradas, y si manipulacion.py falla las piernas siguen sostenidas.

TCP en 127.0.0.1 (red.puerto_brazos), una linea JSON por peticion y otra por respuesta. Peticiones {"op", "id"}:
  estado {cuerpo}                fase, listo, controlado (y si es suyo), moviendo, consigna (27), parada, amortiguado;
                                 con cuerpo, lo medido en el ultimo ciclo de 500 Hz: IMU (rpy, giroscopo), carga y
                                 fuerza de los pies, integral del estabilizador y q y par de las 27 (curva_pasos.py)
  tomar {recuperar}              el control de los brazos: de pie, nivelado, con el estabilizador, sin parada ni
                                 movimiento, y con los brazos a 0 (recuperar: desde donde esten, para que la salida
                                 segura de manipulacion.py saque unos brazos que se quedaron sobre la mesa)
  mover {q (14), segundos, salida}   los brazos (13-26) a q con Cuerpo.mover: minima sacudida de >= 8 s, pico
                                 <= 0.5 rad/s, recortada a los limites. Uno cada vez (con otro en curso, no)
  congelar                       la consigna de los brazos a la postura medida (para el movimiento en curso)
  detener                        para el movimiento en curso donde va la consigna
  fijar {q (14)}                 la consigna de los brazos sin trayectoria, con saltos <= 0.05 rad (el toque del
                                 nivel 1: lo medido mas el error estatico, el brazo se queda donde esta)
  soltar                         devuelve el control; los brazos se quedan donde esten
Cada peticion del que tiene el control es su latido: sin ninguna en `latido_max_s`, los brazos se congelan y pierde
el control (un movimiento en curso acaba solo, pero el siguiente ya no llega).
Una parada de arnes.py con el control dado (parar_remoto: ESPACIO, boton del mando, terminal cortado, temperatura)
congela los brazos y desde entonces solo acepta movimientos de salida: manipulacion.py hace su salida segura (abrir
la mano, subir, por la observacion a 0) y suelta, y despues arnes.py sigue con la suya. L2+B amortigua en el hilo
de Cuerpo y aqui se rechaza todo.
"""

import json
import socket
import socketserver
import threading
import time

import numpy as np

from .cuerpo import BRAZOS


class _Manejador(socketserver.StreamRequestHandler):
    def handle(self):
        srv = self.server.servidor
        with srv.cerrojo:
            srv._conexiones.add(self.connection)
        try:
            self._atender(srv)
        finally:
            with srv.cerrojo:
                srv._conexiones.discard(self.connection)

    def _atender(self, srv):
        for linea in self.rfile:
            try:
                resp = srv.atender(json.loads(linea))
            except Exception as ex:                    # una peticion mala no tumba el servidor
                resp = {"ok": False, "error": f"{type(ex).__name__}: {ex}"}
            try:
                self.wfile.write((json.dumps(resp) + "\n").encode())
            except OSError:
                return


class _Tcp(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True

    def service_actions(self):
        self.servidor.vigilar_latido()


class ServidorBrazos:
    def __init__(self, cuerpo, puerto, listo, brazos_cero, log=print, latido_max_s=2.0, tol_cero_rad=0.05,
                 reloj=time.monotonic):
        """cuerpo: manip.cuerpo.Cuerpo (mover, moviendo, congelar, consigna, amortiguado, motivo). listo():
        (bool, por que no) para dar el control. brazos_cero: las 14 q de los brazos a 0."""
        self.cu, self.puerto, self.listo = cuerpo, int(puerto), listo
        self.brazos_cero = np.asarray(brazos_cero, float)
        self.log, self.latido_max, self.tol_cero, self.reloj = log, float(latido_max_s), float(tol_cero_rad), reloj
        self.cerrojo = threading.Lock()
        self.controlador, self.t_latido, self.parada = None, None, None
        self.ocupado_local = False          # arnes.py mueve los brazos ([m], [k]): no se da el control
        self.movimientos = 0
        self._tcp, self._hilo = None, None
        self._conexiones = set()

    # --- ciclo de vida ---------------------------------------------------------------------------------
    def iniciar(self):
        self._tcp = _Tcp(("127.0.0.1", self.puerto), _Manejador)
        self._tcp.servidor = self
        self._hilo = threading.Thread(target=self._tcp.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True)
        self._hilo.start()

    def parar(self):
        """Deja de escuchar y corta las conexiones abiertas (el cliente lo nota en su siguiente peticion)."""
        if self._tcp is not None:
            self._tcp.shutdown()
            self._tcp.server_close()
            self._tcp = None
        with self.cerrojo:
            conexiones, self._conexiones = list(self._conexiones), set()
        for c in conexiones:
            try:
                c.shutdown(socket.SHUT_RDWR)
                c.close()
            except OSError:
                pass

    # --- lo que pide arnes.py ---------------------------------------------------------------------------
    @property
    def controlado(self):
        with self.cerrojo:
            return self.controlador

    def vigilar_latido(self):
        with self.cerrojo:
            if self.controlador is None or self.reloj() - self.t_latido <= self.latido_max:
                return
            quien, self.controlador = self.controlador, None
        self.cu.congelar(BRAZOS)
        self.log(f"{quien}: sin noticias en {self.latido_max:.1f} s; brazos congelados y sin control remoto")

    def parar_remoto(self, motivo):
        """Una parada de arnes.py con el control dado: congela los brazos y solo se aceptan los movimientos de
        salida. True si habia alguien con el control."""
        with self.cerrojo:
            if self.controlador is None:
                return False
            self.parada = motivo
        self.cu.congelar(BRAZOS)
        return True

    def quitar(self, motivo):
        """Quita el control (los brazos, congelados)."""
        with self.cerrojo:
            quien, self.controlador = self.controlador, None
        if quien is not None:
            self.cu.congelar(BRAZOS)
            self.log(f"{quien}: control quitado ({motivo}); brazos congelados")

    # --- peticiones -------------------------------------------------------------------------------------
    def atender(self, req):
        op, quien = req.get("op"), req.get("id")
        with self.cerrojo:
            suyo = self.controlador is not None and quien == self.controlador
            if suyo:
                self.t_latido = self.reloj()
        if op == "estado":
            return self._estado(suyo, cuerpo=bool(req.get("cuerpo")))
        if op == "tomar":
            return self._tomar(quien, bool(req.get("recuperar")))
        if not suyo:
            return {"ok": False, "error": "sin el control de los brazos"}
        if op == "mover":
            return self._mover(req)
        if op == "congelar":
            self.cu.congelar(BRAZOS)
            return self._estado(True)
        if op == "detener":
            self.cu.detener()
            return self._estado(True)
        if op == "fijar":
            q = [float(v) for v in req["q"]]
            if len(q) != len(BRAZOS) or not np.all(np.isfinite(q)):
                return {"ok": False, "error": f"q tiene que ser {len(BRAZOS)} numeros (brazos 13-26)"}
            try:
                self.cu.fijar(dict(zip(BRAZOS, q)))
            except ValueError as ex:
                return {"ok": False, "error": str(ex)}
            return self._estado(True)
        if op == "soltar":
            with self.cerrojo:
                self.controlador = None
            d = self._lejos_de_cero()
            self.log(f"{quien} suelta los brazos" + ("" if d <= self.tol_cero else
                                                    f" a {np.degrees(d):.0f} grados de 0: se quedan ahi"))
            return {"ok": True}
        return {"ok": False, "error": f"orden desconocida: {op!r}"}

    def _lejos_de_cero(self):
        return float(np.abs(self.cu.consigna()[BRAZOS] - self.brazos_cero).max())

    def _estado(self, suyo, cuerpo=False):
        listo, porque = self.listo()
        with self.cerrojo:
            controlado, parada = self.controlador is not None, self.parada
        r = {"ok": True, "listo": bool(listo), "porque": porque, "controlado": controlado, "tuyo": suyo,
             "moviendo": self.cu.moviendo(), "consigna": [round(float(v), 6) for v in self.cu.consigna()],
             "parada": parada, "amortiguado": bool(self.cu.amortiguado), "motivo_amortiguado": self.cu.motivo}
        if cuerpo:
            r["cuerpo"] = self._cuerpo()
        return r

    def _cuerpo(self):
        e = self.cu.estado                          # el dict del ultimo ciclo (se sustituye entero en cada uno)
        if not e:
            return None
        return {"t": round(float(e["t"]), 4),
                "rpy": [round(float(e[k]), 6) for k in ("roll", "pitch", "yaw")],
                "gyro": [round(float(v), 5) for v in e["gyro"]],
                "carga": round(float(e["carga"]), 4), "fz": {k: round(float(v), 1) for k, v in e["fz"].items()},
                "integral": [round(float(v), 3) for v in e["integral"]],
                "q": [round(float(v), 6) for v in e["q"]], "tau": [round(float(v), 3) for v in e["tau"]]}

    def _tomar(self, quien, recuperar):
        if not quien:
            return {"ok": False, "error": "falta el id"}
        if self.cu.amortiguado:
            return {"ok": False, "error": f"amortiguado ({self.cu.motivo})"}
        with self.cerrojo:
            otro = self.controlador
        if otro is not None and otro != quien:
            return {"ok": False, "error": f"ya tiene los brazos: {otro}"}
        listo, porque = self.listo()
        if not listo:
            return {"ok": False, "error": f"arnes.py no esta listo: {porque}"}
        if self.ocupado_local or self.cu.moviendo():
            return {"ok": False, "error": "arnes.py esta moviendo los brazos"}
        d = self._lejos_de_cero()
        if d > self.tol_cero and not recuperar:
            return {"ok": False, "error": f"los brazos estan a {np.degrees(d):.0f} grados de 0: para sacarlos, "
                                          "manipulacion.py --arnes --recuperar"}
        with self.cerrojo:
            self.controlador, self.t_latido, self.parada = quien, self.reloj(), None
        self.log(f"{quien} toma los brazos" + (f" para recuperarlos (a {np.degrees(d):.0f} grados de 0)" if recuperar else ""))
        return self._estado(True)

    def _mover(self, req):
        if self.cu.amortiguado:
            return {"ok": False, "error": f"amortiguado ({self.cu.motivo})"}
        salida = bool(req.get("salida"))
        with self.cerrojo:
            parada = self.parada
        if parada and not salida:
            return {"ok": False, "error": f"parada en arnes.py ({parada}): solo la salida segura"}
        if not salida:
            listo, porque = self.listo()
            if not listo:
                return {"ok": False, "error": f"arnes.py no esta listo: {porque}"}
        if self.cu.moviendo():
            return {"ok": False, "error": "hay otro movimiento en curso"}
        q = [float(v) for v in req["q"]]
        if len(q) != len(BRAZOS) or not np.all(np.isfinite(q)):
            return {"ok": False, "error": f"q tiene que ser {len(BRAZOS)} numeros (brazos 13-26)"}
        segundos = req.get("segundos")
        T = self.cu.mover(dict(zip(BRAZOS, q)), float(segundos) if segundos else None, esperar=False)
        self.movimientos += 1
        return {"ok": True, "T": T}
