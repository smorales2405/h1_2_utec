"""Entrada al modo Debug para rt/lowcmd (decision del usuario: ReleaseMode() desde el programa).

Regla (memoria del proyecto y h1_2_robot_selector_manos.py): el Debug de verdad es CheckMode() con name ''.
Con name 'ai' el control de marcha publica en rt/lowcmd a 500 Hz y nuestros mensajes se ignoran (o se
alternan con los suyos). ReleaseMode() suelta el control de marcha y LAS PIERNAS: solo con el robot colgado.
Se llama tras escribir SOLTAR, como mucho 3 veces, y despues se escucha rt/lowcmd 3 s: si alguien publica,
no se crea nuestro publicador.
"""

import time

INTENTOS = 3
PALABRA = "SOLTAR"


class Conmutador:
    """MotionSwitcherClient del SDK: CheckMode y ReleaseMode."""

    def __init__(self, timeout_s=5.0):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        self.c = MotionSwitcherClient()
        self.c.SetTimeout(timeout_s)
        self.c.Init()

    def activo(self):
        """El nombre del modo activo ('' = Debug) o None si el servicio no contesta."""
        code, data = self.c.CheckMode()
        if code != 0 or data is None:
            return None
        return data.get("name", "")

    def soltar(self):
        code, _ = self.c.ReleaseMode()
        return code == 0


def contar_lowcmd(segundos, iface=None):
    """Mensajes de rt/lowcmd que llegan en `segundos` (el DDS ya iniciado por manip.robot.Robot)."""
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    n = [0]
    sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
    sub.Init(lambda m: n.__setitem__(0, n[0] + 1), 10)
    time.sleep(segundos)
    try:
        sub.Close()
    except Exception:
        pass
    return n[0]


def entrar_debug(conmutador, contar=contar_lowcmd, preguntar=input, log=print, escucha_s=3.0, dormir=time.sleep):
    """Deja el robot en Debug con rt/lowcmd libre o lanza RuntimeError. Devuelve el modo que habia."""
    previo = conmutador.activo()
    if previo is None:
        raise RuntimeError("motion_switcher no contesta: no se puede saber si rt/lowcmd esta libre")
    log(f"CheckMode: '{previo}'")
    if previo != "":
        log(f"Esta activo el control de marcha ('{previo}'). ReleaseMode() lo suelta y las PIERNAS SE SUELTAN:")
        log("el robot tiene que estar COLGADO del arnes con los pies sin tocar el suelo.")
        if preguntar(f"Escribe {PALABRA} para llamar a ReleaseMode() (otra cosa cancela): ").strip() != PALABRA:
            raise RuntimeError("cancelado por el operador")
        for k in range(1, INTENTOS + 1):
            ok = conmutador.soltar()
            dormir(1.0)
            ahora = conmutador.activo()
            log(f"ReleaseMode() intento {k}: {'ok' if ok else 'error'}; CheckMode -> '{ahora}'")
            if ahora == "":
                break
        else:
            raise RuntimeError(f"tras {INTENTOS} ReleaseMode() el modo sigue en '{ahora}': no se publica nada")
    n = contar(escucha_s)
    if n:
        raise RuntimeError(f"alguien publica en rt/lowcmd ({n} mensajes en {escucha_s:.0f} s): no se publica nada")
    log(f"Debug: CheckMode '' y rt/lowcmd en silencio {escucha_s:.0f} s")
    return previo
