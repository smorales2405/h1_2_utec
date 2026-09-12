"""Control articular de los brazos del Unitree H1-2.

    joints          tabla de los 27 motores: índice, nombre, grupo, topes, par
    crc             CRC propio de Unitree para `unitree_hg/LowCmd`
    gains           lectura de `config/gains.yaml`: ganancias, topes,
                    autocolisión y postura de reposo
    arm_client      el cliente: publica a frecuencia fija, vigila el estado,
                    aborta por par o temperatura y suelta en rampa
    trajectories    generadores de consigna, `f(t) -> (q, dq)`
    gravity         par de gravedad con los parámetros de masa identificados
    motion_switcher soltar y recuperar el controlador de alto nivel

Para usarlo desde un algoritmo, lo que importa es `arm_client`::

    from h1_2_arm_control.arm_client import H12Client
    from h1_2_arm_control.joints import ARM_INDICES

    with H12Client(controlled=ARM_INDICES, channel="lowcmd") as cli:
        cli.wait_for_state()
        cli.engage()
        cli.ramp_to({16: 0.0}, speed=0.15)
        cli.release()
"""
