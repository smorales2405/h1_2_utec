# `robot_onboard` — el gesto sin ordenador

Demonio que corre **en el PC2 del robot**, arranca con él, y al pulsar `R2+up`
en el mando hace el gesto «six-seven» y devuelve los brazos donde estaban. Sin
nada conectado.

Es el equivalente de:

```bash
ros2 run h1_2_arm_control six_seven_remote --ros-args \
    -p amplitude_deg:=10.0 -p speed:=1.0 -p duration:=10.0 -p approach_speed:=0.75
```

## Por qué no es el mismo programa

**El PC2 no tiene ROS 2.** Tiene Python 3.10 y `unitree_sdk2py` sobre
`cyclonedds`, instalados sin Internet desde un bundle (ver
`h1_2_teleoperation/README_DEPLOY.md` §4). El nodo de `ros_h1_2_ws` depende de
`rclpy` y de los mensajes de `unitree_ros2`, así que ahí no arranca.

Lo que cambia es **solo el transporte**: `ChannelPublisher("rt/arm_sdk")` en vez
de un publicador ROS. La matemática del gesto, la tabla de motores
(`joints.py`), las ganancias (`gains.yaml`) y la protección de autocolisión son
los mismos ficheros, copiados.

## Instalar

```bash
./install_pc2.sh                      # unitree@192.168.123.164 por defecto
```

Necesita entrar por SSH sin contraseña:

```bash
ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519     # si no tienes clave
ssh-copy-id unitree@192.168.123.164
```

El script comprueba el entorno del robot, copia el código, instala la unidad de
`systemd` y la deja habilitada para el siguiente arranque.

## Ver qué hace

```bash
ssh unitree@192.168.123.164 'journalctl -u h1_2_six_seven -f'
ssh unitree@192.168.123.164 'sudo systemctl stop h1_2_six_seven'
```

## Probar a mano

Desde el robot, o desde la laptop con `--nic enp12s0`:

```bash
python3 six_seven_daemon.py --dry-run                  # sin publicar nada
python3 six_seven_daemon.py --once --trigger-now        # una vez, sin mando
python3 six_seven_daemon.py --keep-posture --amplitude-deg 4.0 --speed 0.3
```

## Seguridad de un servicio que arranca solo

- **Mientras espera no publica nada.** El robot tiene sus brazos enteros.
- **SIGTERM suelta antes de morir.** `systemd` para con SIGTERM y la unidad da
  20 s: soltar los brazos de un robot de pie no se interrumpe a medias.
- **Un gesto cada vez.** Pulsar durante la ejecución no hace nada.
- **Si algo aborta** —par sostenido, temperatura, `rt/lowstate` rancio— el peso
  baja a cero y el demonio vuelve a esperar, en vez de caerse.
- **`Restart=always`** por si aun así muere.

### Lo que queda sin verificar

Qué hace el robot si el proceso muere de golpe (`kill -9`, corte de luz al PC2)
con el peso a 1. El demonio no puede soltar en ese caso; depende de que
`arm_sdk` tenga su propio plazo de caducidad en el robot, y eso no se ha
comprobado. Con `systemd` parando por SIGTERM no ocurre, pero conviene saberlo.

## Ganancias

`gains.yaml`, conjunto **`tuned`** y no `tuned_gff`: en `arm_sdk` el servicio
del robot aplica **su** compensación de gravedad, y sumar la nuestra sería
contarla dos veces. Por eso aquí no hace falta `pinocchio` ni el URDF.
