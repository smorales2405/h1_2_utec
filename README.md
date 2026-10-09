# h1_2_utec

Teleoperación XR del **Unitree H1-2** con manos **Inspire RH56DFTP** y **Meta Quest 3**,
sobre [`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) de Unitree.

Este repositorio contiene **solo lo propio**: los scripts escritos para este
despliegue y los parches al código de terceros. Los repositorios upstream no se
vendorizan — se clonan siguiendo [`README_SIM.md`](h1_2_teleoperation/README_SIM.md) §5
(el robot real no los necesita en este PC: corren en su PC2).

```
docs/
└── CAPACITACION_H1_2.md          resumen de la capacitación Robotics 4.0: seguridad, modos,
                                  bajo nivel, arm_sdk, MuJoCo. Leer antes de escribir control

h1_2_joint_control/           Control y sintonización articular (SIN teleoperación)
├── README.md                     guía completa
├── config/gains.yaml             conjuntos de ganancias + topes de seguridad
├── docs/                         diagnóstico, arquitectura, seguridad, metodología, bitácora
├── h1_2_joint_control/           librería: joints, crc, client, metrics, trajectories
├── scripts/                      00_diagnose · 01_hold · 02_move · 03_tune · 04_sweep_arms · 05_plot
└── logs/                         CSV y gráficas de los ensayos

h1_2_teleoperation/
├── README.md                 ROBOT REAL: cómo lanzar teleop_xr.sh desde este PC
├── README_SIM.md             Despliegue en SIMULACIÓN (Isaac Sim + unitree_sim_isaaclab)
├── scripts/                  Robot real: se reenvían por SSH al PC2, donde corre todo
│   ├── teleop_xr.sh              teleoperación con las Quest 3 (brazos + manos); `parar` la detiene
│   ├── comun.sh                  el reenvío por SSH al robot; lo cargan los demás
│   ├── estado.sh                 estado del robot y CheckMode, ¿Debug de verdad? (solo lectura)
│   └── robot.sh                  comprobar red y clave, URL de las gafas, terminal en el robot
├── scripts_sim/              Simulación (lado laptop)
│   ├── _conda.sh                 localiza conda y el entorno `tv`
│   ├── 00_check_host.sh          diagnóstico del entorno `tv`: versiones, imports, certificados
│   ├── 01_install_host.sh        instalación del entorno conda `tv` y sus dependencias
│   ├── 02_gen_certs.sh           certificados TLS autofirmados para Vuer/WebXR (puerto 8012)
│   ├── 09_isolate_conda_env.sh   aísla los entornos conda del ROS/robotpkg del sistema
│   ├── 10_install_sim.sh         instalación del entorno conda `unitree_sim_env` (simulador)
│   ├── 11_launch_sim.sh          lanza unitree_sim_isaaclab con el H1-2 + manos Inspire
│   ├── 12_launch_teleop_sim.sh   lanza xr_teleoperate en modo `--sim` contra el simulador
│   ├── 13_check_sim.sh           diagnóstico del lado simulación
│   └── 14_test_inspire_ftp_bridge.py   prueba del puente FTP del simulador, sin Isaac Sim
└── patches/
    ├── inspire_sdkpy_uint16.patch              corrige un fallo de inspire_sdkpy (ver abajo)
    ├── unitree_sim_isaaclab_inspire_ftp.patch  añade las manos RH56DFTP (FTP) al simulador
    ├── xr_teleoperate_sim.patch                 control de flujo aiohttp + regulador de fps
    ├── televuer_image_format_knob.patch         formato de la imagen hacia el visor
    ├── teleimager_webrtc_warning.patch          quita un aviso falso a 90/s
    └── xr_teleoperate_h1_2_{dq_ref,tuning}.patch  velocidad de referencia y sintonización del
                                                 H1-2 (resultado de h1_2_joint_control; el
                                                 teleop actual del robot no los aplica)

ros_h1_2_ws/                  Workspace ROS 2
├── setup_env.sh                  entorno del workspace
├── docs/                         especificaciones
└── src/
    ├── h1_2_arm_control/         control de brazos: hold, goto, posturas, gravedad, gestos
    ├── h1_2_vision/              cámara de cabeza (D435i) por el servicio `videohub` del
    │                             robot, publicada como una cámara ROS normal
    └── h1_2_inspire_description/ modelo URDF/Xacro del H1-2 con las manos RH56DFTP,
                                  mallas y launches de visualización en RViz
```

## Procedencia de `h1_2_inspire_description`

Este paquete vivió hasta el 2026-09-15 en su propio repositorio,
[`h1_2_inspire_description`](https://github.com/smorales2405/h1_2_inspire_description),
y se integró aquí para tener un solo sitio donde trabajar. **A partir de ahora
los cambios se hacen y se suben en este repositorio.** El repositorio original
queda como registro histórico: conserva los 9 commits del desarrollo del URDF
(masa de las manos medida en balanza, espejado de la mano derecha, poda de
mallas sin usar, variantes para Pinocchio e Isaac Sim). Se trajo el árbol de
archivos, no la historia, para no arrastrar a este repositorio las mallas que
allí se borraron.

## Qué NO está aquí, y por qué

- **`xr_teleoperate`, `unitree_sdk2_python`, `inspire_hand_ws`,
  `unitree_sim_isaaclab`, `cyclonedds`** — repositorios de terceros. Se clonan
  (ver `README_SIM.md` §5). Los cambios a código ajeno están aislados en
  `patches/`.
- **El teleop del robot real** — vive y corre en el PC2 del robot
  (`~/robotics40/meta`, ya instalado). Aquí solo están los lanzadores que lo
  arrancan por SSH. El despliegue antiguo, con el teleop en la laptop, se retiró
  el 2026-10-09 y sigue en el historial de git
  (`git show 1df4e32:h1_2_teleoperation/README_DEPLOY.md`).
- **Certificados y claves** — en simulación, `cert.pem` / `key.pem` se generan
  localmente con `scripts_sim/02_gen_certs.sh`. La clave privada nunca debe
  subirse.

## Puesta en marcha: teleoperación con el robot real

En este PC no hay nada que instalar: basta la WiFi `UTEC_H1_2` y entrar al robot
por clave SSH. Con el robot **colgado** y en **Debug (L2 + R2)**:

```bash
cd h1_2_teleoperation/scripts
./robot.sh comprobar      # red, clave SSH y estado del robot (solo lectura)
./estado.sh               # CheckMode tiene que dar name ''
./teleop_xr.sh            # r empieza, q sale (abre las manos y los brazos vuelven a casa)
./teleop_xr.sh parar      # al terminar
```

En las Quest 3: `https://192.168.0.143:8012/?ws=wss://192.168.0.143:8012`.
Requisitos, seguridad, qué pasa si se corta el SSH y problemas frecuentes, en
[`h1_2_teleoperation/README.md`](h1_2_teleoperation/README.md).

## Control y sintonización articular

Antes de teleoperar conviene comprobar que **cada articulación sigue su
referencia**. Eso vive en [`h1_2_joint_control/`](h1_2_joint_control/), que es
independiente de `xr_teleoperate`: corre sobre `unitree_ros2` y no necesita ni
conda ni `unitree_sdk2py`.

```bash
cd h1_2_joint_control
source scripts/env.sh
python3 scripts/00_diagnose.py                       # solo lectura
python3 scripts/01_hold.py --seconds 10              # tomar el control sin mover
python3 scripts/02_move.py --joint L_elbow --traj step --amp 0.15
python3 scripts/03_tune.py --joint L_elbow --kp-list 50,80,110,140 --kd-list 2
python3 scripts/04_sweep_arms.py                     # las 14 articulaciones
```

Las ganancias que salen de ahí valen tal cual para `xr_teleoperate`: los dos
caminos escriben el mismo `kp`/`kd` en el mismo mensaje DDS.

## Simulación

El mismo `xr_teleoperate` corre contra
[`unitree_sim_isaaclab`](https://github.com/unitreerobotics/unitree_sim_isaaclab)
(Isaac Sim 5.1 + Isaac Lab), con el H1-2 de 27 DoF y manos Inspire. El comando de
teleoperación lleva los mismos argumentos que en el robot real salvo el flag `--sim`, porque
este repo añade al simulador el protocolo **FTP** de las RH56DFTP —el simulador
de Unitree solo hablaba el de las manos DFX del G1—.

La guía completa, con las once incidencias del procedimiento oficial, está en
[`README_SIM.md`](h1_2_teleoperation/README_SIM.md).

```bash
cd h1_2_teleoperation
./scripts_sim/11_launch_sim.sh          # terminal 1: simulador
./scripts_sim/12_launch_teleop_sim.sh   # terminal 2: teleoperación
```

## Hallazgos que condicionan el despliegue

Cosas descubiertas sobre el hardware que no están en ninguna documentación y sin
las cuales la teleoperación no funciona, o funciona mal:

1. **La lateralidad de las manos va al revés de la convención de Unitree.**
   `192.168.124.210` es la mano **derecha** y `.211` la **izquierda**. Ambas
   reportan `HAND_ID = 1` y ningún registro Modbus distingue lateralidad: solo se
   sabe mirando. Con la convención por defecto la teleoperación sale espejada.

2. **Las manos cuelgan del bridge `br0` (192.168.124.0/24)**, no de la red
   `192.168.123.x` de los ejemplos, y hablan Modbus TCP, no RS485.

3. **La cámara de cabeza (RealSense D435i) estaba cableada a PC1**, al que no
   tenemos acceso, y el robot la exponía por DDS con su servicio `videohub`, sin
   credenciales. El despliegue antiguo la llevaba a `teleimager` con un puente,
   y en eso se basa `ros_h1_2_ws/src/h1_2_vision`. Hoy la D435i está en el USB
   del PC2 (la ve `lsusb`, 2026-10-07), y el teleop del robot real la sirve
   directamente con `teleimager`.

4. **`inspire_sdkpy` revienta con valores negativos en `angle_set`** y se lleva
   por delante el hilo lector de DDS, dejando la mano sorda en silencio. Duele
   porque `-1` es el no-op del protocolo RH56. Lo corrige
   `patches/inspire_sdkpy_uint16.patch`.

5. **`H1_2_ArmController` arranca llevando los brazos a 0°** a 30 rad/s, antes de
   pulsar `r`, y no se puede evitar desde fuera. El antiguo `arm_joint_test.py`
   (en el historial de git) lo esquivaba con una subclase que fijaba el objetivo
   en la postura actual antes de que el hilo publicara nada. En el teleop actual
   sigue pasando: hay que lanzarlo sin nadie al alcance de los brazos.

6. **`rt/lowcmd` ya tiene dueño.** El controlador de alto nivel del robot (`ai`)
   publica ahí **a 500 Hz sin parar**. Un script que publique en el mismo tópico
   no lo sustituye: se alterna con él y el motor recibe consignas
   contradictorias. Eso explica los dos síntomas que se veían al mover
   articulaciones a bajo nivel —una que no llega a su referencia y otra que se
   mueve pero vibra— sin que las ganancias tengan nada que ver. El canal bueno
   para los brazos es **`rt/arm_sdk`**, que está libre y además deja las piernas
   al controlador del robot. Medidas y detalle en
   [`h1_2_joint_control/docs/01_DIAGNOSTICO.md`](h1_2_joint_control/docs/01_DIAGNOSTICO.md).

## Hardware

| Equipo | Rol |
|---|---|
| Unitree **H1-2** (brazos 7 DoF) | robot controlado |
| 2 × **Inspire RH56DFTP** (6 DoF, 17 zonas táctiles) | efectores finales |
| **Meta Quest 3** | dispositivo XR |
| PC Ubuntu 22.04 | lanza el teleop por SSH (robot real) · host de la simulación |
| **PC2** del robot (x86_64) | corre el teleop real: `xr_teleoperate`, servidor de imagen (D435i) y driver de manos |

## Créditos

- [`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) — Unitree Robotics
- [`inspire_hand_ws`](https://github.com/NaCl-1374/inspire_hand_ws) — SDK de las manos Inspire FTP
- [`TeleVision`](https://github.com/OpenTeleVision/TeleVision) — base del enfoque de teleoperación XR
