# h1_2_utec

Teleoperación XR del **Unitree H1-2** con manos **Inspire RH56DFTP** y **Meta Quest 3**,
sobre [`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) de Unitree.

Este repositorio contiene **solo lo propio**: los scripts escritos para este
despliegue y los parches al código de terceros. Los repositorios upstream no se
vendorizan — se clonan siguiendo las instrucciones de abajo.

```
h1_2_joint_control/           Control y sintonización articular (SIN teleoperación)
├── README.md                     guía completa
├── config/gains.yaml             conjuntos de ganancias + topes de seguridad
├── docs/                         diagnóstico, arquitectura, seguridad, metodología, bitácora
├── h1_2_joint_control/           librería: joints, crc, client, metrics, trajectories
├── scripts/                      00_diagnose · 01_hold · 02_move · 03_tune · 04_sweep_arms · 05_plot
└── logs/                         CSV y gráficas de los ensayos

h1_2_teleoperation/
├── README_DEPLOY.md          Despliegue FÍSICO: topología, hallazgos, puesta en marcha
├── README_SIM.md             Despliegue en SIMULACIÓN (Isaac Sim + unitree_sim_isaaclab)
├── scripts/                  Lado HOST (laptop Ubuntu 22.04)
│   ├── 00_check_host.sh          diagnóstico: entorno, versiones, imports, certificados, red
│   ├── 01_install_host.sh        instalación del entorno conda `tv` y sus dependencias
│   ├── 02_gen_certs.sh           certificados TLS autofirmados para Vuer/WebXR (puerto 8012)
│   ├── 03_launch_teleop.sh       lanzador con los parámetros del H1-2 + manos FTP
│   ├── 04_test_inspire_dds_loopback.py   prueba de la cadena DDS de las manos SIN hardware
│   ├── arm_joint_test.py         primer movimiento de brazos, una articulación, en 3 peldaños
│   ├── 09_isolate_conda_env.sh   aísla los entornos conda del ROS/robotpkg del sistema
│   ├── 10_install_sim.sh         instalación del entorno conda `unitree_sim_env` (simulador)
│   ├── 11_launch_sim.sh          lanza unitree_sim_isaaclab con el H1-2 + manos Inspire
│   ├── 12_launch_teleop_sim.sh   lanza xr_teleoperate en modo `--sim` contra el simulador
│   ├── 13_check_sim.sh           diagnóstico del lado simulación
│   ├── 14_test_inspire_ftp_bridge.py   prueba del puente FTP del simulador, sin Isaac Sim
│   └── robot_pc2/            Lado ROBOT (PC2) — ejecución, no instalación
│       ├── inspire_ftp_dual_driver.py   puente Modbus ⇄ DDS de las dos manos
│       ├── videohub_image_bridge.py     sustituye a teleimager-server: videohub (DDS) → ZMQ
│       ├── hand_test.py                 prueba de manos: read / wiggle / sweep / open / forceclb
│       ├── probe_hands.py                sonda de solo lectura del estado de las manos
│       └── wait_for_camera.py            vigila la RealSense por si se conecta al PC2
└── patches/
    ├── inspire_sdkpy_uint16.patch              corrige un fallo de inspire_sdkpy (ver abajo)
    ├── unitree_sim_isaaclab_inspire_ftp.patch  añade las manos RH56DFTP (FTP) al simulador
    ├── xr_teleoperate_sim.patch                 control de flujo aiohttp + regulador de fps
    │                                            (el nombre engaña: hace falta TAMBIÉN en el robot real)
    ├── televuer_image_format_knob.patch         formato de la imagen hacia el visor
    └── teleimager_webrtc_warning.patch          quita un aviso falso a 90/s
```

## Qué NO está aquí, y por qué

- **`xr_teleoperate`, `unitree_sdk2_python`, `inspire_hand_ws`,
  `unitree_sim_isaaclab`, `cyclonedds`** — repositorios de terceros. Se clonan
  (ver «Puesta en marcha»). Los cambios a código ajeno están aislados en
  `patches/`.
- **Instalación del PC2** — el bundle offline (224 MB de ruedas), su instalador y
  el script de despliegue por SSH quedan fuera: son de puesta a punto, no de
  operación. `README_DEPLOY.md` §4 documenta el procedimiento por si hay que
  repetirlo.
- **Certificados y claves** — `cert.pem` / `key.pem` se generan localmente con
  `02_gen_certs.sh`. La clave privada nunca debe subirse.

## Puesta en marcha

Los scripts asumen la ruta `/home/utec/Documents/h1_2_teleoperation`. Si se
clona en otro sitio, ajustar la variable `ROOT` al inicio de cada `.sh`.

```bash
git clone https://github.com/smorales2405/h1_2_utec.git
cd h1_2_utec/h1_2_teleoperation

# repositorios upstream, junto a este directorio
git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd xr_teleoperate && git submodule update --init --depth 1 && cd ..
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
git clone https://github.com/NaCl-1374/inspire_hand_ws.git     # de aquí sale inspire_sdkpy

# parche al SDK de las manos
cd inspire_hand_ws && git apply ../patches/inspire_sdkpy_uint16.patch && cd ..

# entorno + certificados + comprobación
# OJO con pinocchio: el README de xr_teleoperate fija 3.1.0, pero las tres
# compilaciones de 3.1.0 que hay hoy en conda-forge para python 3.10 tienen el
# binding de `buildReducedModel` roto — falla con CUALQUIER combinacion de
# argumentos, incluida la que usa el propio `robot_wrapper.py` de pinocchio, y
# sin ella `H1_2_ArmIK` no se puede construir. Comprobado en 2026-09-08 con las
# builds py310hed69631_0/_1 y py310h4a8bb0c_2.
#   3.2.0 y 3.3.1 funcionan. Se usa 3.2.0, la mas cercana a la fijada.
#   Arrastra numpy 2.x; los imports funcionales pasan igual.
conda create -y -n tv python=3.10 pinocchio=3.2.0 -c conda-forge
bash scripts/01_install_host.sh
bash scripts/02_gen_certs.sh
bash scripts/00_check_host.sh
```

El detalle completo —incluidas cuatro incidencias del procedimiento oficial que
no funcionan tal cual— está en [`README_DEPLOY.md`](h1_2_teleoperation/README_DEPLOY.md).

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
teleoperación es idéntico al del robot real salvo por el flag `--sim`, porque
este repo añade al simulador el protocolo **FTP** de las RH56DFTP —el simulador
de Unitree solo hablaba el de las manos DFX del G1—.

La guía completa, con las once incidencias del procedimiento oficial, está en
[`README_SIM.md`](h1_2_teleoperation/README_SIM.md).

```bash
./scripts/11_launch_sim.sh          # terminal 1: simulador
./scripts/12_launch_teleop_sim.sh   # terminal 2: teleoperación
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

3. **La cámara de cabeza (RealSense D435i) está cableada a PC1**, al que no
   tenemos acceso. Pero el robot la expone por DDS mediante su servicio
   `videohub`, sin credenciales: `videohub_image_bridge.py` la convierte al
   formato de `teleimager` y `xr_teleoperate` funciona sin modificar.

4. **`inspire_sdkpy` revienta con valores negativos en `angle_set`** y se lleva
   por delante el hilo lector de DDS, dejando la mano sorda en silencio. Duele
   porque `-1` es el no-op del protocolo RH56. Lo corrige
   `patches/inspire_sdkpy_uint16.patch`.

5. **`H1_2_ArmController` arranca llevando los brazos a 0°** a 30 rad/s, antes de
   pulsar `r`, y no se puede evitar desde fuera. `arm_joint_test.py` lo esquiva
   con una subclase que fija el objetivo en la postura actual antes de que el
   hilo publique nada.

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
| Laptop Ubuntu 22.04 | host de teleoperación |
| **PC2** del robot (x86_64) | driver de manos + puente de imagen |

## Créditos

- [`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) — Unitree Robotics
- [`inspire_hand_ws`](https://github.com/NaCl-1374/inspire_hand_ws) — SDK de las manos Inspire FTP
- [`TeleVision`](https://github.com/OpenTeleVision/TeleVision) — base del enfoque de teleoperación XR
