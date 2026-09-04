# h1_2_utec

Teleoperación XR del **Unitree H1-2** con manos **Inspire RH56DFTP** y **Meta Quest 3**,
sobre [`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) de Unitree.

Este repositorio contiene **solo lo propio**: los scripts escritos para este
despliegue y los parches al código de terceros. Los repositorios upstream no se
vendorizan — se clonan siguiendo las instrucciones de abajo.

```
h1_2_teleoperation/
├── README_DEPLOY.md          Guía completa del despliegue: topología, hallazgos, puesta en marcha
├── scripts/                  Lado HOST (laptop Ubuntu 22.04)
│   ├── 00_check_host.sh          diagnóstico: entorno, versiones, imports, certificados, red
│   ├── 01_install_host.sh        instalación del entorno conda `tv` y sus dependencias
│   ├── 02_gen_certs.sh           certificados TLS autofirmados para Vuer/WebXR (puerto 8012)
│   ├── 03_launch_teleop.sh       lanzador con los parámetros del H1-2 + manos FTP
│   ├── 04_test_inspire_dds_loopback.py   prueba de la cadena DDS de las manos SIN hardware
│   ├── arm_joint_test.py         primer movimiento de brazos, una articulación, en 3 peldaños
│   └── robot_pc2/            Lado ROBOT (PC2) — ejecución, no instalación
│       ├── inspire_ftp_dual_driver.py   puente Modbus ⇄ DDS de las dos manos
│       ├── videohub_image_bridge.py     sustituye a teleimager-server: videohub (DDS) → ZMQ
│       ├── hand_test.py                 prueba de manos: read / wiggle / sweep / open / forceclb
│       ├── probe_hands.py               sonda de solo lectura del estado de las manos
│       └── wait_for_camera.py           vigila la RealSense por si se conecta al PC2
└── patches/
    └── inspire_sdkpy_uint16.patch       corrige un fallo de inspire_sdkpy (ver abajo)
```

## Qué NO está aquí, y por qué

- **`xr_teleoperate`, `unitree_sdk2_python`, `inspire_hand_ws`** — repositorios de
  terceros, sin modificar. Se clonan (ver «Puesta en marcha»). El único cambio a
  código ajeno está aislado en `patches/`.
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
conda create -y -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
bash scripts/01_install_host.sh
bash scripts/02_gen_certs.sh
bash scripts/00_check_host.sh
```

El detalle completo —incluidas cuatro incidencias del procedimiento oficial que
no funcionan tal cual— está en [`README_DEPLOY.md`](h1_2_teleoperation/README_DEPLOY.md).

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
