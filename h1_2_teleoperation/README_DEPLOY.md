# Teleoperación XR del Unitree H1-2 con manos Inspire RH56DFTP

Despliegue **físico** (`Physical Deployment`) de
[`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) sobre:

| Equipo | Rol |
|---|---|
| Humanoide Unitree **H1-2** (brazos 7 DoF) | robot controlado |
| 2 × **Inspire RH56DFTP** (6 DoF, 12 juntas, 17 zonas táctiles) | efectores finales |
| **Meta Quest 3** | dispositivo XR (seguimiento de manos o mandos) |
| Laptop Ubuntu 22.04 (`utec-Precision-3581`) | **Host**: corre la teleoperación |
| **PC2** = `unitree-h1-2-pc4`, x86_64 | servicio de imagen + driver de manos |

> El **Simulation Deployment** (Isaac Sim / Isaac Lab / `unitree_sim_isaaclab`)
> vive en su propio documento: [`README_SIM.md`](README_SIM.md). Comparte el
> entorno conda `tv` y los scripts `0*`, así que las incidencias de §4 de este
> documento aplican también allí.

---

## 1. Estado

| Parte | Estado |
|---|---|
| Laptop (Host) | ✅ configurada y verificada |
| PC2: software | ✅ instalado (offline) y verificado |
| Manos RH56DFTP | ✅ **probadas a fondo**: 12/12 actuadores mueven, comandadas por DDS desde la laptop — ver §5 |
| Lateralidad de las manos | ✅ confirmada visualmente, y va **al revés** de la convención de Unitree — ver §2 |
| Cámara de cabeza (D435i) | ✅ resuelta por el servicio `videohub` del robot — ver §7 |
| Sensor de fuerza `.211` | ✅ resuelto: era el cero descalibrado, no mecánica — ver §8 |
| Router externo | ⏳ pendiente de conectar — ver §9 |
| Movimiento de articulaciones | ⏸️ no ejecutado, por indicación expresa |

Diagnóstico del host en cualquier momento:

```bash
bash ~/Documents/h1_2_teleoperation/scripts/00_check_host.sh
```

---

## 2. Topología real

Descubierta explorando el robot, **no** es la de los ejemplos de Unitree: las
manos no cuelgan de la red `192.168.123.x` sino de un **bridge `br0` en
`192.168.124.x`** que el PC2 arma con las bocas `net2`, `net3` y `net4` de una
tarjeta de cuatro puertos.

```
   Meta Quest 3
        │  WiFi · WebXR sobre HTTPS/WSS  →  https://<ip-laptop>:8012
        ▼
   ┌──────────────────── LAPTOP (Host) ─────────────────────┐
   │  conda env "tv"                                        │
   │  teleop_hand_and_arm.py --arm H1_2 --ee inspire_ftp    │
   │    · televuer → Vuer/WebXR                 :8012       │
   │    · pinocchio → IK de los 2 brazos (14 DoF)           │
   │    · dex-retargeting (DexPilot) → 6 DoF/mano           │
   │    · teleimager.image_client → recibe vídeo            │
   │  enp0s31f6 = 192.168.123.222                           │
   └───────────────────────────┬────────────────────────────┘
                               │  CycloneDDS dominio 0
   ┌───────────────────────────┴────────────────────────────┐
   │  H1-2                                                  │
   │   PC1  eth  192.168.123.161 ── control de bajo nivel   │
   │        rt/lowstate · rt/lowcmd · rt/arm_sdk            │
   │                                                        │
   │   PC2  eth0 192.168.123.164   ← SSH, DDS, teleimager   │
   │        br0  192.168.124.164   (net2+net3+net4)         │
   │        · inspire_ftp_dual_driver  Modbus ⇄ DDS         │
   │             rt/inspire_hand/{ctrl,state,touch}/{l,r}   │
   │        · teleimager-server  ZMQ 55555 · WebRTC 60001   │
   │                    │ Modbus TCP :6000 sobre br0        │
   │        ┌───────────┴───────────┐                       │
   │   Inspire RH56DFTP        Inspire RH56DFTP             │
   │   192.168.124.210         192.168.124.211              │
   └────────────────────────────────────────────────────────┘
```

### Direcciones confirmadas

| Nodo | IP | Comprobado |
|---|---|---|
| Laptop ↔ robot (`enp0s31f6`) | `192.168.123.222/24` | perfil NM `unitree-h1_2`, `never-default` |
| Laptop ↔ WiFi (`wlp0s20f3`) | DHCP (asignada por el router) | por aquí entra el Quest 3 |
| H1-2 PC1 | `192.168.123.161` | ping ✔ |
| H1-2 PC2 `eth0` | `192.168.123.164` | ping ✔ · SSH ✔ |
| H1-2 PC2 `br0` | `192.168.124.164` | ✔ |
| Mano **DERECHA** → tópico `r` | `192.168.124.210` | Modbus :6000 ✔ · confirmada visualmente |
| Mano **IZQUIERDA** → tópico `l` | `192.168.124.211` | Modbus :6000 ✔ · confirmada visualmente |

> ⚠️ **La lateralidad va al revés de la convención de Unitree.** Sus ejemplos
> asignan `.210 → left`, `.211 → right`. En este robot es lo contrario,
> comprobado moviendo un dedo a la vez y mirando cuál se movía. Ambas manos
> reportan `HAND_ID = 1` y **no existe registro Modbus que distinga izquierda de
> derecha**: la única forma de saberlo es mirar.
>
> Si se hubiera dejado la convención por defecto, la teleoperación habría salido
> **espejada**: la mano izquierda del operador moviendo la derecha del robot.
> `inspire_ftp_dual_driver.py` ya lleva la asignación correcta en `LEFT_IP` /
> `RIGHT_IP`. Si alguna vez se cambian las manos de sitio, repetir:
>
> ```bash
> ~/teleop_venv/bin/python ~/hand_test.py wiggle 192.168.124.210 --dof 3
> ```

El perfil ethernet de la laptop lleva `ipv4.never-default yes`, así que el robot
no se lleva la ruta por defecto y la laptop conserva Internet por WiFi mientras
teleopera.

---

## 3. Instalado en la laptop (Host)

Entorno conda `tv`, Python 3.10.21, en `/home/utec/miniconda3/envs/tv`.

| Paquete | Versión | Nota |
|---|---|---|
| `pinocchio` | 3.1.0 (conda-forge) | cinemática; versión fijada por el README de Unitree |
| `numpy` | 1.26.4 | **fijado**; `televuer`/`dex-retargeting` exigen `<2.0` |
| `torch` | 2.3.0**+cpu** | solo lo usa `dex-retargeting`; ruedas CPU (~190 MB vs ~2.5 GB) |
| `params-proto` | **2.13.2** | ver §4 |
| `vuer` | 0.0.60 | servidor WebXR |
| `cyclonedds` | 0.10.2 | la que exige `unitree_sdk2_python` |
| `unitree_sdk2py` | 1.0.1 | commit 2026-07-21, muy posterior al mínimo `404fe44` |
| `inspire_sdkpy` | 1.0.0 | IDL + driver de las manos FTP |
| `pymodbus` | 3.6.9 | misma versión que tu `inspire_hand_interface` |
| `televuer`/`teleimager`/`dex_retargeting` | 4.0.0 / 1.6.0 / 0.4.7 | editables, desde los submódulos |

Certificados TLS autofirmados (10 años) en `~/.config/xr_teleoperate/` y en
`xr_teleoperate/teleop/televuer/`.

```
/home/utec/Documents/h1_2_teleoperation/
├── xr_teleoperate/       repo principal + 3 submódulos
├── unitree_sdk2_python/  SDK de comunicación con el robot
├── inspire_hand_ws/      SDK de las manos FTP (de aquí sale inspire_sdkpy)
├── pc2_offline/          bundle offline que se instaló en el PC2 (224 MB)
├── scripts/              ver §6
└── README_DEPLOY.md
```

---

## 4. Instalado en el PC2 — y por qué fue offline

El PC2 **no tiene salida a Internet**: su netplan solo define `eth0`
(192.168.123.164) y `br0` (192.168.124.164), sin ruta por defecto. Intenté
darle Internet haciendo NAT desde la laptop, pero esa operación quedó bloqueada
por permisos, así que **todo se instaló offline** — mejor resultado, además:
no se tocó ni un byte de la configuración de red del robot.

El bundle `pc2_offline/` (224 MB: 50 ruedas + fuentes) se copia con `rsync` y
se instala con `install_pc2.sh`. Quedó en `~/teleop_venv` sobre el **Python
3.10.12 del sistema** —misma versión menor que el entorno `tv` de la laptop—
con `unitree_sdk2py` 1.0.1, `inspire_sdkpy` 1.0.0, `teleimager` 1.6.0,
`pymodbus` 3.6.9 y `cyclonedds` 0.10.2.

Dos obstáculos más del PC2 y cómo se resolvieron:

- **No tiene el paquete `python3.10-venv`**, así que `python3.10 -m venv` falla
  al llegar a `ensurepip` — y sin Internet no se puede `apt install`. Se crea
  el entorno con `--without-pip` y se arranca pip **ejecutándolo desde dentro
  de su propia rueda** (`python pip-26.2.1.whl/pip install ...`). No hace falta
  root.
- **Los envs conda que ya existían no sirven**: `base` es Python 3.12 (y
  `teleimager` pide `<3.12`), `unitree` es 3.8 con `cyclonedds` 0.10.5 y el
  obsoleto `unitree_dds_wrapper`, y `h1_arm` está vacío. Se dejaron intactos.

`~/inspire_hand_ws` ya existía en el PC2 desde mayo de 2025, pero nunca se
había instalado en ningún intérprete (`import inspire_sdkpy` fallaba en los
tres envs) y su submódulo `unitree_sdk2_python` estaba en un commit de
septiembre de 2024. Se instaló la copia al día que trae el bundle.

### Incidencias del lado laptop

**1. `vuer` 0.0.60 se rompe con `params-proto` ≥ 3.** `vuer/server.py` importa
`Flag`, eliminado en la 3.x. pip resuelve a 3.3.0 y `vuer/__init__.py` se traga
el `ImportError` en un `try/except`, así que en vez de un error claro sale un
mensaje engañoso pidiendo `pip install 'vuer[all]'` —que ya estaba instalado—.
Se fija `params-proto==2.13.2`.

**2. `dex-retargeting` declara `pin` (pinocchio de PyPI).** Instalarla
machacaría el `pinocchio` 3.1.0 de conda. Se instala con `--no-deps`.

**3. `--ee inspire_ftp` necesita `inspire_sdkpy`, que no está en PyPI ni lo
menciona el README.** Viene de
[`NaCl-1374/inspire_hand_ws`](https://github.com/NaCl-1374/inspire_hand_ws).
El README §3.2 de `xr_teleoperate` manda a `DFX_inspire_service`, que es para
las manos **DFX** (`rt/inspire/cmd|state`): para las RH56DFTP no sirve.

**4. `inspire_sdkpy` ignora el argumento `network`** — condición invertida en
`inspire_sdk.py`:

```python
if network is None:
    ChannelFactoryInitialize(0, network)   # network es None
else:
    ChannelFactoryInitialize(0)            # ← se pierde la interfaz
```

`inspire_ftp_dual_driver.py` lo esquiva: inicializa DDS por su cuenta y
construye los handlers con `initDDS=False`. Importa aquí porque el PC2 tiene
varias NIC (`eth0`, `br0`, `net1..4`).

**5. `inspire_sdkpy` revienta con cualquier valor negativo en `angle_set`** — y
se lleva por delante el hilo lector de DDS. Pasa los valores crudos de DDS a
pymodbus, que empaqueta con `struct.pack(">H")`:

```
struct.error: argument out of range
```

Duele porque el protocolo de la RH56 usa **`-1` como no-op** ("deja este DOF
como está"), que es justo lo que hace falta para mover un dedo sin tocar los
otros. Peor: la excepción sube por `__ChannelReaderThreadFunc` y mata el hilo,
así que **la mano se queda sorda hasta reiniciar el driver**, en silencio.

Parcheado en `inspire_hand_ws/inspire_hand_sdk/inspire_sdkpy/` (y propagado al
bundle y al PC2): un `_u16()` que enmascara a 16 bits preservando el complemento
a dos, más un `try/except` para que ningún mensaje malformado vuelva a tumbar el
hilo. No afecta a la teleoperación normal —`Inspire_Controller_FTP` siempre
manda 0..1000— pero sí a cualquier diagnóstico que use el no-op.

**6. `ModbusDataHandlerDouble` (ruta RS485) espeja las dos manos.** Registra
**el mismo callback** para `ctrl/l` y `ctrl/r`, sobrescribe `self.sub`, y
escribe cada comando a **los dos** `device_id`. Sobre RS485 las manos quedarían
espejadas y no se pueden controlar por separado. En este robot las manos van por
Modbus TCP (una IP cada una), que no tiene el problema; esa ruta queda solo como
respaldo de diagnóstico y con el fallo documentado en el propio código.

---

## 5. Verificaciones ejecutadas

### En la laptop, sin hardware

| Prueba | Resultado |
|---|---|
| `H1_2_ArmIK()` + `solve_ik()` (pinocchio + CasADi + URDF) | ✔ carga en 4 s, resuelve en 25 ms |
| `HandRetargeting(INSPIRE_HAND)` (DexPilot) | ✔ mapa retarget→hardware `[4,6,2,0,9,8]` |
| Servidor Vuer con TLS en `:8012` | ✔ handshake con el certificado generado |
| Cadena DDS de las manos contra un PC2 simulado | ✔ `scripts/04_test_inspire_dds_loopback.py` |

### Contra el robot real

**Lectura Modbus de ambas manos** (solo lectura, sin escribir ningún registro):
las dos responden en `:6000`, `ERROR = 0` en los 12 actuadores, temperaturas
38-40 °C.

**Driver Modbus ⇄ DDS y recepción en la laptop:** los cuatro tópicos
(`state/l`, `state/r`, `touch/l`, `touch/r`) llegan desde el PC2 a la laptop
por `enp0s31f6`, con **1062 taxeles en 17 zonas por mano**. Los `angle_act`
recibidos por DDS coinciden exactamente con los leídos por Modbus.

**Rendimiento medido en este robot**, por mano:

| Modo | Frecuencia |
|---|---|
| `--no-touch` (solo `angle_act` + `status`) | **274 Hz** |
| con los 17 sensores táctiles | **35 Hz** |

Los táctiles cuestan ~8× la frecuencia del lazo. Como `Inspire_Controller_FTP`
emite comandos a 100 Hz y solo consume `angle_act`, para teleoperar conviene
`--no-touch`; los táctiles se activan cuando se quieran grabar.

**Prueba funcional de las dos manos** (`scripts/robot_pc2/hand_test.py`), con
velocidad baja (`SPEED_SET = 150/1000`), cierre parcial hasta 500, `-1` en los
DOF que no se tocan, vigilancia de fuerza con aborto y restauración al terminar:

- **12/12 actuadores mueven correctamente**, `ERROR = 0` en todos, ningún aborto
  por fuerza. Fuerzas durante el movimiento entre −92 y +46 gf.
- **Orden de DOF confirmado visualmente**: `[meñique, anular, medio, índice,
  pulgar-flexión, pulgar-rotación]` — se movió el DOF 3 y el usuario confirmó
  que era el índice.
- **Lateralidad confirmada visualmente** y contraria a la convención de Unitree
  (§2).

**Comando extremo a extremo por DDS**, que es la ruta exacta de la
teleoperación: la laptop publica en `rt/inspire_hand/ctrl/{l,r}` con
`mode = 0b1001` (ángulo + velocidad) y el índice de cada mano responde:

| | inicial | comandado a 500 | de vuelta a 1000 |
|---|---|---|---|
| izquierda (`/l` → `.211`) | 999 | **499** | 999 |
| derecha (`/r` → `.210`) | 997 | **500** | 999 |

Solo se movió el DOF 3; el resto quedó intacto gracias al `-1`. Sin errores en
el driver. Esto valida la cadena completa
**laptop → DDS → PC2 → Modbus → mano**.

Tras las pruebas el driver se detuvo y ambas manos quedaron abiertas, con
fuerzas entre −93 y +46 gf y `ERROR = 0`.

**Cadena de imagen completa** (§7), contra el robot real:
`PC1/videohub → DDS → puente → ZMQ → ImageClient → Vuer`. El `ImageClient` de
xr_teleoperate recibe el `cam_config`, los fotogramas llegan a 15 Hz con el
tamaño que declara `image_shape`, y `render_to_xr()` escribió 199 fotogramas a
33 Hz sin errores mientras Vuer servía HTTPS en `:8012`.

---

## 6. Puesta en marcha

### 6.1 Una sola vez (ya hecho)

```bash
rsync -a pc2_offline/ unitree@192.168.123.164:~/pc2_offline/
ssh unitree@192.168.123.164 'bash ~/pc2_offline/install_pc2.sh'
```

La laptop ya tiene su clave pública en el PC2: `ssh unitree@192.168.123.164`
entra sin contraseña.

### 6.2 Cada sesión

**Terminal 1 — PC2, driver de las manos**

```bash
ssh unitree@192.168.123.164
~/teleop_venv/bin/python ~/inspire_ftp_dual_driver.py \
    --transport tcp --network eth0 --no-touch     # añade --full-state para diagnóstico
```

Las IP ya vienen por defecto **con la lateralidad correcta** (`.211` = izquierda,
`.210` = derecha; ver §2). Quita `--no-touch` si quieres los táctiles (baja de
274 Hz a 35 Hz).

Conviene tarar las fuerzas antes de una sesión, con las manos abiertas y libres:

```bash
~/teleop_venv/bin/python ~/hand_test.py forceclb 192.168.124.211
~/teleop_venv/bin/python ~/hand_test.py forceclb 192.168.124.210
```

**Terminal 2 — PC2, puente de imagen**

La D435i está en PC1 y no podemos abrirla en local, así que en vez de
`teleimager-server` corre el puente, que saca la imagen del servicio `videohub`
del robot por DDS y la republica en el mismo formato (§7):

```bash
~/teleop_venv/bin/python ~/videohub_image_bridge.py --network eth0
```

Opciones útiles: `--width/--height` (por defecto 1280×720), `--quality`,
`--passthrough` (publica el 1920×1080 original sin recomprimir).

**Terminal 3 — laptop, teleoperación**

```bash
cd ~/Documents/h1_2_teleoperation
./scripts/03_launch_teleop.sh
```

```bash
EXTRA="--record"      ./scripts/03_launch_teleop.sh   # grabar episodios
INPUT_MODE=controller ./scripts/03_launch_teleop.sh   # mandos en vez de manos
EXTRA="--motion"      ./scripts/03_launch_teleop.sh   # con locomoción
```

**En el Quest 3**

1. Misma WiFi que la laptop; activar seguimiento de manos en Ajustes.
2. Navegador → `https://<ip-wifi-de-la-laptop>:8012` → *Advanced* →
   *Proceed to … (unsafe)*. Solo la primera vez.
3. Botón **Virtual Reality** y aceptar permisos.

**Secuencia de control**

1. Colocar los brazos imitando la **pose inicial del robot** antes de empezar.
2. `r` en la Terminal 3 → el robot empieza a seguir.
3. `s` inicia/guarda grabación (solo con `--record`).
4. `q` para salir. **Antes de pulsar `q`, llevar los brazos cerca de la pose
   inicial**: al salir el robot los recoloca en 5 s y un salto grande puede
   dañarlo.

### Scripts

| Script | Dónde | Qué hace |
|---|---|---|
| `00_check_host.sh` | laptop | diagnóstico: entorno, versiones, imports, certificados, red |
| `01_install_host.sh` | laptop | instalación del host (ya ejecutado; idempotente) |
| `02_gen_certs.sh` | laptop | regenera `cert.pem`/`key.pem`; acepta IPs extra |
| `03_launch_teleop.sh` | laptop | lanza la teleoperación con los parámetros H1-2 + FTP |
| `04_test_inspire_dds_loopback.py` | laptop | prueba de la cadena DDS sin hardware |
| `05_deploy_to_pc2.sh` | laptop | copia certificados y scripts al PC2 |
| `pc2_offline/install_pc2.sh` | PC2 | instalación offline completa |
| `robot_pc2/inspire_ftp_dual_driver.py` | PC2 | puente Modbus ⇄ DDS de las dos manos |
| `robot_pc2/hand_test.py` | PC2 | prueba de manos: `read` / `wiggle` / `sweep` / `open` / `forceclb` (§8) |
| `robot_pc2/videohub_image_bridge.py` | PC2 | **sustituye a teleimager-server**: videohub (DDS) → ZMQ (§7) |
| `robot_pc2/wait_for_camera.py` | PC2 | vigila la D435i por si algún día se conecta al PC2 (§7) |
| `robot_pc2/probe_hands.py` | PC2 | sonda **solo lectura** del estado de las manos |

---

## 7. ✅ Resuelto: la cámara, vía el servicio `videohub`

La cámara de cabeza es una **Intel RealSense D435i**, y está cableada a **PC1**
(el computador de locomoción), no al PC2. Comprobado con cinco pruebas
independientes en el PC2:

| Comprobación | Resultado |
|---|---|
| `lsusb` | ningún dispositivo Intel (la D435i enumera como `8086:0b3a`) |
| `/dev/video*` | vacío; `uvcvideo` ni siquiera cargado |
| `rs-enumerate-devices` | `No device detected. Is it plugged in?` |
| `pyrealsense2` | 0 cámaras |
| `dmesg` desde el arranque | ningún evento USB de vídeo |

Sin errores de USB de ningún tipo: el dispositivo no está en el bus, no es una
conexión defectuosa. (El PC2 sí está *preparado* para ella: `librealsense`
compilada en `/usr/local/bin`, reglas udev instaladas y un
`~/.realsense-config.json` de agosto de 2024.)

**PC1 no acepta la credencial que tenemos**, así que ni se puede mover el cable
por software ni instalar nada allí. Pero no hace falta.

### El robot ya publica su cámara por DDS

En `ROS_DOMAIN_ID=0` corre el stack propio del robot, y entre sus tópicos están
`/api/videohub/request` y `/api/videohub/response` — con **un suscriptor activo**.
Es el servicio `videohub` de Unitree, el mismo que expone `VideoClient` en
`unitree_sdk2py` (`go2/video/video_client.py`, `VIDEO_SERVICE_NAME = "videohub"`).

Responde a cualquiera en el dominio DDS 0, **sin credenciales**:

```python
from unitree_sdk2py.go2.video.video_client import VideoClient
c = VideoClient(); c.SetTimeout(5.0); c.Init()
code, data = c.GetImageSample()      # -> JPEG 1920x1080
```

Medido en este robot:

| Métrica | Valor |
|---|---|
| Resolución | **1920×1080** |
| Tamaño por fotograma | ~77-82 KB (JPEG) |
| Latencia por petición | media **6.4 ms**, mediana 6.5, máx 7.3 |
| Tasa de petición sostenible | 151 Hz |
| **Fotogramas NUEVOS** | **~15 Hz** (53 de 59 respuestas eran repeticiones) |

Los 15 Hz son el límite real: el servicio devuelve el último fotograma en caché,
así que pedir más rápido no aporta. Es menos que los 30 fps que asume el
repositorio, pero perfectamente usable para teleoperar.

### El puente

`videohub_image_bridge.py` sustituye a `teleimager-server` sin necesitar la
cámara en local. Habla **exactamente** el mismo protocolo, así que el
`ImageClient` de `xr_teleoperate` funciona **sin modificar una línea del repo**:

```
videohub (DDS, PC1) --JPEG 1920x1080--> [puente en PC2] --ZMQ PUB :55555--> laptop
                                                        --ZMQ REP :60000--> cam_config
```

Reutiliza las clases del propio `teleimager` (`ZMQ_Responser`,
`ZMQ_PublisherManager`) para que el formato no se desvíe. Reescala a 1280×720 y
recomprime (configurable; `--passthrough` publica el 1920×1080 original tal
cual).

```bash
# en el PC2
~/teleop_venv/bin/python ~/videohub_image_bridge.py --network eth0
```

Detalle que importa: `image_shape` del `cam_config` **debe** coincidir con el
tamaño real de los fotogramas, porque televuer reserva memoria compartida de
exactamente ese tamaño y escribe encima. El puente lo garantiza reescalando.

### Verificado extremo a extremo

| Eslabón | Resultado |
|---|---|
| `VideoClient.GetImageSample()` desde el PC2 | ✔ `code=0`, JPEG 1920×1080 válido |
| Puente publicando | ✔ 30 Hz publicados · 15 Hz nuevos · 0 fallos |
| `ImageClient` de xr_teleoperate desde la laptop | ✔ `cam_config` recibido; 15 Hz de fotogramas distintos |
| Tamaño recibido vs. `image_shape` | ✔ 1280×720 coincide |
| `TeleVuerWrapper.render_to_xr()` | ✔ 199 fotogramas escritos a 33 Hz, sin errores |
| Vuer sirviendo HTTPS en `:8012` | ✔ con la config real del robot |

Cadena completa validada:
**PC1/videohub → DDS → puente → ZMQ → ImageClient → Vuer → Quest**.

> ⚠️ **La D435i es monocular**, así que `binocular: false` y el operador verá la
> misma imagen en ambos ojos, sin estereoscopía. El repositorio asume por defecto
> una binocular que manda las dos vistas lado a lado en un fotograma de 480×1280.
>
> ⚠️ **Hay algo tapando parte de la vista.** En las capturas
> (`camera_test/*.jpg`) un objeto oscuro —parece una correa o un arnés— ocupa
> cerca de un tercio del encuadre, justo en el centro. Conviene apartarlo antes
> de teleoperar.

### Si algún día se quiere la cámara en el PC2

Bastaría mover el cable USB de la D435i de PC1 a un puerto USB 3 libre del PC2
(el hub Terminus de 7 puertos solo usa 2). `pyrealsense2` ya está instalado, y
`wait_for_camera.py` detecta la cámara en cuanto enumere y escupe el bloque de
`cam_config_server.yaml` con su número de serie. Eso daría 30 fps y acceso a la
profundidad, a cambio de quitársela a PC1.

---

## 8. ✅ Resuelto: la mano `.211` no estaba atascada

Al explorar el robot, la mano en `.211` leía el anular y el medio cerrados
(`ANGLE_ACT = 0`, `POS_ACT` ~7970/7988 contra 28-245 en la otra mano) y el medio
marcaba **2600 gf ≈ 25 N constantes**. Con `CURRENT = 0`, parecía una carga
mecánica sostenida, así que lo reporté como posible atasco y no toqué nada.

**No era eso.** Al probarla con permiso:

1. El índice, que también estaba cerrado, **abrió con normalidad** (`ANGLE_ACT`
   0 → 786, corriente fluyendo, 140 gf).
2. El anular abrió perfecto: `POS_ACT` 7967 → 101, fuerza 6 → 32 gf.
3. El medio **también abrió del todo** — `POS_ACT` 7989 → **4**, más abierto
   incluso que la mano sana — y sin embargo `FORCE_ACT` seguía clavado en
   2595 gf.

Un dedo totalmente abierto, sin tocar nada, con corriente cero y una lectura de
fuerza inmóvil en 2595: eso no es mecánica, es el **cero del sensor de fuerza
descalibrado**. Encaja con que la lectura fuera idéntica (2600/2599/2601) en
tomas separadas: una fuerza de contacto real fluctúa.

Comparación con la palma abierta y libre, antes de tarar:

| DOF | meñique | anular | **medio** | índice | pulg-flex | **pulg-rot** |
|---|---|---|---|---|---|---|
| `.210` (derecha) | −43 | +17 | −8 | +16 | +41 | −12 |
| `.211` (izquierda) | +39 | +15 | **+2596** | +109 | +124 | **+394** |

**Solución: `forceClb` (registro 1009)**, tu procedimiento estándar — abrir la
palma y tarar. Las seis fuerzas de `.211` pasaron de
`[28, 17, 2595, 93, 126, 415]` a **`[0, 0, 0, 0, 0, 0]`**:

```bash
~/teleop_venv/bin/python ~/hand_test.py forceclb 192.168.124.211
```

Después, el barrido de los 6 DOF dio fuerzas entre −73 y +5 gf, con `ERROR = 0`.

> **Antes de cualquier sesión que use la fuerza**, tara ambas manos con la palma
> abierta y sin tocar nada. La `.210` sigue con su cero de fábrica (offsets de
> ±45 gf, tolerables pero no cero); si vas a comparar fuerzas entre manos,
> conviene tararla también.

### `hand_test.py` — salvaguardas

El script que se usó vive en `scripts/robot_pc2/hand_test.py` y está en el PC2
como `~/hand_test.py`. Subcomandos: `read` (no mueve nada), `wiggle` (un dedo),
`sweep` (los 6 DOF), `open`, `forceclb`. Protecciones:

- `SPEED_SET` bajo por defecto (150/1000). Tu caracterización mostró que el
  sobreimpulso de fuerza lo domina la velocidad de cierre, así que se mueve
  despacio a propósito.
- `-1` en todos los DOF que no se tocan: no-op real, verificado.
- Aborto por `|FORCE_ACT|` sobre el umbral (`--fmax`, 800 gf por defecto).
- El cierre nunca llega al tope: se para en `--target` (500 por defecto).
- Restauración del estado inicial al terminar, por Ctrl-C o por excepción.
- **Nunca** escribe el registro SAVE (1005): ningún cambio queda persistido.

---

## 9. Pendiente

1. **Apartar lo que tapa la cámara.** En las capturas de `camera_test/` un
   objeto oscuro —parece una correa o un arnés— ocupa cerca de un tercio del
   encuadre, justo en el centro (§7).
2. **Tarar también la mano `.210`** si vas a comparar fuerzas entre manos (§8).
3. **Router externo.** Cuando laptop y robot se conecten al router, la IP WiFi
   de la laptop cambiará. Hay que regenerar el certificado —el script detecta
   solo las IP vivas— y volver a aceptar el aviso en el Quest:

   ```bash
   ./scripts/02_gen_certs.sh
   ```

   El enlace ethernet directo laptop↔robot (`192.168.123.222` ↔ `.164`) puede
   quedarse como está: es el que lleva DDS y conviene dedicado. El router solo
   necesita dar WiFi común a la laptop y al Quest.
4. **Primer movimiento de los BRAZOS.** No se ha ejecutado ninguno. Ver §11:
   empezar por `scripts/arm_joint_test.py` (una sola articulación, 200× más
   lento) y **no** por la teleoperación completa, con el robot colgado del arnés.

---

## 10. Nota sobre la velocidad de cierre de las manos

`Inspire_Controller_FTP` publica `mode = 0b0001` (solo ángulo) y nunca escribe
`speed_set`: las manos cierran a la velocidad guardada en su configuración.

Tu caracterización del RH56DFTP mostró que el sobreimpulso de fuerza en contacto
está dominado por la velocidad de cierre (hasta ~3300 gf) y que el modo híbrido
lo reduce entre 30× y 82×. Al teleoperar contra objetos se está justo en ese
régimen — y los 2600 gf que ahora marca el dedo medio de la mano `.211` dan una
idea de lo que aguanta la mecánica cuando algo se queda cargado.

Si quieres acotarlo, el cambio mínimo está en
`xr_teleoperate/teleop/robot_control/robot_hand_inspire.py`, `_send_hand_command`:

```python
left_cmd_msg.angle_set = left_angle_cmd_scaled
left_cmd_msg.speed_set = [400] * 6      # 0..1000
left_cmd_msg.mode = 0b1001              # ángulo + velocidad
```

No lo dejé aplicado: cambia el comportamiento respecto al repositorio original y
el valor sale de tus propios datos, no de un número por defecto.

El mismo método tiene un bloque de depuración que imprime los 50 primeros
comandos publicados (`self._debug_count`); es del repositorio original y solo
ensucia el log al arrancar.


---

## 11. Primer movimiento de los brazos

### Qué pasa al arrancar, antes de tocar nada

Dos efectos del arranque que no son obvios y conviene tener claros:

**1. Construir `H1_2_ArmController` energiza el robot ENTERO, no solo los
brazos.** Su `__init__` recorre `H1_2_JointIndex` completo —las 27
articulaciones, piernas incluidas— y fija `mode = 1`, ganancias
(`kp_high = 300 / kd_high = 5` para las fuertes) y `q = la posición actual de
cada una`. Acto seguido arranca el hilo que publica `rt/lowcmd` a 250 Hz. O sea:
**nada más lanzar el programa, y antes de pulsar `r`, el robot se pone rígido en
la postura en la que esté**.

**2. Entrar en modo debug suelta el control de locomoción.**
`MotionSwitcher.Enter_Debug_Mode()` llama a `ReleaseMode()` hasta que no queda
ningún modo activo. Si el robot se estaba sosteniendo solo, deja de equilibrarse.

**3. El controlador arranca llevando los brazos a 0°.** `__init__` fija
`self.q_target = np.zeros(14)` en su primera línea y lanza el hilo publicador en
la última. En cuanto el constructor retorna, ese hilo ya está publicando a
250 Hz con objetivo **cero**, y `clip_arm_q_target` avanza hasta
`30 rad/s × 1/250 s = 0.12 rad` por paso. Las 14 articulaciones de los brazos
barren hacia 0° en un par de segundos, **antes de pulsar `r`**.

Es intencionado en el repositorio —cero es la "home" a la que también vuelve al
salir, y por eso el README pide alinear tus brazos con la pose inicial antes de
empezar— pero para una primera prueba cautelosa es justo lo contrario de lo que
se quiere. Y no se puede evitar desde fuera: bajar `arm_velocity_limit` después
de construir llega tarde.

`arm_joint_test.py` lo resuelve con una subclase que intercepta el propio hilo.
Su `target` es `self._ctrl_motor_state`, que Python resuelve a la subclase, así
que lo primero que ejecuta —antes de publicar nada— es fijar el objetivo en la
postura **actual** y bajar el límite de velocidad:

```python
class _ControladorSeguro(H1_2_ArmController):
    def _ctrl_motor_state(self):
        self.set_arm_velocity_limit(velocity_limit)
        actual = self.get_current_dual_arm_q()
        with self.ctrl_lock:
            self.q_target = actual
            self.tauff_target = np.zeros(14)
        super()._ctrl_motor_state()
```

El script mide la deriva entre la postura leída antes de tomar el control y la
de después, y la reporta: si sale por debajo de 0.02 rad, los brazos no se
movieron.

> ✅ **`L2 + B` SÍ funciona en modo debug.** El manual del H1 lo dice
> explícitamente: dentro del modo debug, `L2 + B` sale de vuelta al estado de
> amortiguación. Es el paro de emergencia y hay que tenerlo presente durante toda
> la prueba. *(Corrige una afirmación anterior de esta guía, que lo daba por no
> disponible extrapolando de un caso del G1.)*
>
> Aun así el arnés sigue siendo obligatorio: al pasar a amortiguación el robot
> **se deja caer despacio hasta el suelo**, que es justo lo que el arnés evita.

### Modos del mando del H1

| Modo | Combinación |
|---|---|
| Amortiguación (Damping) — **paro de emergencia** | `L2 + B` |
| Par cero (Zero Torque) | `L2 + Y` |
| Preparado (Ready) — desde amortiguación | `L2 + UP` |
| Movimiento (Motion) | `R2 + X` |
| **Debug — el que hay que usar para el SDK** | `L2 + R2` |

El manual es explícito: *"cuando se usa el SDK para desarrollo y depuración,
asegúrate de que el robot esté en modo Debug para detener el programa de control
de movimiento"*, y así evitar conflictos de comandos y temblores.

**El H1-2 no soporta sentarse.** Así que la postura de trabajo para las pruebas
es **colgado del arnés**, con los pies sin carga o rozando el suelo.

### Estado leído del robot (2026-09-02, solo lectura)

```
mode_machine = 6      MotionSwitcher.CheckMode() -> (0, {'form': '0', 'name': 'ai'})

  idx articulacion       q (rad)   q (deg)    tau_est
    3 L_elbow_pitch        1.478      84.7      -0.18
   10 R_elbow_pitch        1.508      86.4       0.18
   (las otras 12 entre -0.20 y +0.09 rad, es decir ±11°)

|tau| máximo en piernas y torso: 0.28 Nm
```

Tres cosas que se leen de ahí:

- El robot está en modo **`ai`**: el servicio de locomoción está corriendo. Hay
  que sacarlo de ahí antes de mandar `rt/lowcmd`.
- **Los dos codos están flexionados ~85°** y el resto de articulaciones casi en
  cero. Si algo mandara los brazos a 0°, los codos barrerían esos 85° de golpe.
  Es exactamente el riesgo del arranque descrito arriba, ahora con número.
- Pares residuales de 0.28 Nm como máximo: el robot no está soportando carga.

### La escalera recomendada

`scripts/arm_joint_test.py` implementa tres peldaños, de menos a más
compromiso. Con el robot **colgado del arnés** y en **modo Debug** (`L2 + B` para
amortiguación, luego `L2 + R2` para debug):

```bash
conda activate tv
cd ~/Documents/h1_2_teleoperation

# 1. Solo lectura. No publica nada. Modo actual + ángulo/par/temperatura de las 14.
python scripts/arm_joint_test.py read --network enp0s31f6

# 2. Toma el control y sostiene la postura ACTUAL. No comanda movimiento.
python scripts/arm_joint_test.py hold --network enp0s31f6 --seconds 10

# 3. Mueve UNA articulación un ángulo pequeño y vuelve.
python scripts/arm_joint_test.py move --network enp0s31f6 \
    --joint L_elbow_pitch --delta 0.15
```

El peldaño 3 baja `arm_velocity_limit` de **30.0 rad/s a 0.15** —200× más
lento—, mueve 0.15 rad (≈8.6°), vigila `tau_est` con aborto a 25 Nm, comprueba
que la articulación sigue la consigna, y restaura el ángulo inicial al salir.

**Las otras 13 articulaciones se mantienen en su ángulo actual, no en 0°.** El
objetivo se construye como `destino = q_actual.copy()` y solo se modifica el
índice de la articulación bajo prueba; `clip_arm_q_target` recalcula desde la
posición real en cada paso, así que las demás tienen delta ≈ 0 y no se mueven.

### Por qué NO empezar por `teleop_hand_and_arm.py`

Sin el Quest conectado, `televuer` devuelve las poses por defecto
`CONST_LEFT_ARM_POSE` / `CONST_RIGHT_ARM_POSE` (marcadas en el código como *"For
Robot initial position"*). El lazo principal llama a `solve_ik(...)` y
`ctrl_dual_arm(...)` **sin comprobar si hay datos XR**, así que al pulsar `r`
los brazos irían a esa postura fija — desde donde estén, y con el límite de
velocidad por defecto de 30 rad/s. Es justo el arranque brusco que el README del
repositorio advierte cuando dice *"alinea tus brazos con la pose inicial del
robot"*.

### `--display-mode pass-through`

Solo cambia **lo que el visor muestra**; el seguimiento de manos sigue igual:

| display-mode | Qué ve el operador | Imagen del robot al visor |
|---|---|---|
| `immersive` | la cámara del robot ocupando todo | sí |
| `ego` | passthrough + ventana pequeña en primera persona | sí |
| `pass-through` | **el mundo real por las cámaras del Quest** | **no** |

En `pass-through`, `render_to_xr()` se ignora explícitamente (televuer lo avisa
por log). El operador ve el robot con sus propios ojos a través del visor, que
para las primeras pruebas es lo más seguro: si el brazo va a donde no debe, lo
ves directamente y no a través de una cámara con latencia.
