# Simulation Deployment: xr_teleoperate sobre unitree_sim_isaaclab

Despliegue **en simulación** de
[`xr_teleoperate`](https://github.com/unitreerobotics/xr_teleoperate) contra
[`unitree_sim_isaaclab`](https://github.com/unitreerobotics/unitree_sim_isaaclab),
con el **H1-2 de 27 DoF** y manos **Inspire**. Es el hermano del
[`README_DEPLOY.md`](README_DEPLOY.md), que cubre el despliegue **físico**.

La gracia de tenerlo montado así es que **el mismo comando de teleoperación
vale para los dos**: solo cambia el flag `--sim`.

| | robot real | simulación |
|---|---|---|
| Lanzador | `scripts/03_launch_teleop.sh` | `scripts/12_launch_teleop_sim.sh` |
| Argumentos | `--arm H1_2 --ee inspire_ftp` | `--arm H1_2 --ee inspire_ftp --sim` |
| Dominio DDS | 0 | 1 |
| Servidor de imagen | PC2 del robot (`192.168.123.164`) | el propio simulador (IP LAN de esta laptop, §3.6) |
| Manos | driver Modbus⇄DDS en el PC2 | `dds/inspire_ftp_dds.py` (parche de este repo) |

---

## 1. Máquina

| | |
|---|---|
| SO | Ubuntu 22.04.5 LTS |
| GPU | NVIDIA RTX PRO 2000 Blackwell, 16 GB · driver 580.173.02 · CUDA 13.0 |
| CPU / RAM | 20 hilos · 30 GB |
| Isaac Sim | 5.1.0 (pip, `isaacsim[all,extscache]`) |
| Isaac Lab | 2.3.2 (`/home/utec/IsaacLab`, editable) |
| Conda | miniconda3 26.7.1 en `/home/utec/miniconda3` |

> La GPU es Blackwell, así que **Isaac Sim 5.x es obligatorio**: el README de
> Unitree avisa de que las series RTX 50 no funcionan con Isaac Sim 4.5.

Dos entornos conda, uno por lado:

| Entorno | Python | Para qué |
|---|---|---|
| `unitree_sim_env` | 3.11.16 | el simulador (Isaac Sim + Isaac Lab + DDS) |
| `tv` | 3.10.21 | el cliente de teleoperación (`xr_teleoperate`) |

`unitree_sim_env` se creó **replicando el juego exacto de paquetes** del
`env_isaaclab` que ya existía y estaba verificado: `pip freeze` en el bueno,
`pip install -r` en el nuevo. Se queda con las mismas versiones exactas, sin
tocar el entorno de partida y sin la lotería de que el resolutor elija otra
cosa meses después.

> Lo primero que se intentó fue `conda create --clone env_isaaclab`, que es lo
> obvio. **No sirve a esta escala**: conda copia fichero a fichero calculando
> el sha256 de cada uno, y con 158 000 ficheros iba a ~6 ficheros/s — unas 4
> horas. Una copia normal en este mismo disco va a 162 ficheros/s, así que el
> cuello de botella es conda, no el disco.

Un detalle que se aparta de la documentación de Unitree: `doc/isaacsim5.1_install.md`
instala **torch cu126**, pero aquí va **torch 2.7.0+cu128**, que es lo que tenía
el entorno que ya funcionaba. La GPU es Blackwell (`sm_120`) y necesita CUDA
12.8 o superior; con las ruedas cu126 no hay kernels para esa arquitectura.

---

## 2. Topología en simulación

Todo ocurre en esta máquina; lo único externo es el visor.

```
   Meta Quest 3
        │  WiFi · WebXR sobre HTTPS/WSS  →  https://<ip-wifi>:8012
        ▼
   ┌──────────────────────── ESTA LAPTOP ───────────────────────────┐
   │                                                                │
   │  conda "tv"                       conda "unitree_sim_env"      │
   │  teleop_hand_and_arm.py           sim_main.py                  │
   │    --arm H1_2 --ee inspire_ftp      --robot_type h1_2          │
   │    --sim                            --enable_inspire_ftp_dds   │
   │                                     --task Isaac-PickPlace-…   │
   │   · televuer → Vuer/WebXR :8012                                │
   │   · pinocchio → IK 2×7 DoF        · Isaac Sim 5.1 + Isaac Lab  │
   │   · dex-retargeting → 6 DoF/mano  · teleimager (isaacsim)      │
   │   · teleimager.image_client         ZMQ :55555  WebRTC :60001  │
   │            │                             │                     │
   │            └──────── ZMQ 127.0.0.1 ──────┘                     │
   │                                                                │
   │            ┌───────── CycloneDDS, dominio 1 ────────┐          │
   │            │  rt/lowcmd          rt/lowstate        │          │
   │            │  rt/inspire_hand/ctrl/{l,r}            │          │
   │            │  rt/inspire_hand/state/{l,r}           │          │
   │            │  rt/reset_pose/cmd  rt/sim_state       │          │
   │            └────────────────────────────────────────┘          │
   └────────────────────────────────────────────────────────────────┘
```

**El dominio DDS es el 1**, no el 0. Lo fija `sim_main.py`
(`ChannelFactoryInitialize(1)`) y `xr_teleoperate` lo hace igual cuando se le
pasa `--sim`. El robot físico vive en el dominio 0, así que los dos pueden
coexistir en la misma red sin pisarse — pero conviene no tentar a la suerte.

---

## 3. Incidencias del procedimiento oficial

Cosas que no funcionan tal cual las cuenta la documentación de Unitree, o que
son propias de esta máquina.

### 3.1 El README de Unitree y el código no coinciden en dos flags

#### `--robot_type h12` no existe: es `h1_2`

El README de `unitree_sim_isaaclab` dice:

> Specify robot type via `--robot_type g129` or `--robot_type h12`.

Pero `dds/dds_create.py` compara contra `"h1_2"`:

```python
if args_cli.robot_type=="g129" or args_cli.robot_type=="h1_2":
```

Con `h12` **no se crea ningún objeto DDS del robot**: el simulador arranca, la
escena se ve, y el H1-2 simplemente no se mueve ni publica `rt/lowstate`. No da
ningún error. Los ejemplos comentados al final de `sim_main.py` sí usan `h1_2`.

`scripts/11_launch_sim.sh` fija `h1_2`.

#### `--replay` no existe: es `--replay_data`

El README documenta `--replay` para reproducir un dataset, pero el argumento
declarado es `--replay_data`. Cuela de milagro, porque argparse acepta prefijos
no ambiguos y no hay ningún otro `--replay*`; en cuanto Unitree añada uno,
`--replay` empezará a fallar. Mejor escribirlo entero.

### 3.2 El simulador solo hablaba DFX; nuestras manos son FTP

Las tareas del H1-2 se llaman `…-Inspire-…` y se activan con
`--enable_inspire_dds`, pero eso levanta `dds/inspire_dds.py`, que habla el
protocolo de las manos **DFX** del G1:

```
rt/inspire/cmd  ·  rt/inspire/state      unitree_go::MotorCmds_/MotorStates_
```

Las **RH56DFTP** de este H1-2 hablan otra cosa, un par de tópicos por mano con
los IDL de `inspire_sdkpy` y ángulos enteros 0..1000:

```
rt/inspire_hand/ctrl/{l,r}  ·  rt/inspire_hand/state/{l,r}
```

Es decir: con el simulador tal cual hay que teleoperar con `--ee inspire_dfx`,
distinto del `--ee inspire_ftp` del robot real. **Este repo añade el camino
FTP al simulador** para que el comando sea el mismo en los dos sitios:

- `dds/inspire_ftp_dds.py` — clase `InspireFTPDDS`, mismo interfaz y misma
  memoria compartida que `InspireDDS`, así que es un reemplazo directo: ni
  `action_provider_dds.py` ni `tasks/common_observations/inspire_state.py` se
  enteran.
- `dds/inspire_ftp_idl.py` — el IDL, copiado de `inspire_sdkpy` (ver §3.4).
- `sim_main.py` — flag `--enable_inspire_ftp_dds`, que implica
  `--enable_inspire_dds` para no tocar los tres *action providers*.
- `dds/dds_create.py` — elige una clase u otra al registrar el objeto
  `"inspire"`.
- `tasks/common_observations/inspire_state.py` — arreglo del estado barajado
  del H1-2 (§3.3), que no tiene que ver con FTP pero va en el mismo parche.

Todo junto en [`patches/unitree_sim_isaaclab_inspire_ftp.patch`](patches/unitree_sim_isaaclab_inspire_ftp.patch).

El camino DFX sigue disponible: `HAND_DDS=--enable_inspire_dds` en
`11_launch_sim.sh` y `EE=inspire_dfx` en `12_launch_teleop_sim.sh`.

### 3.3 El estado de las manos sale barajado en el H1-2

Esto no es una molestia de instalación: es un **fallo de `unitree_sim_isaaclab`
que rompe la realimentación de las manos en el H1-2**, y afecta igual al camino
DFX de Unitree que al FTP de este repo, porque los dos leen de
`tasks/common_observations/inspire_state.py`. Ahí los índices de articulación
estaban escritos a mano:

```python
inspire_joint_indices = [36, 37, 35, 34, 48, 38, 31, 32, 30, 29, 43, 33]
```

Son los del **G1 de 29 DoF**. El H1-2 tiene otro orden de articulaciones, así
que cada ranura de estado lee el dedo equivocado. El camino de **comando** no
tiene el problema: `action_provider_dds.py` resuelve por nombre
(`inspire_hand_joint_mapping`). Resultado: la teleoperación **mandaba bien y
leía mal**, en silencio.

Medido moviendo un DOF cada vez contra el simulador (`ctrl/… dofN` → dónde
aparecía en `state/…`):

| comando | se veía en | debería |
|---|---|---|
| `ctrl/l` dof3 (índice izq.) | `state/r` ranura 1 | `state/l` ranura 3 |
| `ctrl/l` dof2 (medio izq.) | `state/r` ranura 5 | `state/l` ranura 2 |
| `ctrl/r` dof3 (índice der.) | `state/l` ranura 1 | `state/r` ranura 3 |
| `ctrl/l` dof0 (meñique izq.) | **en ninguna** | `state/l` ranura 0 |
| `ctrl/l` dof4 (pulgar izq.) | **en ninguna** | `state/l` ranura 4 |

El arreglo, incluido en el parche, es resolver los índices **por nombre**, igual
que hace el camino de comando, con la lista fija como último recurso:

```python
joint_names = list(env.scene["robot"].data.joint_names)
inspire_joint_indices = [joint_names.index(n) for n in get_robot_girl_joint_names()]
```

Tras el arreglo, los 12 DOF salen en correspondencia exacta (§7).

### 3.4 `inspire_sdkpy` no se puede meter en el entorno de Isaac Sim

Lo natural sería que el puente importara el IDL de `inspire_sdkpy`. Pero su
`__init__.py` hace:

```python
from .qt_tabs import ImageTab, MainWindow, CurveTab
```

o sea que **importar cualquier submódulo arrastra PyQt5, pyqtgraph y colorcet**,
más pymodbus y pyserial para hablar con la mano física. Nada de eso pinta en el
entorno de Isaac Sim, que además trae su propio stack gráfico.

El IDL está copiado verbatim en `dds/inspire_ftp_idl.py`. Para DDS los dos son
**el mismo tipo**: la compatibilidad va por el `typename`
(`inspire.inspire_hand_ctrl`) y la estructura, no por la clase Python.
`scripts/14_test_inspire_ftp_bridge.py` lo comprueba publicando con el IDL
copiado y leyendo con el de `inspire_sdkpy`.

### 3.5 El `PYTHONPATH` global de ROS/robotpkg rompe pinocchio en los dos entornos

El `~/.bashrc` de esta laptop hace `source /opt/ros/humble/setup.bash` y añade:

```bash
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
```

Las dos van **antes** que lo del entorno conda, así que se cuelan en cualquier
env activado. Rompe de tres maneras distintas:

| Variable | Síntoma |
|---|---|
| `PYTHONPATH` | en `tv` (py3.10) `import pinocchio` carga la **4.1.0 de robotpkg**, no la 3.1.0 de conda que fija el README de xr_teleoperate. En `unitree_sim_env` (py3.11) ni carga: los `.so` son de 3.10 — y `sim_main.py` empieza con `import pinocchio`. |
| `LD_LIBRARY_PATH` | aunque el módulo Python salga del entorno, el enlazador coge `libpinocchio`/`libboost` de `/opt/openrobots`. Mezclar binding de una versión con libs de otra da, al construir el modelo: `RuntimeError: class version St6vectorIS_ImSaImEESaIS1_EE` |
| `PYTHONPATH`, otra vez | **pip también lo mira**. `pip freeze` en cualquier entorno de esta máquina lista 174 paquetes de ROS que no están en el entorno (`ament-*`, `rcl*`, `*-msgs`…), y `pip install` los da por *already satisfied* y no los instala. Replicar un entorno con `pip freeze` \| `pip install -r` sin aislar produce un entorno incompleto que aparenta estar bien |

`scripts/09_isolate_conda_env.sh` lo arregla con hooks de `activate.d` /
`deactivate.d` **dentro de cada entorno**: se guardan y se limpian las dos
variables al activar, y se devuelven al desactivar. No se toca `~/.bashrc`, así
que ROS sigue funcionando fuera de estos entornos. Los instaladores
(`01_install_host.sh`, `10_install_sim.sh`) lo ejecutan solos.

> Los hooks solo actúan con `conda activate`. Si se llama al intérprete —o a
> pip— por su ruta absoluta (`…/envs/tv/bin/python`), las variables globales
> siguen puestas: usar `env -u PYTHONPATH`. Los scripts de diagnóstico y de
> instalación ya lo hacen.

### 3.6 `--img-server-ip 127.0.0.1` deja al visor sin vídeo

Lo intuitivo, con el simulador corriendo en la misma máquina, es apuntar el
cliente de imagen a `127.0.0.1`. **No funciona con el Quest puesto.**

La configuración de cámaras del simulador trae `enable_webrtc: true`, y en ese
caso `teleop_hand_and_arm.py` construye:

```python
webrtc_url = f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer"
```

que televuer entrega **tal cual** como `src` de un `WebRTCStereoVideoPlane`. Ese
componente lo renderiza el **navegador del visor**, así que con `127.0.0.1` el
Quest pide el vídeo a *su propio* localhost. No hay error: simplemente no llega
imagen, y el resto (seguimiento de manos, control) sigue funcionando, lo que
despista bastante.

Hay que pasar la **IP de esta laptop en la red del visor**. Sirve igual para el
ZMQ del `ImageClient`, que es local de todas formas.
`12_launch_teleop_sim.sh` la deduce de la ruta por defecto:

```bash
IMG_SERVER_IP="${IMG_SERVER_IP:-$(ip -4 route get 1.1.1.1 | sed -n 's/.* src \([0-9.]*\).*/\1/p')}"
```

El certificado generado por `02_gen_certs.sh` ya cubre todas las IP vivas de la
máquina, así que el HTTPS del `:60001` valida igual que el del `:8012`.

> En el despliegue físico este problema no aparece porque
> `videohub_image_bridge.py` declara `enable_webrtc: False`: allí la imagen va
> por ZMQ y la empuja el propio Python dentro de Vuer, así que la IP del
> servidor solo la usa el proceso local. La otra salida en simulación sería
> poner `enable_webrtc: false` en `unitree_sim_isaaclab/teleimager/cam_config_server.yaml`
> y volver al camino ZMQ, pero WebRTC da bastante menos latencia en el visor.

### 3.7 La caché de pinocchio no sobrevive a un cambio de versión

`robot_arm_ik.py` guarda el modelo con `pickle` en
`xr_teleoperate/teleop/h1_2_model_cache.pkl`. Pinocchio serializa con
Boost.Serialization y **una caché escrita por otra versión no se puede leer**:
sale el mismo `RuntimeError: class version …` de §3.5, esta vez en
`pickle.load`, lo que despista mucho.

Si se cambia de versión de pinocchio (o se arregla el `PYTHONPATH` después de
haber ejecutado algo), hay que borrar la caché:

```bash
rm -f xr_teleoperate/teleop/*_model_cache.pkl
```

`00_check_host.sh` da la pista cuando detecta ese error.

### 3.8 El resto que ya estaba documentado

Las cuatro incidencias del lado laptop del despliegue físico
([`README_DEPLOY.md` §4](README_DEPLOY.md)) siguen aplicando, porque el entorno
`tv` es el mismo: `params-proto==2.13.2` para que no reviente `vuer`,
`dex-retargeting` con `--no-deps`, `inspire_sdkpy` fuera de PyPI, y el parche
`patches/inspire_sdkpy_uint16.patch`.

Además, `unitree_sdk2py` fija `cyclonedds==0.10.2`, que no publica rueda: pip la
compila y necesita `CYCLONEDDS_HOME`. Los dos instaladores compilan CycloneDDS
0.10.x en `cyclonedds/install` y exportan la variable.

---

## 4. Qué simula el puente de manos y qué no

`InspireFTPDDS` publica un `inspire_hand_state` por mano. No todo tiene
equivalente en el simulador:

| Campo | En simulación |
|---|---|
| `angle_act` | **real**: el ángulo articular del simulador escalado a 0..1000 (1000 = abierto), igual que la mano física |
| `pos_act` | copia de `angle_act`. La mano real reporta ahí la posición cruda del actuador, que el simulador no modela |
| `status` | deducido comparando el ángulo actual con el último comandado: 0 abriendo, 1 cerrando, 2 posición alcanzada |
| `force_act` | **siempre 0**. El simulador no modela el sensor de fuerza de la RH56; publicar ahí el par articular sería dar gramos-fuerza falsos |
| `current`, `err`, `temperature` | siempre 0 |

Del comando solo se atiende el **modo ángulo** (`mode & 0b0001`), que es lo que
manda `Inspire_Controller_FTP`. `pos_set`, `force_set` y `speed_set` se ignoran:
el simulador aplica la posición objetivo directamente, sin lazo de fuerza ni de
velocidad. Sí se respeta `angle_set[i] == -1`, el **no-op** del protocolo RH56
("deja este DOF como está"), que es justo lo que necesita cualquier diagnóstico
que mueva un dedo a la vez.

Orden de los 12 DOF, heredado del simulador
(`tasks/common_observations/inspire_state.py`):

```
índices  0..5   mano DERECHA    →  rt/inspire_hand/{ctrl,state}/r
índices  6..11  mano IZQUIERDA  →  rt/inspire_hand/{ctrl,state}/l
```

y dentro de cada mano, el orden de la RH56:
`[meñique, anular, medio, índice, pulgar-flexión, pulgar-rotación]`.

> La inversión de lateralidad del robot físico ([`README_DEPLOY.md` §2](README_DEPLOY.md))
> **no aplica aquí**: era un cruce de cables/IP en el robot, y lo corrige el
> driver del PC2. A nivel de tópicos DDS, `/l` siempre es la izquierda.

---

## 5. Instalación desde cero

```bash
git clone https://github.com/smorales2405/h1_2_utec.git
cd h1_2_utec/h1_2_teleoperation

# --- repos upstream, junto a este directorio ---
git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git
cd unitree_sim_isaaclab && git submodule update --init --depth 1 && cd ..

git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd xr_teleoperate && git submodule update --init --depth 1 && cd ..

git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
git clone https://github.com/NaCl-1374/inspire_hand_ws.git

# --- parches ---
cd inspire_hand_ws && git apply ../patches/inspire_sdkpy_uint16.patch && cd ..
cd unitree_sim_isaaclab && git apply ../patches/unitree_sim_isaaclab_inspire_ftp.patch && cd ..

# --- assets USD del simulador (~1.7 GB, HuggingFace + git-lfs) ---
sudo apt install -y git-lfs unzip cmake build-essential openssl
cd unitree_sim_isaaclab && bash fetch_assets.sh && cd ..

# --- entorno del simulador ---
# Si ya hay un entorno con Isaac Sim 5.1 + Isaac Lab (aqui, "env_isaaclab"), la
# forma fiable de replicarlo es congelar su juego de paquetes y reinstalarlo:
# se queda con las versiones exactas ya verificadas. Cuenta con volver a bajar
# los ~13 GB de ruedas de Isaac Sim: pip cachea los paquetes normales, pero no
# las ruedas gigantes de isaacsim (isaacsim_extscache_kit sola son 3021 MB).
# NO uses `conda create --clone`: tarda horas (ver §1).
conda create -y -n unitree_sim_env python=3.11.16
PIPSRC=/home/utec/miniconda3/envs/env_isaaclab/bin/pip
PIPDST=/home/utec/miniconda3/envs/unitree_sim_env/bin/pip
# `env -u PYTHONPATH` en los dos lados y `--no-deps` en la instalacion: ver §3.5
env -u PYTHONPATH $PIPSRC freeze | grep -v '^-e git+.*IsaacLab' > /tmp/sim_env_reqs.txt
env -u PYTHONPATH $PIPDST install --no-deps -r /tmp/sim_env_reqs.txt \
    --extra-index-url https://pypi.nvidia.com \
    --extra-index-url https://download.pytorch.org/whl/cu128
for pkg in isaaclab isaaclab_assets isaaclab_contrib isaaclab_mimic isaaclab_rl isaaclab_tasks; do
    $PIPDST install -e "/home/utec/IsaacLab/source/$pkg" --no-deps
done

bash scripts/10_install_sim.sh

# --- entorno del cliente de teleoperación ---
conda create -y -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
bash scripts/01_install_host.sh

# --- certificados TLS para WebXR y WebRTC ---
bash scripts/02_gen_certs.sh

# --- diagnóstico ---
bash scripts/13_check_sim.sh
bash scripts/00_check_host.sh     # su §6 (red) solo aplica al robot físico
```

Si no existiera ningún entorno con Isaac Sim, sirve el `auto_setup_env.sh` de
Unitree (`bash auto_setup_env.sh 5.1 unitree_sim_env`) o los pasos manuales de
[`doc/isaacsim5.1_install.md`](https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/main/doc/isaacsim5.1_install.md)
— cambiando `cu126` por `cu128`, por lo de la GPU Blackwell (§1).

---

## 6. Puesta en marcha

**Terminal 1 — simulador**

```bash
./scripts/11_launch_sim.sh
```

```bash
TASK=Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint  ./scripts/11_launch_sim.sh
HAND_DDS=--enable_inspire_dds                      ./scripts/11_launch_sim.sh   # camino DFX
EXTRA=--no_render                                  ./scripts/11_launch_sim.sh   # WebRTC en vez de ventana
```

Tareas disponibles para el H1-2:

| Tarea | Escena |
|---|---|
| `Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint` | coger y colocar un cilindro |
| `Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint` | coger y colocar un bloque rojo |
| `Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint` | apilar tres bloques |

La primera arrancada tarda: Isaac Sim compila shaders y carga los USD. Cuando
salga

```
controller started, start main loop...
```

hay que **hacer un clic dentro de la ventana** para activarla. Si la vista sale
rara: `PerspectiveCamera → Cameras → PerspectiveCamera`.

**Terminal 2 — teleoperación**

```bash
./scripts/12_launch_teleop_sim.sh
```

```bash
EXTRA=--record        ./scripts/12_launch_teleop_sim.sh   # grabar episodios
EE=inspire_dfx        ./scripts/12_launch_teleop_sim.sh   # si el sim va en DFX
INPUT_MODE=controller ./scripts/12_launch_teleop_sim.sh   # mandos en vez de manos
```

**En el Quest 3**

1. Misma WiFi que la laptop; activar seguimiento de manos en Ajustes.
2. Navegador → `https://<ip-wifi-de-la-laptop>:8012` → *Advanced* →
   *Proceed to … (unsafe)*. Solo la primera vez.
3. Botón **Virtual Reality** y aceptar permisos.

**Secuencia de control**: `r` para que el robot empiece a seguir, `s` para
iniciar/guardar grabación (con `--record`), `q` para salir.

**Reproducir un dataset grabado** (mismo formato que la teleoperación real):

```bash
EXTRA="--replay_data --file_path $PWD/xr_teleoperate/teleop/utils/data" ./scripts/11_launch_sim.sh
```

---

## 7. Verificaciones ejecutadas

Todo lo de abajo está medido en esta máquina, con el simulador corriendo la
tarea `Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint` y el puente FTP.

### Sin arrancar Isaac Sim

| Prueba | Resultado |
|---|---|
| `scripts/13_check_sim.sh` | ✔ los 10 apartados en verde |
| `scripts/00_check_host.sh` | ✔ 17 paquetes y los 6 imports del entorno `tv` (§6 es del robot físico) |
| `scripts/14_test_inspire_ftp_bridge.py` | ✔ 12/12: escala 0..1000, lateralidad, no-op `-1`, y el IDL copiado leído por `inspire_sdkpy` |

### Con el simulador arrancado

Sonda de solo lectura en el dominio DDS 1, desde el entorno `tv`:

| Tópico | Frecuencia |
|---|---|
| `rt/lowstate` | 93 Hz |
| `rt/inspire_hand/state/l` | 93 Hz |
| `rt/inspire_hand/state/r` | 93 Hz |
| `rt/sim_state` | 93 Hz |

`angle_act` en reposo llega como `[1000, 1000, 1000, 1000, 999, 929]`: manos
abiertas, y el 929 del pulgar-rotación es exactamente
`(1.3 − 0) / 1.4 × 1000`, la normalización del rango `[−0.1, 1.3]` de ese DOF.

**Comando extremo a extremo**, que es la ruta exacta de la teleoperación: la
laptop publica en `rt/inspire_hand/ctrl/l` con `mode = 0b0001` y `-1` en los DOF
que no se tocan, y el anular izquierdo del robot simulado responde:

| comandado | 0 | 500 | 1000 |
|---|---|---|---|
| `angle_act` leído | 1 | 500 | 1000 |

**Barrido de los 12 DOF**, uno cada vez, antes y después del arreglo de §3.3:

| | antes | después |
|---|---|---|
| DOF que se leían en la ranura correcta | 1 de 12 | **12 de 12** |
| DOF que no aparecían en ningún sitio | 2 | 0 |

### Rendimiento observado

| | |
|---|---|
| Bucle del simulador | ~24 Hz con `--enable_cameras` (3 cámaras a 480×640) |
| Publicación DDS | 93 Hz |
| GPU | 3,5 GB de 16 GB, 40-50 % de uso |
| RAM | ~16 GB de 30 GB |
| Arranque en frío | ~5 min (compilación de shaders y carga de USD) |

---

## 8. Scripts

Los del despliegue físico están en [`README_DEPLOY.md` §6](README_DEPLOY.md).
Los de simulación:

| Script | Qué hace |
|---|---|
| `09_isolate_conda_env.sh` | aísla los entornos conda del ROS/robotpkg global (§3.5) |
| `10_install_sim.sh` | instala las dependencias del simulador en `unitree_sim_env` |
| `11_launch_sim.sh` | lanza `unitree_sim_isaaclab` con el H1-2 + manos Inspire |
| `12_launch_teleop_sim.sh` | lanza `xr_teleoperate` en modo `--sim` contra el simulador |
| `13_check_sim.sh` | diagnóstico del lado simulación |
| `14_test_inspire_ftp_bridge.py` | prueba del puente FTP sin arrancar Isaac Sim |

Todos deducen su ruta base de dónde están, así que el repo se puede clonar
donde sea. Se puede forzar con `ROOT=…`.
