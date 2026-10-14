# scripts_robot — lo que corre en el robot para la teleoperación XR

`scripts/teleop_xr.sh`, lanzado desde el PC, ejecuta la teleoperación **en el PC2
del H1-2**. Esta carpeta es la copia de lo que hace falta en el robot para eso.

- Son copias **exactas** de los archivos del H1-2 de UTEC, comparadas por checksum
  con el robot el 2026-10-14.
- Sirven para tener versionado lo que corre en el robot y para montar la misma
  teleoperación en **otro H1-2 con manos Inspire RH56DFTP** (§4).
- Los valores propios de cada robot (IP, interfaz WiFi, cámara, manos) **no están
  parametrizados**: §5 dice qué línea cambiar y cómo averiguar el valor nuevo.

Cómo se usa la teleoperación, una vez instalada: [`../README.md`](../README.md).

---

## 1. Qué hay y dónde va

`home_unitree/` reproduce el `$HOME` del robot (`/home/unitree`): todo cae en su
sitio con un solo `rsync` (§4, paso 2).

| En esta carpeta | En el robot | Qué es |
|---|---|---|
| `home_unitree/robotics40/teleop_xr.sh` | `~/robotics40/` | punto de entrada: carga `comun.sh` y lanza `meta/arrancar.sh` (o `meta/parar.sh` con `parar`) |
| `home_unitree/robotics40/comun.sh` | `~/robotics40/` | rutas, intérpretes, IP de las manos; quita el `CYCLONEDDS_URI` del `.bashrc`, que rompe el DDS. Fuera del robot, reenvía por SSH |
| `home_unitree/robotics40/meta/` | `~/robotics40/meta/` | `arrancar.sh`, `parar.sh`, `lanzar_teleop.py` (con los cambios del 2026-10-07: `q` abre las manos, corte del SSH = `q`), `enviar.py`, `prueba_gafas.py`, `instalar.sh` (crea `env_tv`), `README.md` y `certs/san.cnf` |
| `home_unitree/inspire_ftp_dual_driver.py` | `~/` | driver de las manos, Modbus TCP ⇄ DDS (`rt/inspire_hand/*`), que lanza `arrancar.sh` |
| `home_unitree/pc2_offline/src/unitree_sdk2_python` | `~/pc2_offline/src/` | `unitree_sdk2py` 1.0.1, la versión exacta que funciona |
| `home_unitree/pc2_offline/src/inspire_hand_sdk` | `~/pc2_offline/src/` | `inspire_sdkpy` 1.0.0, **ya con** [`patches/inspire_sdkpy_uint16.patch`](../patches/inspire_sdkpy_uint16.patch) |
| `home_unitree/robotics40/estado.sh`, `lectores_h1_2/` | `~/robotics40/` | estado del robot y `CheckMode` (¿Debug?): verificación, no hace falta para el teleop |
| `home_unitree/robotics40/manos/` | `~/robotics40/manos/` | `manos.sh leer`, `borrar-error`, `puno`/`abrir`… para diagnosticar las manos |
| `xr_teleoperate_cambios/teleop/teleimager/cam_config_server.yaml` | `~/robotics40/meta/xr_teleoperate/teleop/teleimager/` | cámara: solo la D435i de la cabeza, por ZMQ, sin WebRTC. Se copia **después** de clonar `xr_teleoperate` |

Los dos SDK de `pc2_offline/src/` se instalan en modo editable en `env_tv` y en
`teleop_venv`: **tienen que quedarse en esa ruta**.

**No está aquí**, y cómo se consigue:

| Qué | Tamaño | Cómo |
|---|---|---|
| `meta/xr_teleoperate` (Unitree, commit `817fb00` + 3 submódulos) | 404 MB | `git clone` (§4, paso 3) |
| `meta/env_tv` (conda, Python 3.10, pinocchio 3.1.0) | 2,9 GB | conda + `meta/instalar.sh` (§4, paso 4) |
| `~/teleop_venv` (Python 3.10 del sistema) | 0,5 GB | venv + pip (§4, paso 5) |
| `cert.pem` / `key.pem` (en `meta/certs/` y `~/.config/xr_teleoperate/`) | — | se generan en cada robot (§4, paso 7). **Las claves privadas no van a git** |
| Driver de la WiFi TP-Link | — | [`robot_config/wifi_tplink_tx20u_plus`](../../robot_config/wifi_tplink_tx20u_plus/README.md) |
| `pc2_offline/wheels`, `install_pc2.sh` | 234 MB | la instalación sin Internet de septiembre. Ya no hace falta: el robot tiene Internet por la WiFi |

## 2. Qué se ejecuta en el robot

```
~/robotics40/teleop_xr.sh                       (llega por: ssh -t unitree@<ip> "cd ~/robotics40 && ./teleop_xr.sh")
  └─ comun.sh                                   PYTHONNOUSERSITE=1, sin CYCLONEDDS_URI
  └─ meta/arrancar.sh
       1. sudo env_tv/bin/python -m teleimager.image_server --rs     D435i → ZMQ :55555     logs/image_server.log
       2. teleop_venv/bin/python ~/inspire_ftp_dual_driver.py --network eth0 --no-touch      logs/manos.log
       3. env_tv/bin/python meta/lanzar_teleop.py --arm=H1_2 --ee=inspire_ftp --input-mode=hand --headless
                --network-interface=eth0 --img-server-ip=127.0.0.1      → teleop_hand_and_arm.py (Vuer :8012)
                                                                          logs/teleop.log (el terminal ve un tail)
```

## 3. Requisitos del robot nuevo

- **H1-2 con manos Inspire RH56DFTP por Modbus TCP**, colgando de `br0`
  (`192.168.124.x`). PC2 con el mismo sistema: Ubuntu 22.04, kernel `6.5.2-rt8`,
  usuario `unitree`. Las rutas de `env_tv` y de los SDK editables dependen de
  `/home/unitree`.
- **La D435i en un USB 3 del PC2.** En el robot de UTEC está ahí; en otro puede
  estar en el PC1 y habría que moverla. Comprobar con `lsusb | grep 8086:0b3a`.
- **El adaptador WiFi TP-Link Archer TX20U Plus** en un USB del PC2.
- **conda** en `~/anaconda3`: en el robot de UTEC ya estaba, desde 2024-11. Si no
  hay, instalar Miniconda y usar su `conda` en el paso 4.
- **Un PC con cable** a la red interna del robot (puerto de depuración, IP fija
  `192.168.123.x/24`) para los primeros pasos, hasta que funcione la WiFi.

## 4. Instalación en un H1-2 nuevo

Desde la raíz del repositorio en el PC. `<IP>` es la IP WiFi que tendrá el robot
nuevo; hasta el paso 1, se entra por el cable (`192.168.123.164`).

**Paso 0 — acceso por clave**

```bash
ssh-copy-id unitree@192.168.123.164
ssh unitree@192.168.123.164 'hostname; uname -r; whoami; ip -br addr show eth0 br0'
```

Tiene que dar el kernel `6.5.2-rt8`, el usuario `unitree`, `eth0` con
`192.168.123.164` y `br0` con `192.168.124.164`. Apunta el `hostname` (§5).

**Paso 1 — WiFi.** Seguir
[`robot_config/wifi_tplink_tx20u_plus/README.md`](../../robot_config/wifi_tplink_tx20u_plus/README.md):
driver y conexión a `UTEC_H1_2`. Al terminar, el robot tiene `<IP>` e Internet, y
`ssh-copy-id unitree@<IP>`.

**Paso 2 — copiar los archivos**

```bash
rsync -a h1_2_teleoperation/scripts_robot/home_unitree/ unitree@<IP>:~/
```

En un robot que ya tenga `~/robotics40`, esto **sobrescribe** los archivos con el
mismo nombre.

**Paso 3 — `xr_teleoperate` de Unitree, en el commit que funciona**

```bash
ssh unitree@<IP> 'cd ~/robotics40/meta && git clone https://github.com/unitreerobotics/xr_teleoperate.git &&
    cd xr_teleoperate && git checkout 817fb00 && git submodule update --init --recursive &&
    cp teleop/teleimager/cam_config_server.yaml teleop/teleimager/cam_config_server.yaml.orig'
scp h1_2_teleoperation/scripts_robot/xr_teleoperate_cambios/teleop/teleimager/cam_config_server.yaml \
    unitree@<IP>:robotics40/meta/xr_teleoperate/teleop/teleimager/
```

Los submódulos quedan en `teleimager 57cf2a4`, `televuer 766de45` y
`dex-retargeting d7753d3`, como en el robot de UTEC.

**Paso 4 — entorno `env_tv`** (en el robot; tarda y descarga unos 3 GB)

```bash
~/anaconda3/bin/conda create -y -p ~/robotics40/meta/env_tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 \
    -c conda-forge --override-channels
bash ~/robotics40/meta/instalar.sh 2>&1 | tee ~/robotics40/meta/instalar.log
```

Tiene que terminar con `OK pinocchio 3.1.0 numpy 1.26.4 torch 2.3.0+cpu` y
`### LISTO`. Es el mismo comando (sacado del historial de conda) y el mismo
`instalar.sh` con los que se creó el entorno del robot de UTEC el 2026-09-25.

**Paso 5 — entorno `teleop_venv`** (en el robot; para el driver de las manos y `manos.sh`)

```bash
export PYTHONNOUSERSITE=1
python3.10 -m venv --without-pip ~/teleop_venv        # el PC2 no trae python3.10-venv: pip se instala aparte
curl -sS https://bootstrap.pypa.io/get-pip.py | ~/teleop_venv/bin/python
P="$HOME/teleop_venv/bin/python -m pip"
$P install setuptools wheel
$P install "cyclonedds==0.10.2" "numpy==1.26.4" opencv-python "pymodbus==3.6.9" pyserial "PyQt5==5.15.11" pyqtgraph colorcet
$P install --no-build-isolation --no-deps -e ~/pc2_offline/src/unitree_sdk2_python
$P install --no-build-isolation --no-deps -e ~/pc2_offline/src/inspire_hand_sdk
~/teleop_venv/bin/python ~/inspire_ftp_dual_driver.py --help | head -2     # tiene que salir el "usage:"
```

**Paso 6 — valores propios del robot.** Revisar y cambiar todo lo de §5 **antes**
de lanzar nada.

**Paso 7 — certificado HTTPS** para las gafas, con la IP nueva (en el robot):

```bash
cd ~/robotics40/meta/certs
sed -i 's/^IP.2=.*/IP.2=<IP>/' san.cnf
openssl req -x509 -newkey rsa:2048 -nodes -days 3650 -keyout key.pem -out cert.pem -config san.cnf
chmod 600 key.pem
mkdir -p ~/.config/xr_teleoperate && cp cert.pem key.pem ~/.config/xr_teleoperate/
openssl x509 -in cert.pem -noout -ext subjectAltName      # tiene que aparecer <IP>
```

`televuer` lee el de `~/.config/xr_teleoperate/`.

**Paso 8 (opcional) — `estado.sh`.** Sus lectores C++ se compilan con
`~/robotics40/lectores_h1_2/compilar.sh`, que necesita el `unitree_sdk2` de C++ en
`/opt/unitree_robotics` (en el robot de UTEC venía instalado) y `cmake`. Sin ellos,
`estado.sh` solo da el `CheckMode`, que es lo importante. Mira también `PY_SDK`
en §5.

**Paso 9 — en el PC**

```bash
cd h1_2_teleoperation/scripts
ROBOT_SSH=unitree@<IP> ./robot.sh comprobar
ROBOT_SSH=unitree@<IP> ./teleop_xr.sh
```

O cambia la IP por defecto en `scripts/comun.sh` (línea 11) y `scripts/robot.sh`
(línea 12). Ojo: las gafas usan `https://<IP>:8012/?ws=wss://<IP>:8012`.

## 5. Valores propios de cada robot

Rutas relativas a `home_unitree/`, es decir, al `$HOME` del robot.

| Qué | Dónde | En el robot de UTEC | Cómo averiguarlo en el robot nuevo |
|---|---|---|---|
| **IP WiFi del robot** | `robotics40/meta/certs/san.cnf:15` (paso 7) · `robotics40/comun.sh:11` · en el PC, `scripts/comun.sh:11` y `scripts/robot.sh:12`, o la variable `ROBOT_SSH` | `192.168.0.143` | `ip -br addr show <wlx…>`; mejor reservarla en el router |
| **Hostname del PC2** | `robotics40/comun.sh:12` (en el robot; en el PC no hace falta cambiarlo) | `unitree-h1-2-pc4` | `hostname`. Si no coincide, `comun.sh` cree que no está en el robot y **se reenvía por SSH a sí mismo** |
| **Interfaz WiFi** | `robotics40/meta/arrancar.sh:22` | `wlxc03a5522df31` | `ip -br link \| grep wlx`. Solo sirve para imprimir la URL de las gafas |
| **Número de serie de la D435i** | `xr_teleoperate_cambios/…/cam_config_server.yaml:19` | `"134322071879"` | el comando de abajo |
| **IP de cada mano** | `inspire_ftp_dual_driver.py:60-61` (y su copia en `robotics40/manos/h1_2_utec/`) · `robotics40/comun.sh:31-32` · `robotics40/manos/mano.py:15` · `manos/prueba_manos.py:20` · `manos/prueba_manos_pulgar.py:20` | izquierda `.211`, derecha `.210` (al revés que la documentación de Unitree) | ver abajo |
| **Intérprete `PY_SDK`** (solo `estado.sh` y otras herramientas) | `robotics40/comun.sh:28` | `~/unitree_sdk2_python/.venv/bin/python` | si no existe, pon `~/teleop_venv/bin/python`: tiene `MotionSwitcherClient` y `LocoClient` del H1 (comprobado) |
| **`sudo` sin contraseña** | — | el usuario `unitree` tiene `NOPASSWD` | si no lo tiene, `arrancar.sh` y `parar.sh` piden la contraseña en el terminal, y funciona igual |

**Número de serie de la D435i** (en el robot; solo enumera, no abre la cámara):

```bash
sudo env PYTHONNOUSERSITE=1 ~/robotics40/meta/env_tv/bin/python -c "import pyrealsense2 as rs; \
print([(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.usb_type_descriptor)) \
for d in rs.context().query_devices()])"
```

Tiene que dar `RealSense D435I`, su número de serie y USB `3.x`. El número que da
`udevadm` **no** es este. A 30 fps la cámara necesita USB 3.

**IP y lateralidad de las manos:** con `ping -c 1 192.168.124.210` y `.211` se ve
que responden, pero **no cuál es cuál**: las dos dicen `HAND_ID 1`, y solo se sabe
mirando. Con el robot **colgado** y nadie cerca, en el robot:

```bash
~/robotics40/manos/manos.sh leer            # solo lee; tiene que contestar las dos
~/robotics40/manos/manos.sh puno izq        # ¿se cierra la IZQUIERDA del robot?
~/robotics40/manos/manos.sh abrir izq
```

Si se cierra la derecha, intercambia las dos IP en todos los sitios de la tabla.
Si no, la teleoperación sale espejada.

## 6. Primeras pruebas en el robot nuevo

De menos a más. Las tres primeras no mueven nada:

1. **El teleop arranca** (en el robot):
   `cd ~ && PYTHONNOUSERSITE=1 ~/robotics40/meta/env_tv/bin/python ~/robotics40/meta/lanzar_teleop.py --help`.
   Tiene que imprimir la ayuda de `teleop_hand_and_arm.py`.
2. **Las manos se leen**: `~/robotics40/manos/manos.sh leer`.
3. **Las gafas, sin DDS**: arranca solo el servidor de imagen (la línea `sudo
   bash -c …` de `meta/arrancar.sh`) y después
   `cd ~/robotics40/meta && PYTHONNOUSERSITE=1 env_tv/bin/python prueba_gafas.py`.
   Abre `https://<IP>:8012/?ws=wss://<IP>:8012` en las Quest. Para todo con
   `~/robotics40/teleop_xr.sh parar`.
4. **Desde el PC**: `ROBOT_SSH=unitree@<IP> ./robot.sh comprobar`.
5. **El teleop completo**, con el robot colgado y en Debug, siguiendo
   [`../README.md`](../README.md).

## 7. Estado de esta guía

**Probado** el 2026-10-14 sobre el robot de UTEC, sin tocar lo instalado:

- `teleop_venv` desde cero por Internet, en una carpeta temporal (paso 5): los
  imports y el `--help` del driver funcionan;
- la generación del certificado (paso 7);
- el comando del número de serie de la D435i;
- la compilación del driver WiFi.

**Sin reprobar:**

- **`env_tv`**: son exactamente el comando y el `instalar.sh` que lo crearon, pero
  los paquetes sin versión fija de PyPI pueden haber cambiado desde entonces.
- **La instalación completa en un robot nuevo**: no se ha hecho todavía.

## 8. Actualizar esta copia desde el robot

Si cambian los archivos del robot (desde la raíz del repositorio, con
`R=unitree@192.168.0.143`):

```bash
D=h1_2_teleoperation/scripts_robot
rsync -a $R:robotics40/{teleop_xr.sh,comun.sh,estado.sh} $D/home_unitree/robotics40/
rsync -a $R:robotics40/meta/{arrancar.sh,parar.sh,lanzar_teleop.py,enviar.py,prueba_gafas.py,instalar.sh,README.md} $D/home_unitree/robotics40/meta/
rsync -a $R:robotics40/meta/certs/san.cnf $D/home_unitree/robotics40/meta/certs/
rsync -a --exclude=build --exclude=__pycache__ $R:robotics40/lectores_h1_2 $D/home_unitree/robotics40/
rsync -a --exclude=experimental --exclude=__pycache__ $R:robotics40/manos $D/home_unitree/robotics40/
rsync -a $R:inspire_ftp_dual_driver.py $D/home_unitree/
rsync -a --exclude='*.egg-info' --exclude=__pycache__ $R:pc2_offline/src/{unitree_sdk2_python,inspire_hand_sdk} $D/home_unitree/pc2_offline/src/
rsync -a $R:robotics40/meta/xr_teleoperate/teleop/teleimager/cam_config_server.yaml $D/xr_teleoperate_cambios/teleop/teleimager/
```

Con `rsync -anc -i` en vez de `-a` se ve qué ha cambiado sin copiar nada.
