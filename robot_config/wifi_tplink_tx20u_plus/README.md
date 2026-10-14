# WiFi del PC2 del H1-2: TP-Link Archer TX20U Plus (AX1800)

El PC2 del H1-2 no trae WiFi propia. Se le añade un adaptador USB **TP-Link Archer
TX20U Plus** y queda en la red del router del laboratorio. Eso le da dos cosas:

- **Las Meta Quest 3 llegan al robot**, que es donde corre la teleoperación XR.
- **Internet**, para instalar paquetes.

El kernel del robot (`6.5.2-rt8`) no trae driver para este adaptador: hay que
compilarlo.

| | |
|---|---|
| Adaptador | TP-Link Archer TX20U Plus, AX1800 (chip Realtek RTL8832AU, familia 8852AU) |
| USB | `2357:013f` («TP-Link 802.11ac WLAN Adapter» en `lsusb`) |
| Driver | [`lwfinger/rtl8852au`](https://github.com/lwfinger/rtl8852au), commit `865ab0f` |
| Módulo | `8852au`, versión `v1.15.0.1-0-g487ee886.20210714` |
| Interfaz | `wlx` + la MAC del adaptador. En el robot de UTEC, `wlxc03a5522df31` |

> El driver `rtl8852bu` **no sirve** para este adaptador: se probó en el robot de
> UTEC y quedan restos (`8852bu` en dkms y `/etc/modprobe.d/8852bu.conf`), pero
> en un robot nuevo no hace falta instalarlo.

## 1. Antes

- **Adaptador enchufado a un USB del PC2.** Comprobar con `lsusb | grep 2357:013f`.
- **El PC conectado por cable a la red interna del robot**: puerto de depuración
  del lateral derecho, adaptador 8+2, con una IP fija en `192.168.123.x/24` (por
  ejemplo `.99`). Comprobar con `ssh unitree@192.168.123.164 true`.
- **Internet en el PC**: el script descarga el driver aquí, porque el robot no
  tiene Internet hasta que su WiFi funciona.

## 2. Instalar el driver

Desde el PC, en esta carpeta:

```bash
./instalar_driver_wifi.sh                         # robot por cable: unitree@192.168.123.164
./instalar_driver_wifi.sh unitree@<otra-ip>       # si el robot está en otra dirección
SOLO_COMPILAR=1 ./instalar_driver_wifi.sh         # solo compila, sin instalar nada
```

Qué hace:

1. Clona el driver en este PC (`~/.cache/h1_2_utec/rtl8852au`, commit `865ab0f`).
2. Lo copia al robot, en `~/robotics40/rtl8852au`.
3. Lo compila allí con `make`, contra el kernel en marcha.
4. Instala `8852au.ko` en `/lib/modules/<kernel>/extra/`, ejecuta `depmod`, lo
   añade a `/etc/modules-load.d/8852au.conf` para que cargue en cada arranque y
   hace `modprobe`. Este paso pide el `sudo` del robot.

Es lo mismo que se hizo a mano en el robot de UTEC el 2026-09-23.

## 3. Conectar la WiFi

En el robot (por el cable: `ssh unitree@192.168.123.164`). El nombre de la
interfaz sale de `ip -br link | grep wlx`:

```bash
sudo nmcli dev wifi connect UTEC_H1_2 password '<clave>' ifname <wlx...> name UTEC_H1_2_tplink
sudo nmcli con modify UTEC_H1_2_tplink ipv4.route-metric 500 connection.autoconnect yes
sudo nmcli con up UTEC_H1_2_tplink
ip -br addr show <wlx...>          # la IP que le ha dado el router
ping -c 2 8.8.8.8                  # Internet
```

Así está el robot de UTEC: conexión `UTEC_H1_2_tplink`, SSID `UTEC_H1_2`, IP por
**DHCP** (`192.168.0.143`, puerta de enlace `192.168.0.1`), `route-metric 500`,
conexión automática.

- **Que la IP no cambie.** Lo más sencillo es reservarla en el router para la
  MAC del adaptador (la que va en el nombre `wlx…`). Otra opción es IP fija:
  `ipv4.method manual ipv4.addresses <ip>/24 ipv4.gateway 192.168.0.1 ipv4.dns 192.168.0.1`.
  Esa IP es la que usan los scripts del PC (`ROBOT_SSH`) y la que va en el
  certificado del teleop.
- **No toques `eth0` ni `br0`.** Por `eth0` va el DDS del robot y por `br0` las
  manos. La WiFi solo añade la ruta por defecto; `eth0` no tiene puerta de
  enlace, así que no compiten.

## 4. Comprobar

```bash
lsmod | grep 8852au                 # módulo cargado
nmcli -t -f NAME,DEVICE con show --active | grep wlx
sudo reboot                         # y tras el arranque, sin hacer nada, la WiFi tiene que volver sola
```

## 5. Deshacer

```bash
sudo rm /lib/modules/$(uname -r)/extra/8852au.ko /etc/modules-load.d/8852au.conf
sudo depmod -a && sudo reboot
```

## Estado

Probado el 2026-10-14 contra el robot de UTEC con `SOLO_COMPILAR=1`: compila en
~36 s y el módulo sale con la misma versión y el mismo `vermagic` que el
instalado (`6.5.2-rt8 SMP preempt_rt mod_unload modversions`). El paso de
instalación no se repitió en ese robot, porque ya lo tenía. **Está pendiente de
probarse completo en un robot nuevo.**

**Si hay que recompilar**, el módulo depende de la versión exacta del kernel: si
el robot cambia de kernel, hay que volver a ejecutar el script.
