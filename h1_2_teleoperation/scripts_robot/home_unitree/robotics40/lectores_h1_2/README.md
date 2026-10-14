# lectores_h1_2/ — herramientas de lectura (y alguna prueba) del H1-2

Casi todo es C++ contra el `unitree_sdk2` de `/opt/unitree_robotics`, hecho en la sesión 2 (2026-09-21) para
saber qué pasa en el robot **sin poder moverlo**.

## Lanzador: `lectores.sh` (y `../estado.sh` para un vistazo rápido)

| Orden | Herramienta | Mueve | Qué hace |
|---|---|---|---|
| `./lectores.sh lowstate [s]` | `lector_lowstate` | no | `rt/lowstate` + `rt/lf/bmsstate`: Hz, modos, temperaturas, fallos |
| `./lectores.sh postura` | `lector_postura` | no | las 27 juntas y **quién publica en `rt/lowcmd`** (¿Debug o no?) |
| `./lectores.sh fsm` | `lee_fsm` | no | `GetFsmId` / `GetFsmMode` / `GetBalanceMode` (cuadrado y caja lo usan) |
| `./lectores.sh modo` | `check_mode.py` | no | `CheckMode` del MotionSwitcher: `'ai'` = servicio de Unitree activo (ignora `rt/lowcmd`), `''` = Debug |
| `./lectores.sh mando` | `lee_mando` | no | `wireless_remote[40]` y el kp/kd del control interno |
| `./lectores.sh mando-vivo [s]` | `mando_vivo` | no | los botones del mando en vivo |
| `./lectores.sh espia-armsdk [s]` | `espia_armsdk` | no | consigna de `rt/arm_sdk` contra la medida |
| `./lectores.sh log [s]` | `log_bajo_nivel` | no | CSV de tobillos, muñecas y rodilla a 250 Hz |
| `./lectores.sh d435` | `d435_info.py` | no | info e imágenes de la D435i (`--frames`, `--guardar DIR`) |
| `./lectores.sh stream-d435` | `stream_d435.py` | no | los 4 flujos en `http://<ip>:8080` |
| `./lectores.sh atras [vx] [s]` | `atras.py` | **sí** | marcha atrás por `LocoClient` (-0.3 m/s, 10 s), FSM 201 |

La interfaz es `eth0`; `IFACE=lo ./lectores.sh ...` para otra.

## FSM, `rt/lowcmd` y `CheckMode`: qué dice cada uno

| Lo que se ve | Qué significa |
|---|---|
| `rt/lowcmd` a ~500 Hz (`postura`) | el control interno manda (FSM 201, de pie): no se puede usar `rt/lowcmd`; `rt/arm_sdk` sí |
| `rt/lowcmd` en silencio y `CheckMode` `'ai'` | **no es Debug**: `rt/lowcmd` se ignora sin error (visto con FSM 0, `GetBalanceMode` 8301) |
| `rt/lowcmd` en silencio y `CheckMode` `''` | Debug de verdad: el robot obedece `rt/lowcmd` y las piernas no se sostienen solas |

En Debug (`CheckMode` `''`) el `LocoClient` ya no responde y **`lee_fsm` aborta** con `BadCastException`: es normal, no
es un fallo del robot. `estado.sh` sigue y enseña el `CheckMode` igualmente.

## Lo que no tiene lanzador

| Fichero | Por qué |
|---|---|
| `prueba_standup`, `prueba_start` | Demostraron que `StandUp()`, `Start()` y `SetFsmId()` devuelven 0 **y no hacen nada** en el H1-2. Ya no hacen falta |
| `arm_sdk_corregido` | El ejemplo oficial de `rt/arm_sdk` con el peso en el slot 27 corregido. **Mueve los brazos** |
| `caminar_10s.py`, `esperar_y_caminar.py`, `arrancar_motion.py` | Intentos de la sesión 2 que no constan como probados |

## Recompilar

```bash
./compilar.sh
```

Rehace `build/` desde cero (el antiguo se configuró en `~/lectores_h1_2`, la ruta de antes). Se enlaza contra
`/opt/unitree_robotics/lib/cmake/unitree_sdk2`: el `unitree_sdk2Config.cmake` del `build/` del SDK no vale para
`find_package`.
