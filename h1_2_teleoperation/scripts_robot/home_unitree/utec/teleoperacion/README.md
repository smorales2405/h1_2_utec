# teleoperacion — teleoperación XR con las Meta Quest 3

`xr_teleoperate` de Unitree (commit `817fb00`) para el H1-2 con las manos Inspire FTP: **los brazos y los dedos
siguen a los del operador**. Todo corre en el PC2. Funciona desde la sesión 10 (2026-09-25); probado con el
robot colgado, en dos sesiones (20 s y 6 min). Vive aquí desde el 2026-11-04; antes, en `~/robotics40/meta`, que sigue
intacta como respaldo.

## Cómo se lanza

```bash
~/utec/teleoperacion/teleop_xr.sh          # = arrancar.sh
~/utec/teleoperacion/teleop_xr.sh parar    # = parar.sh
```

`arrancar.sh` levanta tres cosas: el servidor de imagen (teleimager con la D435i, pide sudo), el driver de las
manos (Modbus ↔ DDS) y el teleop en primer plano. Imprime la URL para las gafas.

1. **Antes:** robot **colgado**, Debug con `L2+R2` (el teleop también llama a `ReleaseMode()`, y **sí**
   funciona: las piernas se sueltan), L2 + B a mano, nadie cerca de los brazos.
2. **Gafas:** WiFi **UTEC_H1_2**, navegador a `https://192.168.0.143:8012/?ws=wss://192.168.0.143:8012`,
   aviso del certificado → Avanzado → Continuar, y «Virtual Reality» **sin los mandos en la mano**.
3. Poner los brazos como el robot y pulsar **`r`** en el terminal. **`q`** para salir, en este orden:
   1. Los dos brazos van a **0°** (antebrazos horizontales hacia delante). Hasta aquí las manos siguen al operador.
   2. Se **abren del todo las dos manos** (espera a leerlas abiertas, máx. 3 s).
   3. Los brazos vuelven a la **postura inicial**, la medida al arrancar el teleop antes de moverse nada.

   Los dos movimientos van con rampa suave, a ~1 rad/s de media en la junta que más se mueve: ~1–3 s cada uno.

Sin teclado (no hay tmux en el PC2): `env_tv/bin/python enviar.py r | q | estado`, si se lanzó con `--ipc`.

## Ficheros

| Fichero | Qué es |
|---|---|
| `arrancar.sh` / `parar.sh` | arranque y parada de los tres procesos |
| `teleop_xr.sh` / `comun.sh` / `estado.sh` | entrada, entorno (y reenvío por SSH desde el PC) y estado del robot (usa `~/robotics40/lectores_h1_2`) |
| `lanzar_teleop.py` | lanza `teleop_hand_and_arm.py` sin tocar el código de Unitree: precarga PIL antes del `fork()`, limita `ReleaseMode()` a 3 intentos, al salir lleva los brazos a 0°, abre las manos y vuelve a la postura inicial, y sale como con `q` si se corta el SSH |
| `enviar.py` | órdenes `r`/`q`/`estado` por IPC |
| `prueba_gafas.py` | Vuer + imagen + seguimiento de manos **sin DDS**: para probar las gafas sin mover nada |
| `instalar.sh` | cómo se montó el entorno `env_tv` (conda, python 3.10, pinocchio 3.1.0). El de aquí es un clon del de `~/robotics40/meta` |
| `certs/` | certificado con las IP .0.143, 123.164, 124.164 y 127.0.0.1 |
| `xr_teleoperate/` | el repo de Unitree, con `cam_config_server.yaml` puesto solo con la D435i |
| `env_tv/` | el entorno conda (no copiar al PC) |
| `logs/` | `teleop.log`, `manos.log`, `image_server.log` de la última vez |

## Lo que hay que saber

- **Cómo saber si está en Debug:** `~/robotics40/lectores_h1_2/lectores.sh modo` tiene que dar `name ''`. El teleop lo
  consigue con `ReleaseMode()`; si sigue en `'ai'`, los brazos no obedecen aunque todo parezca ir bien.
- **Al salir, brazos y piernas se quedan sin fuerza** (Debug y nadie publicando). Por eso los brazos terminan en la
  postura inicial: desde 0° los antebrazos caerían. Si al lanzar colgaban, no deberían caer al salir (sin probar en el robot). Para volver: el mando.
- **Si se corta el SSH, el teleop sale solo como con `q`** (la misma secuencia). Lo detecta
  si se cuelga el terminal (se cierra la terminal o el `ssh` del PC) o si el PC deja de responder al ping 3 s (WiFi
  caída, portátil suspendido): el `sshd` del robot no tiene `ClientAliveInterval` y por sí solo no se enteraría en
  minutos. Para eso el teleop ignora `SIGHUP` y escribe solo en `logs/teleop.log`; en el terminal se ve un `tail`.
  Lo que pasó tras el corte queda en ese log. El servidor de imagen y el driver de las manos siguen: `./teleop_xr.sh parar`.
- El driver de las manos **las abre nada más arrancar**, antes de pulsar `r`.
- Los dedos no llegan a cerrarse del todo (el retargeting se queda corto, sobre todo en la izquierda).
- Si un dedo queda en `err` 5: `~/robotics40/manos/manos.sh borrar-error izq`.
- Usa la misma D435i que `camara.py`: `arrancar.sh` la para antes.
- Siempre `PYTHONNOUSERSITE=1` (ya lo ponen los scripts).
