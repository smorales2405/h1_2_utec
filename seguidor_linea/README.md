# seguidor_linea — reto R40 del H1-2 como seguidor de línea

El H1-2 camina siguiendo una línea con la D435i de la cabeza como única realimentación de
posición y `LocoClient.Move` para la marcha (reto `R40-RT-H1_2-0001`).

- [docs/PLAN.md](docs/PLAN.md): el plan (arquitectura, interfaces, decisiones, hitos y avance).
- [docs/RESULTADOS.md](docs/RESULTADOS.md): lo medido hasta ahora y lo que cambió el plan.

**Solo alto nivel.** Nada de esta carpeta usa `rt/lowcmd`, `rt/arm_sdk` ni `ReleaseMode()`.
L2 + B es la parada de emergencia; ninguna orden de marcha autónoma antes del Hito 3.

## Estado

| Fase | Qué hay | Mueve el robot |
|---|---|---|
| 0 | Servidor de cámara, lectura del robot, registro, fuentes de fotogramas, geometría, pruebas | no |
| Hito 1 | `herramientas/calibrar_camara.py`, `herramientas/comprobar.py`; calibrado el 2026-10-03 (`config/geometria.yaml`) | no |
| Dataset | `herramientas/grabar_dataset.py` (cinta blanca y negra), `escalon_vyaw.py`, `analizar_escalon.py` y `analizar_dataset.py` | solo `escalon_vyaw.py` |
| Hitos 2 y 3, niveles | por hacer: percepción, estimación, control, supervisor (`seguidor/vigilante.py` ya está) | — |

## Dos procesos en el PC2

`pyrealsense2` exige `sudo` y, como root, el DDS de Unitree no arranca (`fs.protected_regular=2`
y `/tmp/cdds.LOG` es de `unitree`). Por eso:

- `camara_servidor.py` corre como **root**: abre la cámara y publica cada fotograma por ZMQ en
  `tcp://127.0.0.1:5556`, con un mosaico de depuración en `http://<robot>:5000`. No importa el SDK.
- Todo lo demás corre como **unitree** con `ejecutar.sh`: lee `rt/lowstate`, la FSM y, en las
  tiradas, manda `Move`. Nunca con `sudo`.

## Uso

Desde la PC, los `.sh` se reenvían solos al robot por SSH (`comun.sh`); se ejecuta el código del
robot, así que primero se despliega. Por cable: `ROBOT_SSH=unitree@192.168.123.164 ./...`.

```bash
# desplegar (desde la raiz del repo, en la PC); la geometria la escribe el robot: no pisarla
rsync -av --exclude datos/ --exclude __pycache__ --exclude config/geometria.yaml seguidor_linea/ unitree@192.168.0.143:~/utec/seguidor_linea/
# traer la calibracion y los registros
rsync -av unitree@192.168.0.143:~/utec/seguidor_linea/config/geometria.yaml seguidor_linea/config/
rsync -av unitree@192.168.0.143:~/utec/seguidor_linea/datos/ seguidor_linea/datos/

cd seguidor_linea
~/Documents/UTEC/CAPACITACION/code_cap/camara.sh parar   # la camara solo la abre un proceso
./camara_servidor.sh                       # IR sin emisor; --profundidad, --color, --emisor
./ejecutar.sh herramientas/comprobar.py    # antes de cada tirada: robot y camara, solo lectura
./camara_servidor.sh parar | estado | log

# Hito 1: robot de pie en FSM 201, quieto, alineado con la linea, nadie delante de la camara.
# Emisor encendido (sin el la profundidad de este suelo tiene 3-15 cm de ruido). Medidas con cinta:
# puntera -> barra de fin, puntera -> una marca transversal de 60 cm a ~1 m, y suelo -> camara.
./camara_servidor.sh --profundidad --emisor
./ejecutar.sh herramientas/calibrar_camara.py --puntera-barra 3.58 --puntera-marca 1.00 --altura 1.65 --yaw
# escribe config/geometria.yaml; despues retirar la marca (el seguidor la tomaria por la barra)

# pruebas: en la PC con pytest, en el robot con unittest
python3 -m pytest tests/
./ejecutar.sh -m unittest discover -s tests
```

## Ficheros

| Ruta | Qué es |
|---|---|
| `config/seguidor.yaml` | Configuración a mano: red, cámara, límites del reto, supervisor, registro |
| `config/geometria.yaml` | La escribe `calibrar_camara.py` y pisa la sección `geometria` |
| `camara_servidor.py/.sh` | Servidor de la D435i (root) |
| `comun.sh`, `ejecutar.sh` | Entorno (`teleop_venv`, `PYTHONNOUSERSITE=1`, sin `CYCLONEDDS_URI`) y reenvío por SSH |
| `seguidor/mensajes.py` | Mensajes entre bloques (tabla de interfaces del plan) |
| `seguidor/robot.py` | `rt/lowstate`, FSM con `lee_fsm`, LocoClient; recorta a 0.4 / 0.2 / 0.5 siempre |
| `seguidor/geometria.py` | Plano del suelo, pose de la cámara, píxel ↔ suelo con corrección por la IMU |
| `seguidor/fuentes.py` | Cámara por ZMQ o dataset grabado, con la misma interfaz |
| `seguidor/registro.py` | CSV de `rt/lowstate` a ~100 Hz (columnas de `datos_cuadrado/` + mando), CSV y JSON |
| `seguidor/mando.py` | Botones y ejes del mando desde `wireless_remote` |
| `seguidor/calibracion.py` | Detectores mínimos de la cinta (línea, barra, marcas) y la inclinación por marcas |
| `seguidor/vigilante.py` | Paradas de la sección 10 que no dependen de la línea |
| `seguidor/analisis.py` | Respuesta a escalones, cadencia y filtros para analizar registros |
| `herramientas/` | Comprobación previa, calibración, grabación del dataset, escalones de vyaw y análisis de escalones y datasets |
| `docs/` | Plan, resultados y figuras clave (`docs/img/`) |
| `tests/` | Pruebas sin robot |
| `datos/` | Registros de cada tirada (fuera de git) |

## Convenciones

Marco del robot: x adelante, y a la izquierda, z arriba, con origen en la vertical de la cámara.
Ángulos en rad y positivos antihorario. Tiempos en `time.monotonic()` del PC2, el mismo reloj en
los dos procesos. Una línea a la izquierda da y > 0 y debe producir vyaw > 0.
