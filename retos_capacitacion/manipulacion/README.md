# manipulacion — reto R40 del H1-2: coger y clasificar objetos sin dar un paso

El H1-2, de pie en FSM 201 delante de una mesa, localiza objetos con la D435i de la cabeza, los coge con
las manos Inspire y los deja en su bandeja (reto `R40-RT-H1_2-0002`). Brazos por `rt/arm_sdk`, manos por
Modbus TCP; la marcha de Unitree equilibra el cuerpo.

- [docs/PLAN.md](docs/PLAN.md): el plan (montaje, arquitectura, interfaces, gemelo, decisiones, hitos y avance).
- [docs/RESULTADOS.md](docs/RESULTADOS.md): lo medido y lo que cambió el plan.

**Solo `rt/arm_sdk` y Modbus de las manos.** Nada de esta carpeta usa `Move`, `ReleaseMode()` ni
`rt/lowcmd`, ni toca `~/robotics40`. L2 + B es la parada de emergencia del operador; ESPACIO en el
terminal, la salida segura del programa. Ningún brazo con la cámara en el lazo antes del Hito 3.

## Estado

| Fase | Qué hay | Mueve el robot |
|---|---|---|
| 0 | Entorno propio en el PC2, cámara, cinemática e IK, gemelo con cámara virtual, calibración, percepción, planificador, brazo, mano, vigilante, supervisor y registro; todo probado en la PC (simulación de punta a punta) | no |
| Hito 1 | `medir_montaje.py` (montaje del 2026-10-13 medido), `poses_brazos.py`, `comprobar.py`; falta `mapa_alcance.py` y `marcar_montaje.py` | no |
| Hito 2 | `poses_calibracion.py`, `calibrar_camara_torso.py` (probada en el gemelo); falta grabar en el robot, la curva de pasos y la fuerza de la mano | sí (poses fijas) |
| Hito 3 | `manipulacion.py --simulacro`; falta probarlo en el robot | no |

## Dos procesos en el PC2

`pyrealsense2` exige `sudo` y, como root, el DDS de Unitree no arranca. Por eso `camara_servidor.py`
corre como **root** y publica los fotogramas por ZMQ en `tcp://127.0.0.1:5556` (mosaico en
`http://<robot>:5000`), y todo lo demás corre como **unitree** con `ejecutar.sh` o `manipulacion.sh`.
El servidor de cámara del seguidor y `camara.py` de robotics40 no pueden estar abiertos a la vez.

## Uso

Desde la PC los `.sh` se reenvían solos al robot por SSH (`comun.sh`). Se ejecuta el código del robot, así
que primero se despliega.

```bash
./desplegar.sh                      # código, config y modelo (URDF, mallas, modelo_h1_2_manos.py) -> ~/utec/manipulacion
./desplegar.sh --traer              # datos/ del robot -> datos/ de la PC
./instalar_entorno.sh               # una vez: ~/utec/manipulacion/.venv (pinocchio 4.1, mujoco 3.14, numpy 2.2...)

./camara_servidor.sh                # color, profundidad e IR con emisor (pide sudo); parar | estado | log
./ejecutar.sh herramientas/comprobar.py                                 # antes de cada tirada, solo lectura
./ejecutar.sh herramientas/grabar_dataset.py --nombre mesa_vacia --n 20 # captura (solo cámara)

# en la PC
python3 herramientas/medir_montaje.py datos/dataset_<fecha>_mesa_vacia  # Hito 1: mesa -> config/montaje.yaml
python3 herramientas/poses_brazos.py                                     # pose de observación (gemelo)
python3 herramientas/poses_calibracion.py --n 16                         # poses de calibración (gemelo)
python3 herramientas/calibrar_camara_torso.py datos/calibracion_<fecha>  # Hito 2 -> config/calibracion_<fecha>.yaml
python3 herramientas/evaluar_percepcion.py datos/dataset_<fecha>_<nombre>
python3 herramientas/generar_sintetico.py objetos --n 60                 # datos con verdad del gemelo
python3 manipulacion.py --nivel 2 --sim --asignacion tenis=A cubo=C --colocacion tenis=0.43,0.05 cubo=0.47,-0.07,0.3
python3 manipulacion.py --nivel 2 --sim --visor --velocidad 4            # ventana de MuJoCo: los 4 objetos a sus bandejas
python3 manipulacion.py --nivel 2 --sim --visor --parada-en TRANSPORTE+5 # salida segura con el objeto en la mano
python3 manipulacion.py --nivel 2 --sim --visor --parada-en TRANSPORTE+5 --salida-directa   # la del PDF, para comparar
python3 -m pytest tests/                                                 # MUJOCO_GL=glfw: las de render necesitan pantalla

# en el robot, Hito 3 y niveles (pide MOVER; ESPACIO = salida segura)
./manipulacion.sh --nivel 1 --simulacro
./manipulacion.sh --nivel 1                          # --escala-tiempo 2 por defecto: el doble de la duración mínima
./manipulacion.sh --nivel 2 --asignacion tenis=A cubo=C cilindro=B prisma=D
```

## Ficheros

| Ruta | Qué es |
|---|---|
| `config/manipulacion.yaml` | Límites de la sección 10, ganancias, tiempos, percepción, planificación, supervisor |
| `config/montaje.yaml` | Mesa medida (`medir_montaje.py`), zona de recogida, bandejas A–D y cruz (el plan) |
| `config/objetos.yaml` | Tabla por objeto: medidas, color, agarres, FORCE_SET, preforma, bandeja |
| `config/poses_brazos.yaml` | Pose de observación de cada brazo (`poses_brazos.py`) |
| `config/poses_calibracion.yaml` | Poses de la calibración (`poses_calibracion.py`) |
| `config/calibracion_<fecha>.yaml` | Cámara–torso y marcadores del dorso (`calibrar_camara_torso.py`) |
| `camara_servidor.py/.sh` | Servidor de la D435i (root) con el extrínseco profundidad → color |
| `comun.sh`, `ejecutar.sh`, `desplegar.sh`, `instalar_entorno.sh` | Lanzadores, despliegue y entorno propio |
| `manipulacion.py/.sh` | Programa principal: `--nivel N`, `--simulacro`, `--escala-tiempo`, `--sim` |
| `manip/mensajes.py` | Mensajes entre bloques (tabla de interfaces del plan) |
| `manip/robot.py` | `rt/lowstate` y FSM, solo lectura (sin LocoClient) |
| `manip/brazo.py` | `rt/arm_sdk` a 50 Hz, rampas, mínima sacudida y topes de la sección 10 |
| `manip/mano.py` | Manos por Modbus: FORCE_SET antes de cerrar, apertura comprobada, agarre, línea base |
| `manip/cinematica.py` | FK e IK propia sobre el URDF del repo (pinocchio 3 y 4) |
| `manip/calibracion.py` | Calibración cámara–torso por reproyección, Kabsch, ArUco |
| `manip/montaje.py` | Medida de la mesa y utilidades del montaje |
| `manip/percepcion.py`, `manip/dianas.py` | Fotograma → objetos en el torso; dianas del nivel 1 |
| `manip/planificacion.py` | Planes de tocar y coger, comprobados contra el gemelo |
| `manip/supervisor.py`, `manip/vigilante.py` | Máquina de estados, salida segura y causas de parada |
| `manip/registro.py`, `manip/teclado.py`, `manip/fuentes.py` | Registro de tiradas, tecla de parada, cámara o dataset |
| `sim/` | Gemelo en MuJoCo, cámara virtual, colisiones, robot y mano simulados, mundo simulado, visor (`--sim --visor`) |
| `herramientas/` | Medida, poses, calibración, datos sintéticos, evaluación, grabación y comprobación |
| `tests/` | Pruebas sin robot (pytest) |
| `datos/` | Grabaciones y registros (fuera de git) |

## Convenciones

Marco del torso (`torso_link` con la cintura a 0, que coincide con la pelvis): x adelante, y a la izquierda,
z arriba; metros y radianes. Marcos ópticos de la cámara: z adelante, x a la derecha, y abajo. `T_a_b` lleva
puntos de `b` a `a`. Juntas en el orden de `arm_sdk` (12 cintura, 13–19 izquierdo, 20–26 derecho). Manos:
DOF 0–5, 0 cerrado y 1000 abierto. Tiempos en `time.monotonic()` del PC2.
