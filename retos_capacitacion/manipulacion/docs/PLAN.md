# Plan del reto: H1-2 manipulación

Plan de trabajo del reto `R40-RT-H1_2-0002` (clasificar objetos con brazos, manos y cámara sin mover el
cuerpo), creado el 2026-10-09. Se avanza según este plan, pero puede cambiar con los resultados; lo
medido y lo que cambie irá a [RESULTADOS.md](RESULTADOS.md), que se crea con la primera medida. Las
casillas de [Orden de implementación](#orden-de-implementación) marcan el avance real.

El sistema corre en el PC2 del robot en dos procesos, como el seguidor de línea: la cámara como root y
el resto como `unitree`, porque el DDS no arranca con sudo. **Desde el 2026-10-15 el cuerpo entero va
por `rt/lowcmd` con el robot en el arnés** (ver [Cambio a rt/lowcmd](#cambio-a-rtlowcmd-2026-10-15)):
piernas en una postura de pie fija con su par de sostén y un estabilizador de tobillos con la IMU,
brazos con `tuned_gff` y la gravedad compensada; las manos, por Modbus TCP. La PC desarrolla, calibra
y comprueba sin robot, con un gemelo del montaje en MuJoCo que tiene cámara virtual y, ahora, un robot
con física colgado del pórtico. Se valida por hitos: de pie en el arnés (Hito 0), alcance y mesa
fijada, calibración y caracterización, percepción y planificación sin robot, simulacro y tiradas al
doble de la duración mínima.

## Cambio a rt/lowcmd (2026-10-15)

**Por qué.** En `comparar_ganancias.py` (`rt/arm_sdk`, FSM 201) la marcha de Unitree dio pasos en los
11 intentos, a veces durante la rampa de peso sin mover el brazo; con la pose de observación se echaba
atrás y al adelantar la mano izquierda daba pasos. El instructor lo atribuye al controlador de
equilibrio (cambiarlo es un trabajo de RL aparte) y autoriza trabajar con `rt/lowcmd` de cuerpo entero
y el robot sujeto por el arnés. Esto sustituye las restricciones del PDF de usar solo `rt/arm_sdk` y no
llamar a `ReleaseMode()` (sección 10); el resto de la sección 10 sigue (≥ 8 s por movimiento, ≤ 0.5
rad/s, FORCE_SET ≤ 800, salida segura).

**Qué se evaluó** (MuJoCo, `datos/evaluacion_lowcmd_20261015/`):

| Configuración | Resultado |
| --- | --- |
| Piernas en PD fijo, sin cuerda (cualquier postura, también inclinada atrás) | Cae en 1–2 s: péndulo invertido; los tobillos dan 160–280 N m/rad y hacen falta > m·g·h ≈ 600 |
| Postura fija + par de sostén + estabilizador IMU → par de tobillo, sin cuerda | De pie; ±0.1–1.5° y < 1 cm con un brazo adelante y atrás; sin deslizar |
| Cuerda cargando el 20 % del peso, sin estabilizador | De pie; ±2°, el torso se mueve 2–3 cm |
| Colgado sin tocar el suelo | 8° y 12 cm con cada movimiento: descartado |

El centro de masas no es el problema: los brazos lo mueven −0.8…+2.1 cm y siempre queda a ≥ 9.7 cm
del borde de los pies. Inclinar el torso atrás no ayuda (lleva el CdM al talón y quita alcance y vista).

**Cómo** (decisiones del usuario, 2026-10-15): los tres pasos, cada uno red del siguiente, con
`herramientas/arnes.py`:

1. **Colgado** con los pies ~5 cm sobre el suelo, en amortiguación: `ReleaseMode()` desde el programa
   (tras escribir SOLTAR, ≤ 3 intentos y 3 s sin nadie en `rt/lowcmd`), toma con kp de 0 a 1, postura
   de pie (la de FSM 201 hecha simétrica), brazos a 0 y tara de la carga de los pies. Medido el
   2026-10-16 (RESULTADOS §14): en Debug el robot ignora L2+B, así que lo atiende el programa
   (amortiguación), y sin órdenes los motores quedan sin par a los 0.2 s.
2. **Apoyo**: se baja el polipasto (manivela) hasta que los pies llevan 50–80 % del peso y `[e]`
   enciende el estabilizador y después pasa el par de sostén a "de pie". En ese orden: el par de pie
   afloja la cuerda solo, y sin estabilizador el robot se balancea (simulado).
3. **De pie, con la cuerda llevando parte del peso** (procedimiento de trabajo, decisión del usuario del
   2026-10-16): tras `[e]` las piernas descargan la cuerda solas y `[n]` (nivelar) lleva la referencia
   del estabilizador a la fija (IMU pitch +0.5°, la de FSM 201 con la misma postura de piernas) solo
   mientras el torso la sigue, y activa la integral. Nivelado (lo dice la pantalla), se centra el pórtico
   hasta que la cuerda quede vertical y se sube el polipasto hasta que los pies llevan 70–85 % (la cuerda
   el resto). En ese orden: al nivelar el torso se echa atrás 2–4° y mueve la cinta ~5 cm (2026-10-17). Con la cuerda floja el robot real se
   atascaba con los tobillos casi al límite (54 de 60 N m) y oscilaba (RESULTADOS §17); con parte del peso
   en la cuerda estaba quieto. Durante las tiradas no se toca el polipasto. Al terminar, al revés: subir el polipasto (el
   estabilizador se apaga solo por debajo del 35 % de carga y el par vuelve a "colgado" por debajo del
   15 %), brazos a la postura colgada y kp a 0.

**Brazos a 0** (codo doblado 90°, antebrazos horizontales hacia delante, la yema a 25 cm sobre el
tablero): el carrito entra por debajo y ya no hace falta la pose de observación para salir del reposo
sin barrer la mesa. Las manos a 0 tapan la imagen de las bandejas: para percibir sigue haciendo falta
una pose de observación, a la que ahora se llega por encima de la mesa.

**Datos medidos que lo apoyan:** el control interno de FSM 201 manda tobillo kp 80, rodilla 400, cadera
250 y cintura 600 (`rt/lowcmd` leído); la postura de FSM 201 deja, en el URDF, el torso 6° adelantado
con el pie plano mientras la IMU del robot marca ~0°, así que la postura se toma de lo medido y el
estabilizador se refiere a la IMU al encenderse; el par de las piernas medido de pie es ~22 % mayor que
el del modelo (escala de la carga 0.82, a recalibrar con la cuerda floja).

## Punto de partida

Medido el 2026-10-09 con el robot en FSM 201 y sin mover nada: una captura de la D435i tras recolocar la
mesa y la IK propia sobre el URDF del repo. Son medidas preliminares; las calibradas irán a
RESULTADOS.md.

| Qué | Valor | Cómo se obtuvo | Qué implica |
| --- | --- | --- | --- |
| Estado del robot | FSM 201, `CheckMode='ai'`, 27/27 motores sin fallo, `mode_machine` 6; D435i a 5000 Mbps; manos .211 (izq) y .210 (der) abiertas y sin error | `estado.sh`, `lee_fsm eth0`, `camara.sh usb`, `manos.sh leer` | Repetir antes de cada tirada |
| Mesa | Carrito negro brillante de 79.5 × 50 cm, superficie a 85.5 cm del suelo, labio de ~2.5 cm, seis alfombrillas grises, asas laterales hasta ~15 cm sobre la superficie, ruedas con freno | Usuario y profundidad | Labio y asas son obstáculos: entran en el gemelo y en la comprobación de colisiones |
| Posición de la mesa | Borde cercano a 0.30–0.33 m de la pelvis (d ≈ 17–20 cm del pecho), ~3 cm a la derecha del eje del robot, girada ~2.5° | ArUco 3, 4 y 5 en el IR, proyectados al plano | Dentro de los 10–35 cm del PDF. El marco de la mesa sale de los ArUco, sin suponerla recta |
| Plano y cámara | Cámara a 0.805 m de la mesa; pelvis a 0.973 m del suelo y mesa a −0.118 m de la pelvis; eje óptico 51.7° bajo el plano (el URDF dice 50.8°), roll 0.5° | RANSAC sobre 15 fotogramas de profundidad, residuo 3 mm | El torso va ~1° inclinado: la pose del URDF solo sirve de semilla de la calibración |
| Campo de visión sobre la mesa | Color desde x ≈ 0.36 m (FOV V 43.4°); IR y profundidad desde x ≈ 0.21 m (FOV V 64.7°) | Esquinas de la imagen proyectadas al plano | La zona de recogida empieza en x ≈ 0.40 m; los ArUco cercanos se leen en el IR |
| Profundidad | 92–99 % de píxeles válidos sobre el negro con emisor, ruido ~3 mm; las esferas salen con 43–52 mm de alto (miden 65–70); reflejos falsos de 15–25 mm | Captura con emisor | Centro de las esferas por ajuste de círculo y diámetro conocido; filtrar manchas por tamaño |
| Alcance por IK (en la pelvis; límites web∩URDF − 0.05 rad) | Tocar con la yema: x ≤ 0.50–0.55 m. Agarre desde arriba: x ≤ 0.50. Agarre de lado con los dedos 15–55° hacia abajo: x ≤ 0.55. De lado con los dedos horizontales: imposible, porque el brazo no baja la muñeca hasta la mesa. Soltar a 10–15 cm: x ≤ 0.60 | IK propia (pinocchio), rejilla de 5 cm, sin colisiones todavía | Banda útil de agarre con color: x 0.40–0.52 m. Las dos manos cubren \|y\| ≤ 0.10–0.15 m |
| Giro de la mano en planta | ~60–90° por mano con agarre desde arriba en el centro de la zona | IK con orientación, cada 30° | El giro de 90° del nivel 4 quizá no sale con un solo agarre |
| Manos (caracterización de `inspire_hand_interface`) | Con FORCE_SET 500 se pasa +112 gf a SPEED 50, +202 a 100 y +637 a 250. Fuerza sostenible de 455–745 gf por dedo según la pose. El dedo marca 70–232 gf solo por doblarse. Cubo: con 2 dedos 279/507 gf e inestable; con 3, 295/179/341 gf y estable. Si FORCE_ACT llega a FORCE_SET, el dedo deja de aceptar ANGLE_SET (en banco). Tras sujetar, el cero se corre 21–46 gf durante ~8–10 s | `Caracterizacion/exp2`, `exp3` y `Control/` | SPEED ≤ 100 al cerrar, FORCE_SET de 250–450 gf, comprobar que la mano abre, línea base con la mano abierta y ≥ 10 s sin carga |
| ArUco | DICT_4X4_50 de 5 cm (la profundidad da 46–49 mm: medir con regla). ID 0 dorso izq, 1 dorso der, 2 esquina lejana izq, 3 lejana der, 4 cercana izq, 5 cercana der | Detección en las fotos y en la captura | `DICT_4X4_250` da un falso ID 190 dentro del 3: usar la de 50 |
| Objetos | Pelota de tenis amarilla (7 cm, 58.7 g), cubo PLA blanco (6 cm, 60.5 g), prisma alargado (por imprimir) y un cuarto por decidir. La bola de espuma (21.7 g) queda fuera | Usuario y báscula | Ver [Montaje acordado](#montaje-acordado) |
| Bandejas | Hueco 12 × 12 cm, pared de 3 mm y 4 cm de alto; blanca, roja, azul y una cuarta | Usuario | Soltar con el objeto por encima de 4 cm: la mano a ~10–12 cm de la mesa |
| PC2 | x86_64, Ubuntu 22.04, Python 3.10.12, con internet. `unitree_sdk2py` en `~/pc2_offline/src/unitree_sdk2_python`. env_tv: cyclonedds 0.10.2, numpy 1.26.4, scipy 1.15.2, opencv 4.11.0.86, pyrealsense2 2.58.4, pymodbus 3.6.9, pyzmq 27.2, pinocchio 3.1 | `uname`, `pip list` | Entorno propio en `~/utec/manipulacion/.venv` con esas versiones |
| PC | mujoco 3.14, pinocchio 4.1, cv2 4.11 | Importación | Gemelo y comprobaciones en la PC; la cinemática debe funcionar con pinocchio 3 (robot) y 4 (PC) |

## Montaje acordado

Coordenadas en el marco del torso (coincide con la pelvis con la cintura a 0): x adelante, y a la
izquierda, z arriba. Las posiciones se fijan en el Hito 1 con el mapa de alcance con colisiones, y se
colocan con `marcar_montaje.py`, que pinta el objetivo sobre la imagen en vivo.

| Elemento | Posición inicial | Por qué |
| --- | --- | --- |
| Mesa **(revisado 2026-10-17)** | 77.8 × 48.8 cm entre labios, ArUco de las esquinas a 3.1 cm de cada labio (wincha, 2026-10-18). Borde cercano a **0.275 ± 0.01 m** (d ≈ 14 cm del pecho, ~3 cm por delante de la punta de los pies), centrada (± 1 cm) y recta (± 1°), con frenos; se coloca con el robot ya de pie y los brazos a 0 mirando `marcar_montaje.py`, y no se mueve en toda la sesión (`objetivo_mesa` en `montaje.yaml`) | Con los brazos a 0 lo que limita es el muslo contra el borde (≥ 0.24 m; ~5 cm de holgura a 0.27). Las baldas del carrito están a ras del tablero (confirmado). La base del pórtico rodea los pies: el carrito tiene que caber delante sin tocarla |
| Zona de recogida (desviación aceptada, revisada) | x 0.40–0.56, y −0.20…+0.20: **40 × 16 cm**, frente a los 40 × 25 del PDF | El color ve desde x ≈ 0.40 y el agarre desde arriba llega a x ≈ 0.56 (IK sin colisiones; el mapa con colisiones puede recortarla a ~0.52). Acercar la mesa no ensancha esta banda: la centra en el tablero |
| Bandejas A y B (izquierda, **revisado 2026-10-17**) | A azul en (0.40, +0.28) y B roja en (0.56, +0.28): a 12.5 y 28.5 cm del borde con la mesa a 0.275 | Rectángulo de 16 × 56 cm centrado en la cruz (las cuatro a 29 cm de ella), alineado con los bordes de la zona. A ±0.28 en vez de los ±0.30 de la guía: el brazo queda ≥ 8.1 cm de las asas al soltar (7.2 a ±0.30) y la bandeja a 1.7 cm de la zona. La azul (prisma) a la izquierda: el prisma se coge mucho mejor hacia ese lado (RESULTADOS §21) |
| Bandejas C y D (derecha, revisado 2026-10-17) | C blanca en (0.40, −0.28) y D naranja en (0.56, −0.28) | La roja y la naranja en lados distintos (tonos cercanos). Con los brazos a 0 las manos tapan parte de las cuatro: no hay sitio libre de esa sombra fuera de la zona y lejos de las asas; `marcar_montaje.py` las mide igual (le bastan dos paredes) |
| Dianas del nivel 1 | Cuatro pegatinas de 2 cm con una cruz, al azar dentro de la zona | — |
| Cruz del nivel 4 (desviación, revisada) | Centro de la zona, (0.48, 0) | El centro del tablero (x ≈ 0.53) queda casi en el borde de la zona |
| Huellas de los pies | Cinta alrededor de los dos pies, ya de pie en el arnés | Un pie que se desliza 5 cm cuenta como paso |
| Orden | Robot de pie con los brazos a 0 → `marcar_montaje.py`: carrito hasta "MESA EN SU SITIO", frenos, `[g]` mide la mesa vacía → bandejas hasta "BANDEJAS EN SU SITIO" | Las bandejas cambian el plano y el labio medidos |

Si el mapa con colisiones recorta las esquinas lejanas de la zona para la mano del lado contrario, la
zona se queda en trapecio o se usa la entrega de mano a mano (6.4.6). La mano contraria no llega a las
bandejas del otro lado: cada objeto lo coge la mano del lado de su bandeja.

Objetos y su bandeja (orden fijado por el usuario el 2026-10-17, en `objetos.yaml`; `--asignacion` lo cambia):
cilindro azul → bandeja roja, pelota amarilla → blanca, cubo blanco → naranja y prisma naranja → azul.

| Objeto | Medidas y peso | Agarre inicial | Riesgos |
| --- | --- | --- | --- |
| Pelota de tenis amarilla | Ø 7 cm, 58.7 g | Desde arriba con 4–5 dedos, o de lado inclinado; FORCE_SET a medir (no hay datos de banco) | Rueda al tocarla y rebota; la profundidad la ve baja |
| Cubo PLA blanco | 6 cm, 60.5 g | Tres dedos (pulgar, índice y medio) o mano completa; nunca pinza de dos | Brillos del PLA; blanco como la bandeja blanca y el papel de los ArUco (se separan por altura) |
| Prisma naranja | 4 × 4 × 10 cm tumbado, PLA (pesar) | Desde arriba, a lo ancho (4 cm), con el ancho de la mano a lo largo | Es el objeto del giro de 90° del nivel 4; con la mano de su bandeja se coge en ~80 % de sus orientaciones |
| Cilindro azul | Ø 5 × 8 cm de pie, PLA (pesar) | De lado inclinado o desde arriba | Da la cuarta forma; por su altura, se suelta más arriba |

## Arquitectura

```mermaid
flowchart LR
  subgraph PC2["PC2 del robot (unitree-h1-2-pc4)"]
    CAM["Proceso 1 · camara_servidor (root, sudo)<br/>D435i: color, profundidad e IR con emisor, 30 fps<br/>No toca el DDS; vídeo de depuración en :5000"]
    subgraph MAN["Proceso 2 · manipulacion (usuario unitree, .venv propio)"]
      F["Fuente<br/>ZMQ en vivo o dataset"] --> P["Percepción<br/>Fotograma → Escena en el torso"]
      CAL[("calibracion_fecha.yaml<br/>montaje.yaml")] --> P
      CAL --> PL
      P --> PL["Planificación<br/>mano, agarre y puntos de paso<br/>IK propia y colisiones"]
      PL --> S["Supervisor<br/>estados y vigilante"]
      S --> B["Cuerpo<br/>rt/lowcmd a 500 Hz, 27 juntas: postura de pie,<br/>par de sostén, estabilizador, brazos tuned_gff"]
      S --> M["Mano<br/>Modbus .211 / .210"]
      R["robot.py<br/>rt/lowstate: q, IMU, mando<br/>FSM con lee_fsm"] --> S
      REG["Registro<br/>CSV a 50 Hz, imágenes de cada agarre, JSON"]
    end
    CAM -- "Fotograma por ZMQ (127.0.0.1)" --> F
  end
  PC1["PC1 · motores en Debug<br/>(ReleaseMode: sin control de marcha)"]
  B -- "DDS rt/lowcmd" --> PC1
  PC1 -- "rt/lowstate 500 Hz" --> R
  MANOS["Manos Inspire RH56DFTP"] <-- "Modbus TCP (br0)" --> M
  MANDO["Mando · operador<br/>botón de parada leído por el programa<br/>L2+B = amortiguación (la hace el programa)"] --> R
  POLI["Polipasto del pórtico<br/>segunda persona en la manivela"] -.- PC1
  PORT["PC portátil<br/>calibración, mapa de alcance<br/>gemelo MuJoCo y cámara virtual"] <-- "ssh -t, rsync" --> MAN
```

El bloque de calibración son herramientas aparte: graban en el robot, calculan en la PC y escriben un
YAML con fecha. El proceso 2 lo lee al arrancar y no lleva ninguna transformación escrita en el código.

## Interfaces entre bloques

Cada bloque es una función sobre estos mensajes y no importa el SDK, salvo `robot.py`, `brazo.py` y
`mano.py`. Así la percepción corre igual sobre el dataset que en vivo, la planificación se prueba contra
el gemelo y la mano sin cámara. Irán en `manip/mensajes.py`.

Convenciones:

- Marco del torso (`torso_link` con la cintura a 0, que coincide con la pelvis): x adelante, y a la
  izquierda, z arriba. Metros y radianes; yaw en planta positivo antihorario.
- Tiempos con `time.monotonic()` del PC2; cada mensaje lleva su `t`.
- Juntas en el orden de arm_sdk: 12 cintura, 13–19 brazo izquierdo, 20–26 brazo derecho. Manos: DOF 0–5
  (meñique, anular, medio, índice, flexión del pulgar, rotación del pulgar); 0 cerrado, 1000 abierto.
- Ninguna transformación en el código: todas salen de `config/calibracion_<fecha>.yaml` y
  `config/montaje.yaml`.

| Mensaje | De → a | Campos (unidades) | Frecuencia |
| --- | --- | --- | --- |
| `Fotograma` | fuente → percepción, registro | `n`, `t_cam` (s), `t_rx` (s), `color` (BGR 640×480), `prof` (uint16 en mm, marco del IR izquierdo), `ir` (opcional), intrínsecos de color y profundidad, extrínseco profundidad → color, `emisor` | 30 Hz |
| `EstadoRobot` | `robot.py` → vigilante, planificación, supervisor, registro | `t`, `q`, `dq`, `tau` (27), `imu` (roll, pitch, yaw, gz), `fsm`, `motores_en_fallo`, `mando` (botones y ejes), `peso_armsdk`, `edad` | 500 Hz leído, 50 Hz registrado |
| `EstadoMano` | `mano.py` → supervisor, registro | `t`, `lado`, `angulo[6]`, `fuerza[6]` (gf), `base[6]` (gf, mano abierta), `force_set[6]`, `tactil` (solo las zonas pedidas), `error[6]`, `estado[6]`, `edad` | ~30 Hz por dedo |
| `Calibracion` | herramientas → todos (YAML con fecha) | `T_torso_ir`, `T_ir_color` (4×4), `T_dorso_izq`, `T_dorso_der` (marcador en `*_wrist_yaw_link`), `plano_mesa` (n, d en el torso), `residuo_mm` (medio y máximo), `n_poses`, `fecha` | Una por sesión |
| `Montaje` | `medir_montaje.py` → percepción, planificación, gemelo (YAML) | Marco de la mesa (origen y yaw), labio, asas, zona de recogida, bandejas A–D (centro, hueco y alto), cruz, objetos (clase, medidas, color) | Una por sesión |
| `Escena` | percepción → planificación, supervisor | `t` del fotograma, `ms` de proceso y, por objeto: `id`, `clase`, `p` (centro en el torso, m), `yaw` (rad, módulo su simetría), `medidas`, `confianza` (0–1), `visible`, `movido` (respecto de la escena anterior) | 5–30 Hz |
| `Plan` | planificación → supervisor | `objeto`, `mano`, `agarre` (preforma; FORCE_SET y SPEED por dedo; pose de agarre en el torso), `puntos_de_paso` (q de 7 juntas + duración), `pose_suelta`, `bandeja`, `comprobado` (límites, velocidad, altura segura y colisiones en el gemelo) | Uno por objeto, rehecho antes de cada agarre |
| `OrdenBrazo` | supervisor → cuerpo | `lado`, `q_obj` (7), `duracion` (s, ≥ 8), `perfil` (mínima sacudida); kp/kd de `tuned_gff` con el par de gravedad de las masas identificadas (`config/gravedad_brazos.yaml`) | Por segmento |
| `OrdenMano` | supervisor → mano | `lado`, `accion` (preforma, cerrar, sujetar, soltar, abrir), `force_set[6]` (≤ 800), `speed[6]`, `angulos[6]` | Por acción |
| `Salud` | vigilante → supervisor | `fsm`, `motores_en_fallo`, `edad_lowstate`, `edad_fotograma`, `mano_ok[2]`, `paso` (y la junta que lo delata), `yaw_desde_inicio`, `roll`, `pitch`, `error_seguimiento` (rad), `fuerza_max` (gf), `parada` (tecla o botón) | 50 Hz |
| `Decision` | supervisor → registro | `estado`, `motivo`, `objeto`, `peso`, `simulacro` | 50 Hz |

## Gemelo del montaje en MuJoCo

Una recreación del montaje medido para comprobar en la PC, sin robot, la cinemática, el alcance, las
colisiones, las trayectorias, la percepción y la calibración. Se construye con `mujoco.MjSpec` sobre
`h1_2_mujoco/modelo_h1_2_manos.py`, que ya carga el URDF con manos, impone las 12 juntas mimic y
comprueba colisiones. Todas las medidas salen de `config/montaje.yaml` y de la calibración: el gemelo
no lleva números escritos.

**Contenido** (`sim/escena.py`):

- **Robot:** URDF con manos del repo, con la pelvis fija a la altura y la inclinación medidas (0.973 m,
  ~1°), las piernas en la postura de pie leída de `rt/lowstate` y la cintura a 0. Los brazos y las
  manos toman la q medida o la planificada.
- **Mesa:** tablero, labio, asas (cápsulas), alfombrillas y estructura del carrito, en la pose de
  `montaje.yaml`.
- **Elementos del reto:** bandejas A–D (pared de 3 mm y 4 cm de alto), objetos con sus medidas y
  colores, dianas y cruz.
- **ArUco 0–5:** texturas reales generadas con `cv2.aruco`, en las esquinas de la mesa y en el dorso de
  cada mano.
- **D435i virtual:** dos cámaras en `torso_link`. Una de color (640×480, fovy 43.4°) y otra de
  IR/profundidad (640×480, fovy 64.7°). Llevan la pose del URDF hasta tener la calibración, y después
  la calibrada.

| Comprobación | Cómo | Criterio | Cuándo |
| --- | --- | --- | --- |
| Cinemática | FK de `cinematica.py` (pinocchio) frente a MuJoCo en 1000 q al azar: yema, punto de agarre, dorso y cámara | < 1 mm | Fase 0 |
| Alcance con colisiones | `mapa_alcance.py`: IK y colisión de mano y antebrazo con mesa, labio, asas, bandejas y cuerpo, por mano y tipo de agarre (también el giro en planta) | Mapa del Hito 1; elige d, la zona y la cruz | Hito 1 |
| Trayectorias | Cada `Plan` muestreado a 50 Hz: límites, ≤ 0.5 rad/s, nada bajo la altura segura salvo el descenso final, distancia a mesa, bandejas, cuerpo y el otro brazo | Ningún contacto previsto; holgura ≥ 2 cm | En el planificador, siempre; informe en el Hito 3 |
| Percepción con la cámara virtual | Imágenes de color y profundidad con los objetos en poses al azar, la mano tapando el objeto y un objeto movido | Error de centro y de yaw frente a la verdad del simulador; % de reconocimiento | Antes del Hito 2, sin robot |
| Calibración con la cámara virtual | El brazo virtual a las mismas poses que la grabación real; la calibración debe recuperar la pose conocida de la cámara | Sin ruido: < 3 mm y < 0.3° | Fase 0 |
| Calibración real | Render con la pose calibrada superpuesto a la imagen real: bordes de la mesa, ArUco, bandejas y la mano | Desfase en píxeles, convertido a mm | Hito 2 |
| Reproducción | `reproducir_mujoco.py` con la q medida en el CSV y, al lado, las imágenes grabadas | Ver qué hizo de verdad el brazo (la caída de ~5°, contactos) | Hito 3 y tras cada tirada fallida |
| Caída del brazo (opcional) | Con gravedad, actuadores de posición con kp 100–150 y kd 2 en los brazos, y la pelvis fija | Comparar con el error medido en el robot | Hito 2 |
| De pie por `rt/lowcmd` **(nuevo)** | `sim/cuerpo_simulado.py`: el robot con física colgado de un gancho con polipasto, que obedece LowCmd; `arnes.py --sim [--operador-auto]` y `tests/test_cuerpo.py` | De pie sin cuerda; inclinación y deslizamiento con los brazos moviéndose | Hito 0, antes de cada cambio del controlador |

Límites del gemelo: las fuerzas de contacto de la mano no son fiables y el robot con física no conoce
los ~6° de desajuste URDF/IMU, la conversión del tobillo paralelo ni los retardos reales; corre en la
PC, no en el PC2. La inclinación y el deslizamiento de verdad solo se miden en el robot. El simulador
C++ de `unitree_mujoco` sí acepta `rt/lowcmd` de cuerpo entero (su cinta elástica hace de cuerda): sirve
para probar el programa por DDS antes del robot.

## Decisiones técnicas iniciales

Son puntos de partida, no respuestas: cada valor se sustituye por el medido y entra en el informe con
su método.

| Pregunta del PDF | Decisión inicial | Cómo se valida |
| --- | --- | --- |
| 6.1.1 Alcance y d | IK propia con posición y orientación con tolerancia: palma abajo ≤ 35°, de lado con los dedos 15–55° abajo, yema abajo ≥ 45°; colisiones en el gemelo | Mapa por mano y por agarre; la d elegida y su porqué |
| 6.1.2 Qué ve la cámara | Plano RANSAC con emisor; color desde 0.36 m e IR desde 0.21 m | Error del plano y de los ArUco sobre la mesa vacía, contra la regla |
| 6.1.3 Cámara–torso | Dorso a ≥ 15 poses por mano con la q **medida**; mínimos cuadrados no lineales sobre `T_torso_ir` y `T_dorso`, empezando con Kabsch y la pose del URDF | Residuo medio y máximo; calibración probada antes sobre la cámara virtual |
| 6.1.4 Consigna frente a medida | Calibrar con la q medida; la diferencia con la consigna es la caída del brazo y se modela aparte | Comparar las dos calibraciones |
| 6.1.5 Vibración de la cámara | ArUco fijo de la mesa durante 60 s con el robot quieto | Desviación en mm |
| 6.2.1 Color o profundidad | Lo que sobresale del plano separa; el color solo identifica | % de detección en el dataset |
| 6.2.2 Reconocimiento | Color (HSV), altura, tamaño en planta y forma (círculo frente a rectángulo) | Matriz de confusión con los cuatro objetos |
| 6.2.3 Centro y yaw | Esferas: círculo y diámetro conocido. Cubo y prisma: rectángulo de la cara superior en la vista desde arriba, yaw módulo 90° y 180° | Contra la regla y contra la verdad del gemelo |
| 6.2.4 Brazo y mano en la imagen | Máscara del propio robot proyectando cápsulas de los eslabones con la FK y la calibración | Ningún objeto falso con el brazo a la vista |
| 6.2.5 Oclusión | Pose medida antes del descenso, con el brazo fuera de la vista, y confirmación por la mano (fuerza y táctil) | Distancia a la que se pierde el objeto en el dataset |
| 6.2.6 Objeto movido | Se vuelve a percibir antes de cada agarre; un objeto se da por movido si se desplaza > 2 cm o gira > 10° | Prueba con el objeto desplazado 5–10 cm |
| 6.3.1–2 Pasos frente a duración **(revisado)** | Con `rt/lowcmd` no hay marcha que dé pasos: se mide la inclinación de la IMU y el deslizamiento de los pies con el estabilizador a 8, 10, 20 y 30 s | Curva de inclinación (`arnes.py [m]` y su registro) |
| 6.3.3 El otro brazo | Quieto a 0 o en la observación | Inclinación con y sin el otro brazo |
| 6.3.4 Error de ~5° **(revisado)** | `tuned_gff` con el par de gravedad de las masas identificadas en `h1_2_joint_control` (rms 0.34 N m); corregir con el dorso visible antes del descenso final | Error de la yema en el nivel 1 |
| 6.3.5 Articulares o cartesianas **(revisado)** | Puntos de paso cartesianos resueltos con IK y movimientos articulares de mínima sacudida entre ellos; altura segura de 15 cm (10 cm al soltar) | Los puntos altos van a 18 cm: entre ellos el brazo bajaba 1–4 cm. Cada tirada pasa por una pose de observación: desde el reposo, subir el antebrazo barre la mesa (RESULTADOS §7) |
| 6.3.6 Cintura (índice 12) | A 0 con kp 200 / kd 2 (`tuned_gff`); girarla 20° no amplía el alcance hacia delante | Yaw del torso en el registro |
| 6.3.7 Fin y aborto **(revisado 2026-10-15)** | Congelar, abrir, subir a la altura segura y brazos a 0 sobre la mesa; el robot sigue de pie con el estabilizador. Para terminar: retirar el carrito, colgar con el polipasto, brazos a la postura colgada y kp a 0 | Simulado en `arnes.py --sim`; con los pies cargados el programa nunca suelta (el robot caería en la cuerda) |
| 6.4.1 Agarre y margen | Uno por objeto (tabla de arriba), con preforma que tolere ±2 cm | Margen medido desplazando el objeto en el gemelo y en el robot |
| 6.4.2 FORCE_SET | 250–450 gf con SPEED ≤ 100; tabla por objeto medida con la báscula | Fuerza real y sobreimpulso por objeto |
| 6.4.3 ¿Hay objeto en la mano? | Dedos parados lejos de 0 con fuerza por encima de su curva de flexión libre | Mano vacía frente a mano con objeto, en el robot |
| 6.4.4 Contacto con la mesa | Táctil de la yema del índice primero; fuerza del dedo y error de seguimiento como respaldo | Cuál aparece antes y con qué umbral, con la báscula |
| 6.4.5 Soltar | Bajar a 10 cm sobre la bandeja, abrir primero el pulgar y luego los dedos, comprobar que la mano abrió y subir en vertical | El objeto queda quieto en el gemelo y en el robot |
| 6.4.6 Entrega de mano a mano **(revisado)** | Cambio de agarre por la mesa: la otra mano deja el objeto girado en la zona y la de su bandeja lo coge | Con la palma abajo el ancho de la mano solo gira ~75° (RESULTADOS §6): un prisma en ~20 % de las orientaciones lo necesita (pendiente) |
| Nivel 4 · giro de 90° | Usar todo el giro disponible entre la recogida y la cruz. Si no llega a 90°, dos reagarres de 45° sobre la cruz | Mapa de giro de la mano del Hito 1 |

## Supervisor

```mermaid
stateDiagram-v2
  [*] --> ESPERA
  ESPERA --> ENTRADA: palabra MOVER, FSM 201, manos y cámara bien
  ENTRADA --> OBSERVAR: rampa de peso de 12 s hecha
  OBSERVAR --> PLANIFICAR: escena con confianza
  PLANIFICAR --> APROXIMAR: plan comprobado
  PLANIFICAR --> OBSERVAR: objeto fuera de alcance o sin plan
  APROXIMAR --> DESCENDER: sobre el objeto, a la altura segura
  DESCENDER --> CERRAR: pose de agarre
  DESCENDER --> RETIRAR: contacto de la yema (nivel 1)
  CERRAR --> LEVANTAR: agarre comprobado
  CERRAR --> OBSERVAR: agarre fallido (abrir, subir, reintentar)
  LEVANTAR --> TRANSPORTAR: objeto aún en la mano
  TRANSPORTAR --> SOLTAR: sobre la bandeja
  SOLTAR --> RETIRAR: mano abierta comprobada
  RETIRAR --> OBSERVAR: quedan objetos
  RETIRAR --> SALIDA: tarea terminada
  SALIDA --> ESPERA: reposo y peso a 0.00 en ≥ 5 s
  ESPERA: ESPERA · peso 0, el instructor puede colocar
  ENTRADA: ENTRADA · rampa de 12 s siguiendo la postura medida
  OBSERVAR: OBSERVAR · brazos fuera de la vista, escena nueva
  SALIDA_SEGURA: SALIDA SEGURA · congelar, abrir la mano, subir a la altura segura y peso a 0 en ≥ 5 s
```

**Revisión pendiente (2026-10-15):** con `rt/lowcmd` el supervisor no tiene peso que subir ni bajar.
ENTRADA pasa a ser el procedimiento de `arnes.py` (colgado → apoyo → de pie, brazos a 0) dentro del
mismo proceso (dos procesos no pueden publicar en `rt/lowcmd`), ESPERA es "de pie con los brazos a 0"
(se puede acercar o retirar el carrito y colocar objetos) y SALIDA vuelve a los brazos a 0, espera a
que se retire el carrito y cuelga. Hasta reescribirlo, la máquina de estados de abajo es la de
`rt/arm_sdk`.

Cualquier estado con el robot moviendo los brazos pasa a SALIDA SEGURA por:

- inclinación respecto de la referencia del estabilizador > 10° (el estabilizador se apaga solo y se
  sostiene la postura), o un pie que se desliza;
- yaw > 5°;
- error de seguimiento por encima del fijado, o fuerza > 1200 gf;
- cámara sin fotogramas o mano sin respuesta;
- motor en fallo o `rt/lowstate` de más de 0.25 s (amortiguación: el robot cae en la cuerda), modo
  distinto de Debug, excepción, Ctrl+C, tecla ESPACIO o el botón de parada del mando (L1).

En simulacro recorre los mismos estados con la cámara en vivo y registra lo que mandaría: las
trayectorias se reproducen en el gemelo y la mano abre y cierra en el aire, pero no publica.

## A) Scripts nuevos

Todo va en `retos_capacitacion/manipulacion/`, que se copia con rsync a `~/utec/manipulacion/` del
PC2. De `~/robotics40` solo se lee, por ejemplo `lectores_h1_2/build/lee_fsm`.

Los lanzadores siguen el patrón de `seguidor_linea/comun.sh`: se reenvían solos por SSH si no están en
el robot, fijan `PYTHONNOUSERSITE=1`, quitan `CYCLONEDDS_URI`, se niegan a correr como root y usan
`~/utec/manipulacion/.venv/bin/python`. Solo `camara_servidor.sh` usa sudo, y ese proceso no toca el
DDS.

| Fichero | Bloque | Qué hace | Dónde corre | Hito | Estado |
| --- | --- | --- | --- | --- | --- |
| `README.md`, `comun.sh`, `ejecutar.sh` | — | Lanzadores con el `.venv` propio | Ambos | 0 | pendiente |
| `instalar_entorno.sh` | — | Crea `.venv` en el PC2 con las versiones de env_tv y pinocchio 3; `unitree_sdk2py` con un `.pth` hacia `~/pc2_offline/src/unitree_sdk2_python`, sin tocar esa carpeta | Robot | 0 | pendiente |
| `config/manipulacion.yaml` | Configuración | Límites de la sección 10, ganancias, duraciones medidas, altura segura, palabra de confirmación, IPs | Ambos | 0 | pendiente |
| `config/montaje.yaml` | Configuración | Mesa, labio, asas, zona, bandejas, cruz, ArUco; lo escribe `medir_montaje.py` | Ambos | 1 | pendiente |
| `config/objetos.yaml` | Configuración | Por objeto: clase, color, medidas, peso, agarre, preforma, FORCE_SET, SPEED | Ambos | 2 | pendiente |
| `config/calibracion_<fecha>.yaml` | Calibración | Lo escribe `calibrar_camara_torso.py` | Ambos | 2 | pendiente |
| `camara_servidor.py` + `.sh` | Fuente | Copia del de `seguidor_linea`: color, profundidad e IR con emisor, y el extrínseco profundidad → color en los metadatos | Robot, sudo | 0 | pendiente |
| `manip/mensajes.py` | Interfaces | Los mensajes de la tabla de interfaces | Ambos | 0 | pendiente |
| `manip/fuentes.py` | Fuente | `FuenteZmq` y `FuenteDataset` con la misma interfaz | Ambos | 0 | pendiente |
| `manip/robot.py` | E/S del robot | `rt/lowstate` (q, IMU, mando), FSM con `lee_fsm`, `mode_machine` leído | Robot | 0 | pendiente |
| `manip/brazo.py` | Brazo | `rt/arm_sdk` a 50 Hz con el peso en el slot 27 y el CRC; rampas de 12 s con tope de 2° y de ≥ 5 s; mínima sacudida; tope de 0.5 rad/s y kp ≤ 150; cintura sujeta; `congelar()` | Robot | 0 | sustituido por `cuerpo.py` (2026-10-15) |
| `manip/cuerpo.py` **(nuevo)** | Cuerpo | `rt/lowcmd` a 500 Hz de las 27 juntas: postura de pie, brazos (`tuned_gff`), par de sostén colgado/de pie, estabilizador IMU → tobillos, rampas de kp, amortiguación por fallo, lowstate viejo o L2+B del mando; tara de la carga colgado | Robot | H0 | hecho, probado en simulación |
| `manip/dinamica.py` **(nuevo)** | Cuerpo | Pinocchio con base libre: par de gravedad (colgado), par de pie con la cuerda en el gancho, carga de cada pie desde el par de las piernas | Ambos | H0 | hecho |
| `manip/modo.py` **(nuevo)** | Cuerpo | CheckMode, `ReleaseMode()` tras escribir SOLTAR (≤ 3 intentos), 3 s escuchando `rt/lowcmd` | Robot | H0 | hecho |
| `config/gravedad_brazos.yaml` + `herramientas/gravedad_brazos.py` **(nuevo)** | Cuerpo | Masas de los brazos identificadas en `h1_2_joint_control` pasadas a este URDF (mano agrupada en `wrist_yaw`); reproduce el rms del ajuste (0.34/0.36 N m) | PC | H0 | hecho |
| `herramientas/arnes.py` **(nuevo)** | Hito 0 | Los tres pasos en el arnés con teclas (sin distinguir mayúsculas); `--solo-colgado`; sigue si se corta el SSH; `--sim [--visor] [--operador-auto]` en la PC | Robot y PC | H0 | hecho, probado en simulación |
| `sim/cuerpo_simulado.py` **(nuevo)** | Gemelo | Robot con física colgado de un gancho con polipasto que obedece LowCmd | PC | H0 | hecho |
| `manip/mano.py` | Mano | Modbus por bloques separados; FORCE_SET antes de cada cierre; preforma, cerrar, sujetar, soltar y abrir comprobando ANGLE_ACT; borrar error (1004); línea base; nunca SAVE (1005) | Robot | 0 | pendiente |
| `manip/cinematica.py` | Planificación | IK propia (Levenberg–Marquardt con límites) sobre el URDF del repo; marcos de la yema, del punto de agarre y del dorso; pinocchio 3 y 4 | Ambos | 0 | pendiente |
| `manip/geometria.py` | Percepción | Píxel y profundidad → torso con la calibración; plano y marco de la mesa | Ambos | 1 | pendiente |
| `manip/percepcion.py` | Percepción | `Fotograma` → `Escena`: plano, lo que sobresale, máscara del robot, clase, centro, yaw, confianza, movido | Ambos | 2 | pendiente |
| `manip/planificacion.py` | Planificación | `Escena` + estado de los brazos → `Plan`, comprobado contra el gemelo | Ambos | 2 | pendiente |
| `manip/supervisor.py` | Supervisor | Máquina de estados; único que cambia el peso | Ambos | 3 | pendiente |
| `manip/vigilante.py` | Supervisor | Causas de la salida segura de la sección 10 | Ambos | 3 | pendiente |
| `manip/registro.py` | Registro | CSV a 50 Hz (q, consignas, IMU, manos, estado), imágenes de cada agarre y JSON por tirada | Robot | 0 | pendiente |
| `manip/teclado.py` | Supervisor | Palabra de confirmación y ESPACIO como parada | Robot | 0 | pendiente |
| `sim/escena.py` | Gemelo | Escena MuJoCo del montaje desde `montaje.yaml` y la calibración | PC | 0–1 | pendiente |
| `sim/camara_virtual.py` | Gemelo | Render de color, profundidad y segmentación con los intrínsecos de la D435i | PC | 0 | pendiente |
| `sim/colisiones.py` | Gemelo | Distancias y contactos de una q o de una trayectoria (reutiliza `h1_2_mujoco`) | PC | 0 | pendiente |
| `manipulacion.py` + `.sh` | Principal | `--nivel N`, `--simulacro`, `--escala-tiempo 2`; pide MOVER antes de mover | Robot | 3 | pendiente |
| `herramientas/comprobar.py` | — | Robot, manos y cámara, solo lectura | Robot | 1 | pendiente |
| `herramientas/medir_montaje.py` | 6.1.2 | Plano, ArUco de la mesa, bandejas y objetos → `montaje.yaml` | Robot | 1 | pendiente |
| `herramientas/marcar_montaje.py` | Hito 1 | Mesa (borde, lateral y giro) y bandejas medidas en vivo frente a su objetivo, con lo que falta; pinta mesa, zona, bandejas y cruz en `http://<robot>:5001` y `[g]` guarda la mesa | Robot y navegador | 1 | hecho (probado en la PC con grabación real y gemelo) |
| `herramientas/mapa_alcance.py` | 6.1.1 | Mapa por mano, agarre y giro en planta, con colisiones del gemelo | PC | 1 | pendiente |
| `herramientas/grabar_calibracion.py` | 6.1.3 | En el arnés, mientras `arnes.py` [k] recorre `poses_calibracion.yaml` (cada brazo desde 0 y de vuelta): una muestra (color, profundidad, q medida y mandada) cada vez que un brazo se queda quieto 1 s en una pose nueva; solo lee | Robot | 2 | hecho, sin probar en el robot (el recorrido, con física en `arnes.py --sim`) |
| `herramientas/calibrar_camara_torso.py` | 6.1.3–4 | Calibración y su residuo; con la q medida y con la consigna | PC | 2 | pendiente |
| `herramientas/vibracion_camara.py` | 6.1.5 | ArUco fijo durante 60 s con el robot quieto | Robot | 2 | pendiente |
| `herramientas/curva_pasos.py` + `analizar_pasos.py` | 6.3.1–2 | Un mismo movimiento a 8, 10, 20 y 30 s; inclinación y deslizamiento (antes: pasos y giro) | Robot y PC | 2 | pendiente |
| `herramientas/fuerza_mano.py` | 6.4.2–4 | Fuerza real frente a FORCE_SET y SPEED con la báscula, apertura tras cerrar por fuerza y contacto de la yema | Robot | 2 | pendiente |
| `herramientas/grabar_dataset.py` | 6.2 | Objetos en posiciones medidas con regla, con y sin mano delante | Robot | 2 | pendiente |
| `herramientas/generar_sintetico.py` | 6.2 | Dataset de la cámara virtual con la verdad del simulador | PC | 0–2 | pendiente |
| `herramientas/evaluar_percepcion.py` | 6.2 | Error contra la regla o contra la verdad, reconocimiento, oclusión, objeto movido, ms por fotograma | PC y robot | 2 | pendiente |
| `herramientas/comprobar_trayectorias.py` | Sección 5 | Todos los planes de un nivel contra el gemelo, con informe | PC | 3 | pendiente |
| `herramientas/reproducir_mujoco.py` | Hito 3 | Una tirada o un simulacro en el gemelo, con su vídeo | PC | 3 | pendiente |
| `herramientas/metricas.py` | Sección 9 | Métricas por tirada | PC | Cierre | pendiente |
| `tests/` | Pruebas | Cinemática (pinocchio frente a MuJoCo), geometría, percepción sobre datos sintéticos y grabados, supervisor y vigilante | PC con pytest; robot con el `.venv` | 0–3 | pendiente |
| `docs/RESULTADOS.md`, `docs/INFORME.md` | Cierre | Medidas y respuestas a 6.1–6.4 | — | Todos | pendiente |

## B) Qué se ejecuta en la PC y en el robot

**En la PC** se desarrolla, se calibra, se planifica contra el gemelo y se analiza; nada del lazo de
control corre aquí. Por WiFi se usa `unitree@192.168.0.143`.

```bash
cd ~/Documents/h1_2_utec/retos_capacitacion/manipulacion
# desplegar y traer datos (la calibración y el montaje los escribe su herramienta: no pisarlos)
rsync -av --exclude datos/ --exclude __pycache__ --exclude .venv ./ unitree@192.168.0.143:~/utec/manipulacion/
rsync -av unitree@192.168.0.143:~/utec/manipulacion/datos/ datos/

python3 -m pytest tests/
python3 herramientas/arnes.py --sim --visor                               # Hito 0 en la PC: [u]/[j] polipasto
python3 herramientas/arnes.py --sim --operador-auto                       # ensayo completo sin teclear
python3 herramientas/generar_sintetico.py --n 200                         # Fase 0
python3 herramientas/mapa_alcance.py --montaje config/montaje.yaml        # Hito 1
python3 herramientas/calibrar_camara_torso.py datos/calib_<fecha>         # Hito 2
python3 herramientas/analizar_pasos.py datos/pasos_<fecha>                # Hito 2
python3 herramientas/evaluar_percepcion.py datos/dataset_<fecha>          # percepción sin robot
python3 herramientas/comprobar_trayectorias.py --nivel 1 2 3 4            # planificación sin robot
python3 herramientas/reproducir_mujoco.py datos/tirada_<fecha>            # Hito 3 y tras cada tirada
python3 herramientas/metricas.py datos/tirada_<fecha>                     # cierre
```

**En el robot**, en terminales separadas (el PC2 no tiene `tmux`). Cada sesión sigue este orden:
`estado.sh`, `camara.sh usb`, `manos.sh leer`, servidor de cámara en la terminal 1 y el programa de la
fase en la terminal 2.

| Programa | Usuario y terminal | Para qué | Cuándo |
| --- | --- | --- | --- |
| `~/robotics40/estado.sh`, `camara.sh usb`, `manos/manos.sh leer` | `unitree` | FSM 201, motores, cámara a 5000 Mbps, manos | Antes de cada tirada |
| `instalar_entorno.sh` | `unitree` | Crear el `.venv` | Una vez |
| `herramientas/arnes.py --solo-colgado` | `unitree`, terminal 2 (desde la PC: `./ejecutar.sh`) | Solo el paso 1 (colgado): postura y tara | Tras cambiar la postura o las ganancias |
| `herramientas/arnes.py` | `unitree`, terminal 2 | Los tres pasos: colgado, apoyo y de pie | Hito 0 |
| `camara_servidor.sh --color --profundidad --emisor` | root por sudo, terminal 1 | Fotogramas por ZMQ | Toda fase con cámara |
| `herramientas/comprobar.py` | `unitree`, terminal 2 | Robot, manos y cámara, solo lectura | Antes de cada tirada |
| `herramientas/medir_montaje.py`, `marcar_montaje.py` | `unitree`, terminal 2 | Mesa, bandejas y zona → `montaje.yaml` | Hito 1 |
| `herramientas/grabar_calibracion.py` (y [k] en `arnes.py`) | `unitree`, terminal 2 | Muestras de calibración mientras `arnes.py` recorre las poses | Hito 2 |
| `herramientas/vibracion_camara.py`, `curva_pasos.py --duraciones 30 20 10 8` | `unitree`, terminal 2 | Cámara quieta; pasos frente a duración | Hito 2 |
| `herramientas/fuerza_mano.py --mano izq --force-set 250 350 450` | `unitree`, terminal 2 | Con la báscula | Hito 2 |
| `herramientas/grabar_dataset.py --nombre objetos_a` | `unitree`, terminal 2 | Dataset de percepción | Antes del Hito 3 |
| `manipulacion.sh --nivel 1 --simulacro` | `unitree`, terminal 2 | Todo el lazo sin publicar en `arm_sdk` | Hito 3 |
| `manipulacion.sh --nivel N --escala-tiempo 2` | `unitree`, terminal 2 | Tiradas; la escala baja a 1 tras una tirada limpia | Niveles |

Los programas de `~/robotics40` no se modifican: solo se ejecutan o se leen.

## C) Con el mando y el polipasto

**Desde el 2026-10-15 (`rt/lowcmd`)** hacen falta dos personas: una en la manivela del polipasto y otra
con el mando y el terminal. El robot no pasa por FSM 201: tras encenderlo colgado se deja en
amortiguación (L2+B) y `arnes.py` hace el resto. La parada del programa es ESPACIO o L1 (sostiene la
postura); Ctrl+C dos veces suelta (kp a 0: el robot cae en la cuerda, que ya lleva parte del peso).
En Debug el robot ignora L2+B (medido el 2026-10-16): lo atiende el hilo de control de `arnes.py` (kp a 0
en 0.5 s, queda kd; de pie, el robot cae en la cuerda). Solo funciona mientras el programa corre: si el
programa muere, los motores quedan sin par a los 0.2 s y el robot también cae en la cuerda. Un corte del
SSH no lo mata: para, y cuando el polipasto lo cuelga termina solo. La tabla de abajo es la
operación con `rt/arm_sdk` (FSM 201) y queda como referencia.

El operador del mando no hace otra tarea y tiene L2+B listo en todas las fases en que el peso de
`arm_sdk` es mayor que 0. Nadie entra en el alcance de los brazos con peso: el instructor coloca los
objetos solo con el supervisor en ESPERA.

| Fase | Qué hace el operador | Ojo |
| --- | --- | --- |
| Arranque, solo si se reinició | Colgado y en la postura de cero, encender, esperar ~150 s, L2+B, L2+UP, bajar hasta apoyar los pies, R2+X; soltar el gancho solo si está estable | La postura de encendido fija el cero de las juntas |
| Colocación | Llevar el robot ante la mesa con START y los joysticks, y dejarlo de pie quieto (sin marcar el paso); luego se marcan las huellas | Después ya no se tocan los joysticks en toda la sesión |
| Al escribir MOVER | Joysticks a cero y las manos fuera de ellos | Un eje distinto de cero hace andar al robot: cuenta como paso |
| Calibración y curva de pasos | Solo vigila, con L2+B listo | Nadie delante de la cámara ni al alcance |
| Tiradas | Detrás del robot, solo con L2+B. Quien está en el terminal para con ESPACIO (salida segura) | L2+B amortigua y el robot cae despacio sobre la cuerda: último recurso |
| Tras un L2+B | Colgar, L2+UP, R2+X, `estado.sh`; explicar la parada con el registro antes de repetir | La calibración puede haber cambiado: comprobarla con la superposición del gemelo |
| Fin de sesión | Enganchar, L2+B y mantener a la vez los dos botones de batería | — |

No se tocan X/Y (cambian la altura y anulan la calibración), ni SELECT+Y/A, ni las combinaciones de
cambio de modo. Opcionalmente, un botón sin función en FSM 201 como parada por software, elegido con
`lectores.sh mando-vivo`.

## Plan por hitos

```mermaid
flowchart TD
  F0["Fase 0 · antes de la sesión<br/>entorno, cámara, brazo y mano sin mover, IK, gemelo y cámara virtual,<br/>percepción y calibración probadas sobre datos sintéticos"]
  H0{{"Hito 0 · de pie por rt/lowcmd en el arnés<br/>colgado → apoyo → de pie con la cuerda floja (arnes.py)"}}
  F1["0:00–0:30 · Preparación y espacio de trabajo<br/>interfaces por escrito; FSM 201, manos y cámara; medir_montaje + mapa_alcance<br/>Entregable: mapa sobre el tablero, d justificada, mesa fijada y montaje marcado"]
  H1{{"Hito 1 · desde aquí se puede mover un brazo"}}
  F2["0:30–1:10 · Calibración y caracterización<br/>grabar_calibracion, vibracion_camara, curva_pasos, fuerza_mano con la báscula<br/>Entregable: cámara–torso con residuo, curva de pasos, tabla por objeto"]
  H2{{"Hito 2 · calibración y tabla por objeto"}}
  F3["1:10–1:50 · Percepción y planificación sin robot<br/>evaluar_percepcion y comprobar_trayectorias en el gemelo; pytest<br/>Entregable: objetos reconocidos, error contra regla, trayectorias sin colisión"]
  F4["1:50–2:30 · Simulacro<br/>manipulacion.sh --simulacro y reproducir_mujoco; mano en el aire;<br/>un movimiento real a la pose de observación y vuelta<br/>Entregable: salida segura al tapar la cámara y al simular un giro"]
  H3{{"Hito 3 · desde aquí se permite la cámara en el lazo"}}
  F5["2:30–3:30 · Tiradas por niveles<br/>primero con --escala-tiempo 2; una tirada fallida se analiza antes de repetir"]
  F6["3:30–4:00 · Métricas y cierre<br/>metricas.py; informe con 6.1–6.4, calibración y registros"]
  F0 --> H0 --> F1 --> H1 --> F2 --> H2 --> F3 --> F4 --> H3 --> F5 --> F6
```

Ningún brazo se mueve antes del Hito 1, y ninguno con la cámara en el lazo antes del Hito 3. Las
poses de calibración y la curva de pasos son movimientos en lazo abierto.

Presupuesto por objeto: unos siete segmentos (aproximar, descender, cerrar, levantar, transportar,
soltar y retirar). Cada uno dura al menos la mínima medida en 6.3 y nunca menos de 8 s, así que sale
alrededor de 1 min por objeto a escala 1 y el doble en las primeras tiradas.

## Orden de implementación

Se implementa en el orden en que los hitos lo necesitan, y cada tarea termina con su prueba.

**Fase 0: sin mover el robot**

- [x] Carpeta con `README.md`, lanzadores, `config/manipulacion.yaml` y `manip/mensajes.py` con la tabla de interfaces
- [x] `instalar_entorno.sh` y comprobar en el PC2 las importaciones y el DDS desde el `.venv` (2026-10-13: pinocchio 4.1 y numpy 2.2, ver RESULTADOS §2)
- [x] `camara_servidor` y `fuentes.py`: grabado por ZMQ y leído como dataset (con el extrínseco profundidad → color)
- [ ] `robot.py` y `registro.py`, solo lectura; `mano.py` abriendo y cerrando en el aire (probados con el robot y la mano simulados; falta el robot)
- [x] `cinematica.py` con pruebas, y FK frente a MuJoCo < 1 mm (< 0.1 mm; pruebas también en el PC2)
- [x] `sim/escena.py` con el montaje medido (2026-10-13) y la cámara virtual; comparado con la captura real y cierre de la medida en 0.6 mm
- [x] `generar_sintetico.py`; calibración que recupera la cámara virtual (1.3 mm y 0.07°)
- [x] `percepcion.py` sobre los datos sintéticos (100 %, ≤ 3 mm) y la captura real; dianas del nivel 1 en el gemelo
- [x] `brazo.py`, `supervisor.py` y `vigilante.py` en simulacro, sin publicar: de punta a punta en el mundo simulado (niveles 1 y 2 y salida segura); falta `--simulacro` en el robot
- [x] Pose de observación (`poses_brazos.py`) y poses de calibración (`poses_calibracion.py`) con el gemelo
- [x] `comprobar.py` (solo lectura) escrito; falta pasarlo en el robot

**Hito 0: de pie por `rt/lowcmd` en el arnés (2026-10-15)**

- [x] `dinamica.py`, `cuerpo.py`, `modo.py`, `gravedad_brazos.yaml`, `cuerpo_simulado.py` y `arnes.py`, con `tests/test_cuerpo.py` (12 pruebas) y el ensayo completo en `arnes.py --sim --operador-auto`
- [x] Desplegar y, en el robot colgado y en amortiguación, `arnes.py --solo-colgado --pruebas` (2026-10-16): Debug al primer `ReleaseMode()`, lazo a 500 Hz con dt máx 6 ms, postura y brazos a 0 con error ≤ 1.8°; L2+B ignorado en Debug y motores sin par a los 0.2 s sin órdenes (RESULTADOS §14). Después: L2+B por el programa, tara colgado, teclas sin mayúsculas, SSH cortado; quitado `--pruebas`
- [x] `arnes.py` completo (2026-10-16): de pie con la cuerda floja, sin oscilar, y los brazos adelante sin mover el torso (±0.1°); escala de la carga 0.82 → 0.75 (RESULTADOS §15)
- [x] Inclinación de 3.4° adelante con el estabilizador solo proporcional → integral (Ki 30, solo de pie del todo); simulado
- [x] Integral en el robot (2026-10-16): vuelve a la referencia en ~30 s sin oscilar, pero la referencia de `[e]` estaba en +5.3° (el robot ~4° adelante con el nivel del móvil)
- [x] Inclinación de referencia fija con `[n]` (+0.5°, la de FSM 201) y la simulación con el desfase de ~5.4° entre el URDF y el robot; simulado
- [x] En el robot con `[n]` (2026-10-16): con la cuerda floja oscila (tobillos a 54 N m, ~110 s); con la cuerda llevando ~25 %, quieto a +0.8° → procedimiento de trabajo con la cuerda llevando parte del peso
- [x] Sesgo del tobillo (+13 N m), la referencia solo avanza si el torso la sigue, integral sin acumular saturada, con banda de 0.5° y sin roll, total del tobillo ≤ 50 N m; simulación con el desfase en cadera y rodilla (reproduce la oscilación); simulado
- [x] En el robot (2026-10-17, dos tiradas): con el pórtico centrado, quieto a ~0° con los brazos moviéndose (pitch −0.03…+0.09°, tobillos ≤ 9 N m); el aviso de la integral decía la dirección al revés (corregido) y el orden pasa a `[n]` → pórtico → polipasto
- [x] En el robot: el orden nuevo (2026-10-17, tres tiradas): nivela de +3.4° a +0.7° en ~20 s sin oscilar (tobillos ≤ 30 N m); con el pórtico centrado +0.5°; con la cuerda tensa y los brazos moviéndose +0.7…+1.0°, tobillos ≤ 16 N m. Una `[n]` pulsada mientras se encendía el estabilizador se perdía (y adelantaba el par de pie): corregido, y el siguiente paso en la línea de estado
- [ ] Integrar `Cuerpo` en `manipulacion.py` y el supervisor (ENTRADA = arnes, ESPERA de pie con los brazos a 0, SALIDA colgando)
- [ ] Nueva pose de observación alcanzable desde los brazos a 0 sin tapar la vista; regenerar `secuencia_ganancias.yaml`
- [x] Mesa medida con el robot de pie en el arnés (2026-10-17): borde cercano a 0.282 m, 2.1 cm a la derecha, yaw −0.7° (solo con los ArUco lejanos y el fondo de la mesa); bandejas de la guía, zona 40 × 16 y cruz en `montaje.yaml`; el gemelo coincide con la imagen real con la calibración del URDF (RESULTADOS §20)
- [ ] `mapa_alcance.py` con colisiones (asas a la altura de los codos, a ±0.48 m)

**Hito 1: espacio de trabajo**

- [ ] `comprobar.py`, `medir_montaje.py` y `mapa_alcance.py` con colisiones y giro en planta
- [x] Bandejas y asignación por color (2026-10-17): en el gemelo, rectángulo (0.40 / 0.56, ±0.28) con azul, roja, blanca y naranja en A–D; `marcar_montaje.py` (RESULTADOS §21)
- [x] Primera colocación en el robot (2026-10-18): carrito y bandejas con `marcar_montaje.py` (la C blanca, con regla: con los brazos a 0 solo se ve una pared), la naranja detectada, los cuatro objetos reconocidos (192/192), la mesa con wincha en `mesa_fisica` y el marco escalado con lo que ve la cámara (RESULTADOS §22)
- [ ] **La cámara ve la mesa ~3 % más grande** (ArUco 2–3: 738 mm de 716): medir la profundidad en la calibración cámara–torso con el ArUco de las manos por la zona, o calibrar el D435i en placa; hasta entonces, ~1–1.5 cm en la zona
- [ ] Volver a colocar el carrito con el marco corregido (el `[g]` del 18 da 0.286 m, y se movió a 0.300 al medir); frenos y huellas; zona marcada con cinta (16 × 40 cm, centrada entre los labios laterales)
- [ ] Ajustar el rango HSV del prisma (toca la roja y la pelota) con más capturas; profundidad del cubo blanco (4–5.5 cm de 6)
- [ ] Supervisor desde los brazos a 0: desde el reposo de la FSM la mano izquierda pasa a 13 mm de la mesa con el carrito a < 0.29 m (fallan las 5 pruebas de `test_supervisor.py`)

**Hito 2: calibración y caracterización**

- [x] Calibración en el arnés (2026-10-18): `poses_calibracion.py` desde los brazos a 0 delante del montaje medido (caminos sin choques y marcador leído en el gemelo), [k] en `arnes.py` y `grabar_calibracion.py`; `calibrar_camara_torso.py` compara la profundidad en el marcador con la esperada (RESULTADOS §23)
- [x] Calibración en el robot (2026-10-18): 32/32 muestras, 1.6 px / 1.7 mm; cámara a 13.9 mm y 2.25° del URDF (los ~2° son del marco de los brazos); montaje y `objetivo_mesa` pasados al marco calibrado (RESULTADOS §24)
- [x] Bandejas medidas con `[g]` en `marcar_montaje.py` (el robot suelta en lo medido) y comprobación del montaje al empezar cada tirada (RESULTADOS §25)
- [ ] Desplegar la calibración, el montaje y las poses (robot apagado hasta el 2026-10-20), volver a colocar las bandejas (~2 cm) con `marcar_montaje.py` y guardarlas con `[g]`
- [ ] Profundidad: ~2 % de más a 1 m sobre papel mate y ~1 % más con el plano del tablero negro; medir en la zona (poses de calibración más bajas, 5–12 cm sobre la mesa, o tocar dianas del nivel 1) y, si hace falta, corregir la profundidad con la distancia
- [ ] Superposición del gemelo calibrado sobre la imagen real
- [ ] `vibracion_camara.py`
- [ ] `curva_pasos.py` a 8, 10, 20 y 30 s; duración mínima al YAML
- [ ] `fuerza_mano.py` con la báscula: fuerza real, sobreimpulso por SPEED, apertura tras cerrar por fuerza y contacto de la yema
- [ ] `objetos.yaml`: peso, medidas, agarre y FORCE_SET por objeto

**Percepción y planificación sin robot**

- [ ] `grabar_dataset.py` y `evaluar_percepcion.py`: error contra la regla, reconocimiento, oclusión, objeto movido y ms por fotograma en el PC2
- [ ] `planificacion.py` y `comprobar_trayectorias.py` en el gemelo

**Hito 3: simulacro**

- [ ] `manipulacion.sh --simulacro` con la cámara en vivo y `reproducir_mujoco.py`
- [ ] Un movimiento real a la pose de observación y vuelta
- [ ] Salida segura al tapar la cámara y al simular un giro, vista en el registro
- [ ] Visto bueno del instructor al Hito 3

**Niveles y cierre**

- [ ] Nivel 1 a escala 2, luego a escala 1
- [ ] Niveles 2, 3 y 4, en ese orden
- [ ] `metricas.py` e `INFORME.md` con las respuestas a 6.1–6.4

## Confirmaciones

- [x] Carpeta `retos_capacitacion/manipulacion/` en el repo y `~/utec/manipulacion/` en el PC2, con PLAN y RESULTADOS
- [x] El código se escribe antes de la sesión de 4 h (Fase 0)
- [x] El carrito es la mesa definitiva; frenos sí; las asas no se quitan; no hay tablero mate
- [x] La curva de pasos se mide a 8, 10, 20 y 30 s (el PDF pide 5 s en 6.3.1 y prohíbe bajar de 8 s en la sección 10)
- [x] Cuerda con holgura durante las tiradas (con `rt/lowcmd`: 2–3 cm, de pie con el estabilizador)
- [x] **2026-10-15:** `rt/lowcmd` de cuerpo entero con el robot en el arnés (con el instructor), en los tres pasos (colgado, apoyo, de pie con la cuerda floja)
- [x] **2026-10-15:** el pórtico deja subir y bajar el robot con la manivela estando en Debug (fotos en `montajes/`: un solo gancho, cintas al soporte del cuello y a los cáncamos)
- [x] **2026-10-15:** `ReleaseMode()` desde el programa (tras escribir SOLTAR)
- [x] **2026-10-15:** mesa a ~0.27 m, bandejas como en la guía, zona 40 × 16; las baldas del carrito están a ras del tablero
- [x] **2026-10-15:** brazos con `tuned_gff` (con la compensación de gravedad), sin el tope de kp 150
- [x] Entorno propio en `~/utec`, no env_tv
- [x] IK propia sobre el URDF del repo
- [x] Zona de recogida reducida (~26 × 12 cm)
- [x] La bola de espuma queda fuera; se imprime un prisma alargado
- [x] Cuatro bandejas con paredes de ~4 cm (se pueden recortar más)
- [x] Hay báscula y regla
- [x] Objetos: pelota de tenis, cubo blanco, cilindro PLA azul de Ø 5 × 8 cm de pie y prisma PLA naranja de 4 × 4 × 10 cm tumbado; cuarta bandeja naranja (2026-10-12)
- [x] Lado del cuadrado negro de los ArUco: 5.0 cm con regla, DICT_4X4_50
- [x] Asignación de cada objeto a su bandeja (2026-10-17, el usuario): cilindro → roja, pelota → blanca, cubo → naranja, prisma → azul
- [x] Quién opera el mando en las tiradas: el usuario

## Fuentes

- Enunciado: [`Reto_Manipulacion_H1_2_Robotics40.pdf`](Reto_Manipulacion_H1_2_Robotics40.pdf) (R40-RT-H1_2-0002 v1.0)
- Fotos del montaje: [`../montajes/`](../montajes/) (`toma_lateral`, `ArUcos_manos`, `ArUcos_mesa`; `toma_frontal` solo en local: salen personas)
- Pórtico y arnés: `montajes/unitree-gantry.jpg`, `h1_2_sujeciones.jpeg`, `h1_2_colgado.jpeg`, `h1_2_colgado_1.jpeg` (2026-10-15)
- Ganancias y masas de los brazos: [`h1_2_joint_control/`](../../../h1_2_joint_control/) (`config/gains.yaml`: `tuned_gff`, `legs_hold`; `logs/gravity_params_*.json`)
- Capacitación: `~/Documents/UTEC/CAPACITACION/Capacitacion_H1_2_Robotics40.pdf` y su resumen en [`docs/CAPACITACION_H1_2.md`](../../../docs/CAPACITACION_H1_2.md)
- Código de referencia: `code_cap/caja_cuadrado.py`, `prueba_brazos_armsdk.py`, `caja_geometria.py`, `manos/`; en el robot, `~/robotics40/lectores_h1_2/`
- Manos: `~/Documents/inspire_hand_interface` (`Caracterizacion/exp2`, `exp3`, `resultados/MONTAJES.md`, `Control/`)
- URDF con manos: `ros_h1_2_ws/src/h1_2_inspire_description/urdf/h1_2_with_RH56DFTP_hands.urdf`
- MuJoCo: [`h1_2_mujoco/`](../../../h1_2_mujoco/) (`modelo_h1_2_manos.py`)
- Patrón de procesos, cámara y lanzadores: [`seguidor_linea/`](../../seguidor_linea/)
