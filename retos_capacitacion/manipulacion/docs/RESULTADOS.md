# Resultados del reto: H1-2 manipulación

Lo medido hasta ahora y lo que cambió el [plan](PLAN.md). Cada número lleva cómo se obtuvo. La Fase 0
(2026-10-12 y 13) se hizo casi entera en la PC: el robot solo se usó para leer (estado, cámara) y para
crear el entorno propio.

## 1. Montaje medido (Hito 1, provisional)

`herramientas/medir_montaje.py` sobre `dataset_20261013_112444_mesa_20261013` (mesa vacía, 15 fotogramas de
profundidad con emisor), con la calibración del URDF (aún sin calibrar con las manos):

| Qué | Valor | Nota |
| --- | --- | --- |
| Plano de la mesa (torso) | n = (0.008, 0.021, 0.9997), d = 0.114 m; residuo 2.9 mm, 75 % de inliers | La mesa queda 0.114 m bajo la pelvis: pelvis a ~0.97 m del suelo |
| Inclinación | ~1.2° de roll del torso respecto de la mesa | La IMU daba 1.6° de roll ese día |
| Marco de la mesa | Borde de la superficie útil a 0.333 m de la pelvis, centro a −0.035 m (a la derecha), yaw −2.5° | Bien para trabajar: el marco sale de los ArUco |
| Superficie útil | 0.52 × 0.81 m | Del centro de los ArUco + 4 cm (el papel mide 7 cm) |
| Asas | A ±0.47 m del eje de la mesa, 13.5–15 cm sobre la superficie, de borde a borde | Entran en el gemelo como cápsulas |
| Labio | 2.3–3.7 cm según la banda que se mire | Medir con regla |
| ArUco vistos | 3, 4 y 5 (el 4 solo en la imagen ampliada ×2); el 2 no se lee (≈20 px, muy oblicuo) | Con 4, 5 y uno lejano basta |
| Lado de los ArUco | 46–49 mm (miden 50) | ~2 mm son sesgo del detector (también en el gemelo); el resto, la calibración del URDF |
| Distancias entre centros | 4–5: 730 mm; 5–3: 442 mm | Comparar con regla en el Hito 1 |

Postura de pie en FSM 201 (rt/lowstate del 2026-10-13, la del gemelo): caderas −0.41/−0.42, rodillas
0.67/0.71, tobillos −0.37/−0.38 rad; brazos en reposo izquierdo (0.28, 0.24, −0.11, 0.68, 0, 0.03, 0) y
derecho (0.29, −0.24, 0.12, 0.71, 0, 0.05, 0.01).

**La bandeja A en (0.42, 0.27) tapa el ArUco 4** (en (0.39, 0.33), con 7 cm de papel). No es un problema: el
montaje se mide con la mesa vacía antes de poner las bandejas, y el 2 y el 3 siguen visibles para comprobar
la cámara durante la sesión (6.1.5).

## 2. Entorno propio en el PC2

- El PC2 no tiene `python3-venv` (sin `ensurepip`): el venv se crea `--without-pip` y el pip lo instala el de
  `teleop_venv` con `--python` (solo se ejecuta, no se modifica). Nada del sistema cambia.
- pinocchio 4.1.0 como la PC, pero de pypi (`pin`, cmeel): trae `cmeel-boost` 1.89, que exige **numpy 2.2**
  (en la PC es la 1.26 de robotpkg). El código vale para las dos. MuJoCo 3.14.0, como la PC.
- Desde el venv: DDS a 500 Hz (`mode_machine` 6, sin fallos), cámara con `pyrealsense2` y numpy 2.2, ArUco,
  `unitree_hg` y CRC. Las 9 pruebas de cinemática pasan igual en el robot y en la PC.
- El servidor de cámara publica ahora el extrínseco de fábrica profundidad → color: el color está 15.2 mm a
  la derecha del IR izquierdo y girado ~1°.

## 3. Gemelo y cámara virtual

| Comprobación | Resultado |
| --- | --- |
| FK de pinocchio frente a MuJoCo (200 q al azar por brazo) | < 0.1 mm y < 1e-4 en la rotación |
| Proyección de la cámara virtual frente al pinhole | ≤ 0.14 px tras corregir el punto principal (ver abajo) |
| Profundidad virtual | Planar (z), como la D435i |
| Cierre: medir la mesa sobre la imagen virtual que se construyó con esa mesa | 0.6 mm de origen, −0.14° de yaw, 0.5 mm de plano |

- **MuJoCo da el punto principal con el signo cambiado y en coordenadas continuas**: con `principal_pixel =
  (cx − W/2, cy − H/2)` la imagen salía desplazada (−3.1, +6.8) px y los ArUco medían 2 mm de menos. Lo
  correcto es `(W/2 − (cx + 0.5), H/2 − (cy + 0.5))` (prueba `test_camara_virtual_es_pinhole`).
- Para que el detector lea los ArUco virtuales hace falta el margen blanco real (20 % del lado) y que las
  asas se apoyen fuera del labio (al principio una bajada tapaba el ArUco 3).

## 4. Calibración cámara–torso, sobre el gemelo

`herramientas/poses_calibracion.py`: 16 poses por mano con el dorso a 13–30 cm sobre la mesa mirando a la
cámara con inclinaciones distintas, sin colisiones (2 cm) y con el marcador entero en la imagen; ordenadas
para que el salto máximo entre dos seguidas sea de 37° (izquierda) y 77° (derecha). Se descartan las
soluciones a más de 100° del reposo (la muñeca girada media vuelta daba saltos de 225°).

`generar_sintetico.py calibracion` con una cámara "verdadera" desviada del URDF (0.6, −1.2, 0.4°; 6, −4, 8 mm),
marcadores del dorso desviados 7–12° y 5 mm, 1 mrad de ruido en las juntas y 1 mm en la profundidad, y
`calibrar_camara_torso.py` partiendo del URDF:

| | Error |
| --- | --- |
| Cámara | 1.3 mm y 0.07° |
| Dorso izquierdo / derecho | 1.5 mm, 0.9° / 0.6 mm, 0.7° |
| Residuo | 0.66 px de media (1.24 máx.); 0.8 mm (1.7 máx.) del centro del marcador a su rayo |
| Kabsch de los centros (la pista del PDF) frente al ajuste completo | 8 mm y 0.4°: peor, por el error de solvePnP a lo largo del rayo |

El residuo en mm se mide **al rayo** del centro detectado: comparado con la posición de solvePnP salían
6.7 mm, que son el error de solvePnP en profundidad con un marcador de 5 cm a 0.6 m, no de la calibración.

## 5. Percepción

`generar_sintetico.py objetos` (60 escenas, 179 objetos al azar en la zona, ruido de 2 mm en la profundidad) y
`evaluar_percepcion.py`:

| Objeto | Reconocidos | Posición (media / p95 / máx.) | Yaw (media / máx.) |
| --- | --- | --- | --- |
| Pelota de tenis | 100 % | 0.7 / 0.8 / 0.9 mm | — |
| Cubo | 100 % | 0.8 / 1.2 / 1.4 mm | 0.4 / 1.9° |
| Prisma | 100 % | 0.8 / 1.3 / 1.4 mm | 0.3 / 0.7° |
| Cilindro | 100 % | 2.3 / 2.7 / 2.9 mm | — |

21.5 ms de mediana por fotograma en la PC (falta medirlo en el PC2). Sobre la captura real del 2026-10-09
(`tests/datos/captura_20261009_objetos`): la pelota y el cubo se reconocen; la bola roja de espuma (fuera del
reto) y el borde de una bandeja salen como "desconocido".

Lo que cambió al probar con datos reales:

- **Los bordes de profundidad** dejan puntos a media altura alrededor de cada objeto (+2 cm por lado) que
  unían el cubo y la pelota, casi en contacto: se agrupa por encima de 2 cm (todos los objetos miden ≥ 4) y la
  planta se mide con la mitad superior.
- **Color y tamaño a la vez**: por color solo, la bola roja daba "prisma" y el borde de la bandeja azul,
  "cilindro".
- **Rangos de color**: el naranja renderizado sale en H ≈ 20 y el amarillo en H ≈ 31 (OpenCV); quedan en 3–24
  y 25–42. Hay que ajustarlos con imágenes reales en el Hito 2, sobre todo naranja frente a rojo.
- **Esferas**: la profundidad las ve 2 cm más bajas; el centro sale ajustando una esfera de radio conocido
  apoyada en la mesa (prueba con solo el casquete visible: < 2 mm).

Dianas del nivel 1 (`manip/dianas.py`) en el gemelo: 4 de 4, a ≤ 1.4 mm, sin confundir el cubo vecino.

## 6. Alcance, orientación de la mano y montaje de las bandejas

- **Giro en planta de la mano con la palma abajo** (IK, centro de la zona): el ancho de la mano (meñique →
  índice) solo llega a **+90…+165°** con la derecha y a **−150…−90°** con la izquierda. Un prisma tiene
  simetría de 180°: con la mano de su bandeja se coge desde arriba en ~80 % de sus orientaciones (hasta 35° de
  desalineación: a 35°, 4 cm de ancho son 4.9 cm de cuerda). Para el resto hace falta cambiar de agarre:
  la otra mano lo deja en la mesa girado y la mano buena lo coge (pendiente, nivel 3–4; también la base del
  giro de 90° del nivel 4).
- **El agarre de lado** sirve para el cilindro de pie; para el prisma tumbado choca con mesa, bandejas o el
  torso en casi todas las posiciones.
- **Las bandejas, muy cerca de la zona**: la mano roza las paredes de 4 cm al coger cerca de los bordes; el
  planificador lo detecta y elige otro agarre u otro objeto antes.

## 7. Reposo y pose de observación (seguridad)

En reposo (peso 0, brazos colgando) **la yema queda 17 cm por debajo del tablero y a 12 cm de su borde**. Subir
el antebrazo desde ahí barre el borde de la mesa: la yema describe un arco de ~54 cm de radio alrededor del
codo y, entre unos 27° y 39° bajo la horizontal, está entre x = 0.34 y 0.40 m por debajo del tablero. Ninguna
postura alcanzable por IK con una sola parada lo evitaba (rejillas de 162 y 432 candidatas).

Por eso **cada tirada pasa por una pose de observación**: reposo → observación (los dos brazos a la vez y en
espejo, para que el centro de masas no se vaya a un lado) → trabajo → observación → reposo → peso a 0. La
elige `herramientas/poses_brazos.py` en el gemelo: tramo limpio desde el reposo, tramos limpios a las esquinas
y al centro de la zona, y el brazo sin tapar la zona en la imagen. Con el primer criterio (la mano fuera de la
imagen) salía con los brazos en alto, por encima de los hombros; con el segundo:

| Brazo | q (rad, orden de arm_sdk) | Dónde queda la yema |
| --- | --- | --- |
| Izquierdo | (0.62, 0.46, 1.33, −0.51, −1.80, 0.09, 0.26) | (0.08, 0.72, 0.46) m del torso: al costado, a la altura del hombro, 59 cm sobre la mesa |
| Derecho | el espejo (0.62, −0.46, −1.33, −0.51, 1.80, 0.09, −0.26) | (0.08, −0.72, 0.46) |

Los brazos se abren al costado y suben (los dos a la vez, simétricos); codo y muñeca se mueven 60 cm en total
desde el reposo y el salto articular mayor es de 103° (la muñeca: ≥ 8 s y ≤ 0.5 rad/s dan unos 16 s, ×2 en las
primeras tiradas). Las dos búsquedas (3000 posturas al azar cada una; 140 y 135 válidas) eligieron la misma:
lo que manda es el camino limpio desde el reposo y hasta la zona, no la imagen. Hay alternativas con la yema
más baja (45 cm sobre la mesa, 62 cm de desplazamiento) en `config/poses_brazos.yaml`.

Hay que probarla en el robot (el Hito 3 lo pide: un movimiento real a la pose de observación y vuelta) y mirar
los pasos y el giro que provoca.

**La salida segura amplía la del PDF**: congelar, abrir la mano, subir a la altura de seguridad, ir a la
observación (si el tramo pasa la comprobación) y al reposo (solo desde la observación), y entonces bajar el
peso. Con el peso en 0 la política devuelve el brazo a su postura por donde quiera (lo advierte el PDF) y,
desde encima de la mesa, eso la barre. Se quita con `supervisor.salida_por_observacion: false`.

## 8. Planificación y simulación de punta a punta

- Cada plan se comprueba en el gemelo tramo a tramo (lineal en articulares, como lo ejecuta `brazo.py`):
  límites, **altura de seguridad de 15 cm sobre la mesa salvo en el descenso final** (los puntos altos van a
  18 cm porque entre ellos el brazo bajaba ~1–4 cm), colisiones con 2 cm de holgura (5 mm en el descenso y el
  agarre) y el objeto que se lleva contra bandejas y mesa.
- **IK**: con la solución en el borde de una banda de orientación, el término de postura la hacía oscilar y
  no convergía (0 de 13 semillas desde la pose de observación); resolviendo con la banda 0.02 más estrecha y
  sin término de postura, 6 de 13.
- **Nivel 1** (mundo simulado): 6 de 6 dianas tocadas, también en las esquinas de la zona, con el contacto a
  ~130 gf (umbral 120, máximo del PDF 300). El índice va primero a 65° bajo la horizontal: a 45° el resto de la
  mano rozaba la mesa en las esquinas. El descenso sigue hasta 2 cm bajo la mesa estimada y para al notarla.
- **Nivel 2** (mundo simulado): cilindro → B, pelota → A y cubo → C, con el agarre comprobado antes de levantar;
  el prisma girado 57° → D no tiene plan con la mano derecha (sección 6).
- **Salida segura** (simulada): un giro de 6° del cuerpo al cerrar la mano la dispara; el peso acaba en 0.
- 39 pruebas automáticas en la PC (`python3 -m pytest tests/`), incluidas cuatro de punta a punta.

## 9. Visor de la simulación (2026-10-13)

`manipulacion.py --sim --visor` dibuja el mundo simulado en una ventana de MuJoCo a velocidad real (o
`--velocidad N`), con el estado, el peso de arm_sdk y los choques del brazo o la mano contra mesa, bandejas y
suelo (puntos rojos y resumen al final). `--parada-en ESTADO+s` provoca la salida segura; `--salida-directa`
quita la salida por la observación. Sin `--colocacion`, los cuatro objetos en la zona (cilindro (0.42, 0.01),
tenis (0.45, 0.10), cubo (0.43, −0.07, 0.3 rad), prisma (0.50, −0.04, 90°)): los cuatro a su bandeja al primer
intento. Con el reloj real salieron dos fallos que el tiempo acelerado escondía:

- **Lectura de la mano en el transporte**: `sujetar()` leía la mano cada 1 s y el vigilante exige una lectura
  de menos de 0.5 s de la mano en uso → en el robot, "mano sin respuesta" y salida segura en cada transporte.
  Ahora lee cada 0.2 s (y reafirma FORCE_SET cada 1 s).
- **Salida segura a mitad del transporte**: el tramo directo a la observación metía el meñique en la mesa
  (−26 mm) o en la bandeja; el brazo se quedaba encima de la mesa y, al bajar el peso, la mano atravesaba el
  tablero. Ahora, si el tramo directo no pasa la comprobación, sale por el punto alto del plan en curso más
  cercano (sobre el objeto, la bandeja o la diana). Parada provocada en 9 momentos de una tirada (entrada a la
  observación, mover, descenso, cerrar, transporte a 1/3/6 s, soltar, volver): **ningún choque**, los dos
  brazos a la observación y al reposo antes de bajar el peso.
- **Comparación con la salida del PDF** (`--salida-directa`, peso a 0 tras subir): parada en el transporte o
  en el descenso → la mano entra **58–59 mm en la mesa y 10–14 mm en la bandeja** al bajar el peso. Ojo: el
  robot simulado devuelve el brazo a su postura en línea recta en articulares; la política real lo devuelve
  "por donde quiera", que no se conoce. Lo que se ve es el orden de magnitud del riesgo, no la trayectoria.
