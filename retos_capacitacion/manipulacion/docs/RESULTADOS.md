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
- 41 pruebas automáticas en la PC (`python3 -m pytest tests/`), incluidas cinco de punta a punta.

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

## 10. Tiempos de una tirada y orden por brazo (2026-10-14)

- **La escala de tiempo se aplicaba dos veces**: el planificador ya daba la duración multiplicada por la
  escala (8 s × 2 = 16 s) y `brazo.py` la volvía a multiplicar → tramos de 32 s. Ahora el planificador da la
  duración base y solo `brazo.py` aplica la escala. Nivel 2 con los cuatro objetos, escala 2 (medido en el
  gemelo, tramo a tramo): **19.4 → 10.0 min** (brazo 9.4 min, rampas de peso 17 s, manos 18 s). Cada objeto son
  8 tramos de brazo (sobre, aproximar, agarrar, levantar, sobre la bandeja, bajar, retirar, observación) y casi
  todos se quedan en el mínimo: con la escala 1 (si la curva de pasos lo permite) serían unos 5 min.
- **Orden por brazo** (decisión del usuario, opción b): los dos brazos van juntos a la observación, como antes,
  y el supervisor hace todos los objetos de un brazo antes de pasar al otro (el que empieza es el del objeto
  más cercano; si ninguno del brazo en curso tiene plan, pasa al otro). Mismo tiempo que intercalando (cada
  objeto empieza y acaba en la observación) y sin la asimetría de tener un solo brazo abierto al costado. El
  esquema de un brazo cada vez con el otro en reposo añadía dos tramos reposo ↔ observación y cargaba el cuerpo
  hacia un lado. Las dianas del nivel 1 se agrupan igual (por el lado de la mesa).


## 11. Comparación de ganancias con rt/arm_sdk: la marcha da pasos (2026-10-15)

- `comparar_ganancias.py --ganancias tuned` en el robot de pie (FSM 201, sin mesa), 11 intentos
  (`datos/ganancias_20261015_*`). Ninguno pasó del tramo 4 de 34: el vigilante paró 8 por un paso (una
  rodilla 6–9° fuera de su postura) y 3 por la FSM sin leer. **En 2 el paso llegó durante la rampa de peso,
  antes de mover el brazo**, y el usuario vio pasos con los brazos quietos en su postura por defecto.
- La pose de observación lleva los brazos atrás (CdM −0.8 cm) y la marcha se echaba ~1° atrás; al adelantar
  la mano izquierda volvía a +1.8° y daba el paso. El error de los brazos con `tuned`: 3–5° (codo derecho).
- Lo que manda el control interno en FSM 201 (`rt/lowcmd` escuchado, sin publicar): cadera 250/6.5,
  rodilla 400/10, tobillo pitch 80/1.5, tobillo roll 40/0.4, cintura 600/6; hombro 250/6 y 400/10, codo 80/2,
  muñecas 20/0.5; `tau` 0 (el par sale de desplazar la consigna: rodilla mandada 28°, medida 36.5°).
- Decisión (usuario e instructor): `rt/lowcmd` de cuerpo entero en el arnés. Ver el PLAN, "Cambio a
  rt/lowcmd".

## 12. De pie por rt/lowcmd: evaluación en simulación (2026-10-15)

Scripts y figuras de la evaluación en `datos/evaluacion_lowcmd_20261015/`; el controlador final, en
`manip/cuerpo.py` con `sim/cuerpo_simulado.py` y `tests/test_cuerpo.py`.

- **Piernas en PD fijo, sin apoyo:** caen en 1–2 s con cualquier postura (la de FSM 201, nivelada o 4°
  atrás) y cualquier juego de ganancias (`legs_hold` 300/140, el interno 250–400/80). Es un péndulo
  invertido: los tobillos dan 2 × 80–140 N m/rad y harían falta más de m·g·h ≈ 600. Sin par de sostén
  (feedforward) además las rodillas ceden ~10°.
- **Centro de masas:** 68.7 kg; el apoyo va de −8.5 a +17.4 cm desde los tobillos y el CdM queda a +5.6 cm.
  Los brazos lo mueven −0.8 (observación) … +2.1 cm (brazos a 0): ≥ 9.7 cm del borde en todas las poses.
  Inclinar el torso atrás no hace falta y lo acerca al talón.
- **Par de sostén** (pinocchio, `manip/dinamica.py`, con la IMU): de pie, rodillas −43/−48 N m y tobillos −12;
  el robot real midió en FSM 201 rodillas −47…−60 y tobillos ~−10. La carga de los pies estimada del par
  medido sale 1.22 veces el peso en los registros de FSM 201: escala 0.82 en la configuración, a recalibrar.
- **Masas de los brazos** (`config/gravedad_brazos.yaml`): las identificadas en `h1_2_joint_control` (otro
  URDF, mismos cuerpos de brazo y otras manos) con la mano agrupada en `wrist_yaw` reproducen el ajuste sobre
  nuestro URDF: rms 0.337 (izq) y 0.360 N m (der), frente a 2.57/2.25 con el URDF. Mano + muñeca: 1.14/1.07 kg.
- **Estabilizador** (par de los tobillos = kp × inclinación + kd × velocidad angular de la IMU, sobre la
  referencia tomada al encenderlo). Robustez frente al lazo (500 Hz, 100 Hz, al azar 2–20 ms, pausas de 50 ms
  el 1 % de los ciclos) y a un empujón de 15 N durante 0.3 s en el torso, peor inclinación en grados
  (aflojar la cuerda / brazo adelante y atrás / empujón):

  | kp / kd | 500 Hz | 100 Hz | al azar | pausas |
  | --- | --- | --- | --- | --- |
  | 300 / 10 | 2.8 / 2.6 / 3.8 | 1.8 / 1.6 / 3.7 | 1.6 / 1.6 / 3.9 | 2.9 / 3.0 / 3.7 |
  | **300 / 15** | 2.4 / 2.0 / 4.5 | 1.1 / 0.8 / 2.9 | 1.1 / 0.7 / 2.9 | 2.5 / 2.2 / 4.7 |
  | 300 / 20 | 2.1 / 1.8 / 4.3 | 0.8 / 0.5 / 2.9 | 0.8 / 0.5 / 2.9 | 2.1 / 2.0 / 4.4 |
  | 300 / 30 | 1.8 / 1.8 / **14.3** | 12.0 (cuerda) | 12.9 (cuerda) | 15.0 |
  | 350 / 20 | 1.7 / 1.6 / 9.8 | 9.0 | 5.8 | 9.8 |
  | 250 / 15 · 350 / 15 · 200 / 20 | ≤ 5.0 | ≤ 3.6 | ≤ 3.8 | ≤ 7.4 |

  Se queda **300 / 15**: kd alto oscila en cuanto el lazo se retrasa o se empuja el robot. En el ensayo de
  `arnes.py --sim` con kd 30 y el lazo a 22 ms (simulación y control en el mismo proceso) el estabilizador se
  apagó por 10°; con kd 15, no.
- **Dos errores encontrados por el camino** (y corregidos): (1) el par de sostén mezclado con la carga
  *medida* de los pies se realimenta (el par de las piernas mueve la carga estimada) y el robot rebota: la
  mezcla colgado/de pie es una rampa que manda el procedimiento; (2) calcular el par de sostén con la
  inclinación instantánea anula la gravedad que necesita el estabilizador y deja el conjunto casi sin
  amortiguar (ζ ≈ 0.08): se calcula con la IMU filtrada 2 s o la referencia del estabilizador.
- **Orden del apoyo:** si el par pasa a "de pie" antes de encender el estabilizador, las piernas empujan,
  aflojan la cuerda solas y el robot se balancea ±15° sobre ella. Encender primero el estabilizador (con la
  cuerda aún llevando 20–50 %) y después el par de pie: transición suave, la cuerda queda floja sola.
- **Colgado sin tocar el suelo:** con la secuencia de brazos el torso se inclina 8° y se mueve 12 cm.

## 13. Brazos a 0 y distancia del tablero (2026-10-15)

- Con las 14 juntas de los brazos a 0 el codo queda doblado 90°: antebrazos horizontales, muñeca a
  (0.23, ±0.21, +0.10) y yema a (0.53, ±0.20, +0.13) en el torso, **25 cm sobre el tablero**. En el reposo de
  FSM 201 la yema queda 18 cm *bajo* el tablero.
- Holgura robot–carrito con los brazos a 0 según el borde cercano x0: lo que limita es el muslo
  (`hip_roll_link`) contra el tablero: −1.7 cm a 0.20, +0.3 a 0.22, +2.2 a 0.24, +4.2 a 0.26, libre desde
  0.28. Propuesta y confirmada: **x0 ≈ 0.27** (d ≈ 14 cm del pecho, ~3 cm por delante de la punta de los pies).
- Mapa de alcance (IK palma abajo ±35°, sin colisiones) y vista del color (640×480, margen de 15 px) sobre el
  plano de la mesa (`mapa_alcance.png`): el color ve desde x ≈ 0.40 y el agarre desde arriba llega a x ≈ 0.56
  con |y| ≤ 0.10. Zona de la guía (40 × 25 a 7–32 cm del borde): con x0 0.27 o 0.33, el 75 % se ve y se
  alcanza a la vez. Girar la cintura 20° no amplía el alcance hacia delante. Las cuatro bandejas de la guía
  (±0.30, a 12 y 30 cm del borde) se alcanzan para soltar; las cercanas solo se ven al 40 %.
- Con los brazos a 0 las manos tapan en la imagen las bandejas lejanas y los lados (`vista_x026.png`): sigue
  haciendo falta una pose de observación.

## 14. Primera sesión con rt/lowcmd, colgado (2026-10-16)

`datos/arnes_20261016_091837/` (`arnes.py --solo-colgado --pruebas`, 200 s; análisis con el ciclo a 100 Hz):

- **Debug:** con el robot en amortiguación CheckMode daba `'ai'`; `ReleaseMode()` al primer intento → `''`,
  y 3 s sin nadie en `rt/lowcmd`.
- **Lazo:** 100 085 ciclos en 200 s (500 Hz), dt máximo 6.1 ms; lowstate siempre de hace ≤ 1.7 ms;
  temperatura 48 °C sin cambios.
- **Seguimiento colgado** (postura de pie, brazos a 0): error ≤ 1.2° en las piernas y ≤ 1.8° en los brazos
  (codo derecho). Con `tuned_gff` el par de los brazos lo pone casi todo el feedforward: hombro −5.5 de
  −5.8 N m, codo −7.4 de −8.7.
- **Piernas colgando:** el par medido es mayor que el de gravedad del modelo (rodillas +6.9 frente a
  +2.9 N m; cadera roll derecha −3.8 frente a +1.6) y el PD pone el resto con < 1° de error. Por eso,
  colgado, la carga estimada de los pies sale −32 N por pie (−9 % del peso): ahora se toma la **tara**
  colgado (`Cuerpo.tarar()`), y `ciclo.csv` guarda la carga sin tara ni escala (`fzc_*`) para recalibrar la
  escala con la cuerda floja.
- **IMU colgado:** pitch +3.5° con los brazos a 0 (+2.2° con los brazos colgando), roll +0.5°.
- **L2+B en Debug:** el programa lo vio en `wireless_remote` y CheckMode siguió en `''`: el robot no vuelve
  a la marcha y no se vio ningún efecto propio de L2+B (las juntas hicieron lo mismo que en la prueba de
  silencio). Ahora L2+B lo atiende el hilo de control: kp a 0 en 0.5 s y queda kd, como la amortiguación
  fuera de Debug.
- **Sin publicar:** los motores mantienen la última orden ~0.2 s y luego quedan **sin par** (el `tau_est`
  se congela): las juntas cargadas por la gravedad (cadera pitch y rodilla) caen a ~10°/s, frenadas solo por
  la fricción de la reductora; colgado, 6–8° en 1–2 s hasta quedarse quietas. Consecuencia: si el programa
  muere de pie, el robot cae en la cuerda (holgura 2–3 cm). Un corte del SSH ya no lo mata (SIGHUP: parada
  y, colgado, termina solo).
- **La [f] no se reconoció:** llegó como 'F' y, al revés, Mayús+F llegó como 'f': casi seguro Bloq Mayús
  activo desde COLGADO/SOLTAR. Las teclas ya no distinguen mayúsculas y cada una queda en `eventos.csv`.

## 15. De pie en el arnés por primera vez (2026-10-16)

`datos/arnes_20261016_102119/` (`arnes.py` completo, 330 s). Tara colgado −42/−49 N. El operador bajó hasta
65 %, pulsó `[e]`, aflojó la cuerda unos centímetros, `[m]`, `[c]`, subió el polipasto y `[f]`.

- **Apoyo:** con el par de colgado y el 65 % en los pies las rodillas cedían 4.8° (44.3° frente a 39.5°).
  Referencia del estabilizador: roll +0.7°, pitch +2.4°. Al pasar el par a "de pie" las piernas se
  estiraron y la carga subió sola al 102 % (la cuerda se descargó), como en la simulación.
- **Cuerda floja:** la carga marcó 1.09 estable (1.20 veces el peso sin tara ni escala, 1.33 con la tara):
  **escala 0.82 → 0.75**. La simulación pasa a un sesgo de 1.33 en el par de las piernas.
- **Inclinación:** al aflojar la cuerda el torso se quedó **3.4° adelante de la referencia** (IMU +5.7°),
  roll −0.35°, sin oscilar (giróscopo 0.6°/s de desviación típica). Tobillos: +26 N m mandados por tobillo
  (18 del estabilizador, 8 del par de sostén) más ~6.5 del PD → el centro de presiones ~9 cm por delante del
  tobillo (la punta está a 17.4).
- **Brazos adelante y atrás (`[m]`):** pitch respecto de la referencia entre 3.33 y 3.49°, roll ±0.05°: los
  brazos no mueven el torso.
- **Lazo:** dt máximo 7.6 ms, lowstate de hace ≤ 3.4 ms, 49–51 °C. Rodillas ≤ 1.1° de error de pie; tobillos
  pitch −2.7/−1.3° (los dobla el par del estabilizador).
- **Colgar:** al subir el polipasto el estabilizador se apagó solo al 34 % y el par volvió a colgado al 10 %.
- **Por qué se inclina:** el estabilizador es proporcional y la rigidez neta es pequeña (2 × 300 del
  estabilizador + 2 × 140 del PD del tobillo − m·g·h ≈ 600 → ~280 N m/rad): ~8 N m por tobillo que le falten
  al modelo de pie dejan 3–4°. La simulación lo reproduce al soltar la cuerda (2.2°). Arreglo: **integral**
  (Ki 30 N m/(rad s), ±20 N m por tobillo) que solo integra de pie del todo (carga ≥ 0.9, par de pie y
  estabilizador al 100 %) y se mantiene si no. Simulado con el centro de masas del torso 3 cm adelante
  (estacionario / brazo adelante / empujón de 15 N 0.3 s, grados respecto de la referencia; lazo a 500 Hz,
  al azar 2–20 ms, pausas de 50 ms):

  | | estacionario | brazo | empujón |
  | --- | --- | --- | --- |
  | proporcional | +2.0 / −2.2 / +2.1 | 2.1 / 2.4 / 2.3 | 4.7 / 4.0 / 4.5 |
  | **Ki 30** | −0.1 / ±1.1 / −0.1 | 0.4 / 0.9 / 0.4 | 2.5 / 2.7 / 2.8 |

  Con Ki 120 el robot oscila despacio (±3°); con 60, bien pero el brazo lo mueve 0.7–1.5°. Con el lazo al
  azar queda la oscilación de ±1° que ya tenía el proporcional.

## 16. La integral y la referencia de inclinación (2026-10-16)

`datos/arnes_20261016_105235/` (`arnes.py`; el robot seguía en Debug de la tirada anterior). Tara colgado
−30/−52 N; `[e]` con la carga en 67 %; se aflojó la cuerda; sin `[m]`; fotos `montajes/h1_2_robot_inclinado.jpeg`
y `h1_2_medidor_nivel.jpeg` al final (el nivel del móvil en el torso marcaba ~4°).

- **La integral funcionó:** el pitch respecto de la referencia pasó de +1.3° a ~0 en ~30 s y se quedó ahí sin
  oscilar (int_pitch +11…+16 N m mientras la cuerda aún tiraba, +5…+6 al final).
- **Pero la referencia estaba inclinada:** al bajar el polipasto con el par de colgado las piernas ceden (tobillos
  de −21.2° a −29.3°) y el torso pasa de +3.1° a +6.2°; al pulsar `[e]` la referencia quedó en **+5.3°** (en la
  sesión anterior, +2.4°). De pie quedó con la IMU en +5.4°; el móvil decía ~4°: la IMU y el nivel coinciden en
  ~1.5°.
- **La postura de verdad:** de pie en FSM 201 (11 tiradas del 2026-10-15) la IMU daba **+0.5°** (−0.5…+0.9) con
  los tobillos a −21.8°, la misma postura de piernas que se manda. El URDF, con esa postura y la suela plana, da
  el torso a +5.9°: **~5.4° entre el modelo y el robot** (articulaciones o geometría; la IMU coincide con el nivel).
- **Arreglo:** referencia fija. `[n]` (de pie, con la cuerda floja) lleva la referencia, desde la inclinación
  medida, a +0.5° a 0.5°/s y solo entonces actúa la integral; `[c]` la deja quieta. Ya no se decide con la carga
  medida, que se movía ±10 % con el par de los tobillos (1.04 → 0.91 al bajar la integral de 16 a 5 N m).
- **Simulación:** `sim/cuerpo_simulado.py` lleva ahora el desfase como 5.4° en los tobillos (el robot dice
  q − 5.4°) y reproduce lo del robot: al 60 % el torso a +6.9° con los tobillos a −29.6°. Con `[n]` a 500 Hz el
  torso queda en 0.0 ± 0.1°, el brazo lo mueve 0.6° y el empujón 2.4°. Con el desfase la transición de `[e]` es
  peor que en el robot (pico de 7–8° frente a 1.1–1.3° en las dos sesiones reales) y con el lazo a 100 Hz o al azar
  2–20 ms el robot simulado cae en la cuerda en esa transición; con 2–8 ms aguanta (el real: dt máximo 7.6 ms).

## 17. Oscilación al nivelar y procedimiento con la cuerda llevando parte del peso (2026-10-16)

`datos/arnes_20261016_113005/` (`[n]` con la cuerda floja) y `datos/arnes_20261017_014902/` (la cuerda
tensada después, ~25 % del peso; el reloj del robot dice 17).

- **Oscilación:** tras `[e]` el robot quedó en +7.8° (al modelo de pie le faltan ~13 N m de pitch por tobillo:
  el robot se inclina hasta que el estabilizador los pone con el error). `[n]` bajó la referencia a +0.5° en
  15 s, más deprisa de lo que el robot la seguía: los tobillos llegaron a **54 N m (el motor da 60)**, el torso
  se quedaba atascado ~10 s a +6° y saltaba a −5°, con la integral saturada; ~110 s hasta asentarse (pitch
  −5.1…+8.2°, giróscopo hasta 20°/s). Asentado y con la cuerda floja: +0.5° ± 0.05°, tobillos 9–16 N m,
  integral +13.4 N m de pitch y −5.7 de roll.
- **Con la cuerda llevando ~25 %** (pies 74–83 %): quieto a +0.8° (giróscopo < 1.3°/s durante 7 min). La
  integral llegó al tope (+20) en pitch y en roll: el gancho no estaba encima de la cinta y la integral peleaba
  contra la cuerda; en roll además no sirve (el torso lo sostienen las dos piernas, el par del tobillo casi no
  lo mueve).
- **Dónde está el desfase de ~5.4° URDF–robot:** con el mismo desfase, el par de pitch que le falta al modelo en
  cada tobillo sería +20.6 N m si está en la cadera, +11.2 en la rodilla y 0 en el tobillo; el robot pidió 13–20.
  La simulación lo lleva ahora repartido (2.7° en cadera y 2.7° en rodilla) y **reproduce el fallo**: con lo de
  entonces y la cuerda floja, el tobillo llega a 60 N m y el robot se queda en +12° apoyado en la cuerda.
- **Cambios** (decisión del usuario: procedimiento de trabajo con la cuerda llevando parte del peso): sesgo de
  +13 N m de pitch por tobillo en el par de pie; la referencia avanza (0.3°/s) solo con el torso a menos de 1°
  de ella; la integral no acumula si el estabilizador está saturado, ni dentro de ±0.5°, ni en roll, con tope de
  15 N m; el total del tobillo (sostén + estabilizador) ≤ 50 N m de pitch y 30 de roll; `[n]` y `[m]` con los
  pies ≥ 60 % (el procedimiento pide 70–85 %) y un aviso en pantalla cuando la integral está al tope (mover el
  gancho). Simulado (500 Hz salvo donde se dice; máximo error respecto de la referencia en grados y par máximo
  del tobillo):

  | Caso | tras `[e]` | `[n]` | brazo | empujón 15 N | torso nivelado |
  | --- | --- | --- | --- | --- | --- |
  | cuerda floja | 1.1 (26 N m) | 1.1 (25) | 0.4 (17) | 2.9 (29) | +0.3 |
  | cuerda ~25 % | 1.1 (26) | 1.0 (15) | 0.5 (14) | 1.8 (21) | +1.0 |
  | ~25 %, gancho 5 cm detrás | 4.6 (37) | 1.8 (34) | 1.0 (26) | 1.8 (31) | +2.2 (integral al tope) |
  | ~25 %, gancho 5 cm delante | 3.2 (19) | 1.1 (4) | 0.5 (4) | 1.2 (8) | 0.0 |
  | ~25 %, lazo 2–8 ms | 2.4 (37) | 1.4 (25) | 0.5 (14) | 1.8 (21) | +1.0 |
  | cuerda floja, como antes | 3.2 (29) | 11.9 (**60**) | 11.8 (49) | 11.9 (49) | +12.3 (en la cuerda) |

## 18. El procedimiento con la cuerda en el robot (2026-10-17)

`datos/arnes_20261017_030022/` y `datos/arnes_20261017_031131/`: `[e]`, polipasto hasta ~79 % y ~77 % en los
pies, `[n]`; en las dos salió "integral al tope" y el operador movió el pórtico hasta dejar la cuerda vertical.

- **Tras `[e]`, con el sesgo del tobillo:** el torso se quedó en la referencia (−0.2° en la primera; antes,
  +3.4° y +1.3…+7.8°). Sin oscilación en ninguna de las dos.
- **`[n]` con la cuerda ya tensa:** la referencia solo bajó hasta donde el torso la seguía (+0.6…+0.7°) y el
  torso se quedó en +1.6…+1.7° con la integral al tope (+15): la cuerda lo sujetaba adelante. Al **retrasar** el
  pórtico el torso pasó a +0.3…+0.4° (el aviso decía "adelanta": estaba al revés; la simulación lo daba al revés
  y manda el robot). La integral bajó luego hasta quedarse en la banda (torso a 0.0…+0.1°, 0.45° por debajo
  del objetivo: banda de 0.5 → 0.25).
- **Por qué salía aunque se centrara el pórtico antes de `[n]`:** al nivelar, el torso se echa atrás 2–4° y la
  cinta se mueve ~5 cm. Orden nuevo: `[n]` con la cuerda descargada por las piernas, luego centrar el pórtico y
  luego subir el polipasto (en la pantalla, al terminar de nivelar).
- **Con la cuerda vertical y los pies al 73–79 %:** quieto; con `[m]` (brazos adelante y atrás) el pitch entre
  −0.03 y +0.09°, roll ±0.05°, giróscopo 0.5°/s de desviación típica y los tobillos ≤ 9 N m (≤ 4 quieto).
- **Simulado con el orden nuevo** (centrado / gancho 5 cm desplazado / lazo 2–8 ms): `[n]` 1.4° (26 N m) /
  hasta 6° (47 N m) / 2.6° (37); con la cuerda tensa el brazo 0.24 / 0.45–2.3 / 0.25° y el empujón 1.4 / 1.1–3.1 /
  1.2°. Centrar el pórtico importa.

## 19. El orden nuevo en el robot (2026-10-17)

`datos/arnes_20261017_033121/`, `_033530/` y `_033921/`.

- **En las dos primeras `[n]` no hizo nada:** se pulsó mientras el programa esperaba a que se encendiera el
  estabilizador (1.3 s después de `[e]`) o en los 3.5 s del paso a "de pie"; esas esperas devolvían la tecla y
  la tiraban, y en la segunda además el par de pie empezó con el estabilizador al 43 %. Ahora esas esperas
  guardan las teclas para después y no se cortan. Sin nivelar el robot estuvo de pie a +3.4° con la cuerda
  descargada (tobillos ~30 N m, sin oscilar).
- **En la tercera:** nivelado de +3.4° a +0.7° en ~20 s (error ≤ 1.0°, tobillos ≤ 30 N m); "nivelado" salió a
  los 19 s pero se perdía entre las líneas de estado (ahora va entre dos líneas de `=` y la línea de estado dice
  el siguiente paso). Con el pórtico centrado, +0.47…+0.49°; al subir el polipasto (pies 80 %) +1.3° que la
  integral bajó a +0.7°. Con `[m]`: pitch +0.70…+1.03°, roll +0.05…+0.12°, tobillos ≤ 16 N m, giróscopo 0.53°/s.
  "Integral al tope" no salió.

## 20. El montaje con el robot de pie en el arnés (2026-10-17)

Robot de pie (procedimiento de trabajo, pies al 84 %, pitch +0.7°) y el carrito delante; fotos
`montajes/h1_2_arnes_manipulacion1-3.jpeg`. El carrito cabe sin tocar la base del pórtico (la punta de los pies
sobresale de los rieles) con el lado largo frente al robot; las asas quedan a los lados, a la altura de los codos.

- **Con la mesa a ~0.27 m la cámara no ve las esquinas cercanas** (ArUco 4 y 5): `manip/montaje.py` construye
  ahora el marco con los dos lejanos (borde lejano) y el fondo de la mesa de una medida anterior (0.483 m entre
  el borde cercano y los centros lejanos). `medir_montaje.py` carga siempre el `montaje.yaml` anterior.
- **Primera colocación** (`datos/dataset_20261017_043140_mesa_arnes/`): borde cercano a 0.262 m, 5.8 cm a la
  derecha, yaw −1.0°. **Centrado** (`datos/dataset_20261017_043829_mesa_arnes_centrada/`, en `montaje.yaml`):
  **0.282 m, 2.1 cm a la derecha, yaw −0.7°**, plano 12.0 cm bajo el torso (rms 2.8 mm), labio 3.9 cm, asas en
  y +0.48/−0.49 de 15/13 cm de alto a todo el fondo. Las bandejas de la guía caben (la izquierda a 2.7 cm del
  borde).
- **Gemelo contra la imagen real** (`gemelo_vs_real.png` en las dos carpetas), con la pose de la cámara del URDF y
  los brazos medidos: bordes de la mesa, ArUco, alfombrillas, asa y manos coinciden a pocos píxeles.
- La prueba de cierre del gemelo pasa a 5 mm (3.1 mm entre el marco de los ArUco cercanos y el de los lejanos
  más el fondo).


## 21. Bandejas, asignación y colocación del montaje (2026-10-17, en la PC)

Pedido del usuario: el carrito lo más cerca posible de 0.27–0.28 m, centrado y recto (el de §20 no es el
definitivo); las bandejas pueden acercarse a la línea media para alejarlas de las asas y, si se puede, que las
lejanas no las tapen los brazos a 0, idealmente las cuatro con el mismo espaciado; y un orden nuevo de objetos y
bandejas. Gemelo con la mesa medida llevada al objetivo (0.275 m, centrada, recta), el robot con la postura de pie
del arnés (`datos/arnes_20261017_041043`) y los brazos a 0. Mapas y vista en `datos/bandejas_20261017/`.

- **Con los brazos a 0 las manos tapan una franja fija** (las yemas a y ±0.20, 25 cm sobre el tablero, vistas
  desde la cabeza): una bandeja centrada en y 0.24–0.30 y x 0.45–0.57 enseña solo el 8–44 % de su fondo (peor
  en x 0.49–0.53); se ve casi entera a |y| ≤ 0.18 (dentro de la zona) o a |y| ≥ 0.36–0.38 (a 6–9 cm de las asas, y la suelta de lado ya
  no llega a las cercanas). **No hay sitio para las bandejas libre de esa sombra**: acercarlas a la línea media la
  empeora (a ±0.28 queda el 21–30 % del fondo de las lejanas, frente al 35–43 % a ±0.30). Lo resuelve la
  herramienta de colocación, que mide cada bandeja con lo que se ve (abajo), y la pose de observación pendiente.
- **Planes completos** (`Planificador.coger` con colisiones, brazos a 0 al empezar y acabar, el otro brazo a 0):
  cada objeto desde una rejilla de la zona (x 0.42/0.48/0.54, y −0.16…+0.16; el cubo con dos guiñadas y el prisma
  con tres) a cada bandeja, para cuatro rectángulos:

  | Bandejas (x, ±y) | Planes válidos desde su lado o el centro | Brazo a las asas en la suelta: mínimo / p10 |
  | --- | --- | --- |
  | (0.39 / 0.57, ±0.30), la guía | 83 % | 7.2 / 9.0 cm |
  | (0.39 / 0.57, ±0.28) | 83 % | 7.8 / 10.6 cm |
  | **(0.40 / 0.56, ±0.28), la elegida** | **83 %** | **8.1 / 10.7 cm** |
  | (0.40 / 0.56, ±0.27) | 83 % | 8.3 / 11.3 cm |
  | (0.40 / 0.56, ±0.26) | 84 % | 9.0 / 11.9 cm |

  Pelota, cubo y cilindro salen al 83–100 % hacia cualquier bandeja (94–100 % en la elegida); **el prisma es el difícil y depende del
  lado**: 74–89 % hacia las izquierdas y 37–56 % hacia las derechas (el giro de la muñeca con la palma abajo no
  es simétrico). Lo que falla desde el otro lado de la zona es el agarre (IK), no la bandeja. Con el objeto
  pegado al borde de la zona (y ±0.17), pelota, cubo y cilindro siguen al 100 %; el prisma, 50 % a ±0.28 y 47 %
  a ±0.27 (los dedos rozan la pared de la bandeja cercana al entrar de lado).
- **Elegido**: rectángulo de 16 × 56 cm centrado en la cruz (0.48, 0), con las cuatro a la misma distancia de
  ella (29 cm) y alineadas con los bordes cercano y lejano de la zona: **A azul (0.40, +0.28), B roja
  (0.56, +0.28), C blanca (0.40, −0.28), D naranja (0.56, −0.28)**. ±0.28 en vez de ±0.27: 1.7 cm entre la bandeja y
  la zona (0.7 a ±0.27) por 0.2 cm menos de holgura con las asas. Con la mesa a 0.275 los centros quedan a 12.5 y
  28.5 cm del borde cercano y la pared exterior a 6.5 cm del lateral de la superficie. La cercana pierde ~10 % por
  abajo de la imagen (a 0.39, ~20 %).
- **Colores**: con la asignación del usuario (cilindro → roja, pelota → blanca, cubo → naranja, prisma → azul),
  la azul va a la izquierda (prisma al 89 % hacia A frente a 37–44 % a la derecha), el cubo fuera de B (94 %) y la
  roja y la naranja en lados distintos (tonos cercanos: la herramienta busca cada color a ±12 cm de su sitio).
  Pares asignados desde su lado: prisma → A 89 %, cilindro → B 100 %, pelota → C 100 %, cubo → D 100 %.
- **Nivel 2 de punta a punta** (`manipulacion.py --nivel 2 --sim`, con la colocación de prueba repartida por lados
  como pide el nivel 2): los cuatro a su bandeja al primer intento, primero los de la izquierda.
- `objetos.yaml` lleva la bandeja de cada objeto por color y `manipulacion.py` acepta color o nombre en
  `--asignacion`. La percepción no mira dentro de las bandejas (+1 cm): sus paredes caen en la zona ampliada.

**`herramientas/marcar_montaje.py`** (el previsto en el PLAN): en el robot de pie, con `camara_servidor.sh`, mide
la mesa en cada fotograma (plano de la profundidad y ArUco 2 y 3 con el fondo de la mesa, como `medir_montaje.py`,
mediana de 5) y dice lo que falta para `objetivo_mesa` (0.275 ± 1 cm, lateral ± 1 cm, giro ± 1°): "acerca 1.5 cm",
"2.0 cm a la derecha del robot", "aleja el extremo izquierdo 15 mm"; dentro 2 s: "MESA EN SU SITIO". `[g]` mide la
mesa vacía con 15 fotogramas y la guarda en `montaje.yaml`. Luego busca cada bandeja por su color y su altura y
ajusta un cuadrado de su medida a las paredes (h > 1.2 cm) y al fondo, así que le basta ver dos paredes. Pinta
el objetivo y lo medido en `http://<robot>:5001`.

- Sobre la grabación real centrada de §20: borde 0.283–0.285 m, lateral −1.7…−2.0 cm, giro −0.8…−1.05° por
  fotograma (`medir_montaje.py` con 15 fotogramas: 0.282, −2.1, −0.71).
- En el gemelo con desvíos conocidos (carrito +1.2 cm, −1.5 cm, +1.2°; bandejas movidas hasta 2 cm y 8°, con los
  brazos a 0 tapándolas): mesa a ≤ 1 mm y 0.1°, bandejas a ≤ 2 mm y 0.3° (`tests/test_montaje.py`).
- Los rangos de color de la roja, la azul y la blanca salen de la captura real del 2026-10-09; **la naranja no
  tiene imagen real**: hay que medirla con las cuatro bandejas en la mesa.

## 22. Montaje, objetos y escala de la cámara con el robot de pie (2026-10-18)

Robot de pie en el arnés (`arnes.py`, brazos a 0) y `camara_servidor.sh`. El usuario colocó el carrito con
`marcar_montaje.py` (tres `[g]`, vale el último: `dataset_20261018_053528_mesa_marcar`) y las bandejas con sus
indicaciones; se grabó la escena con las bandejas (`dataset_..._054851_bandejas_colocadas`) y con los cuatro
objetos (`..._060227_objetos_a`, y `..._062303_objetos_a_regla` tras medirlos con wincha). Sin mover el robot.

- **Bandejas** (estables en los 48 fotogramas): A azul (−0.4, −1.4 cm), D naranja (0, −1.1 cm) bien; B roja a
  1.8 cm a la derecha de su sitio. **La C blanca no se puede medir con los brazos a 0**: la profundidad del D435i
  no tiene dato en una franja a la izquierda de lo cercano y ahí cae su pared interior (a la izquierda de la mano
  derecha); solo se ve la exterior, que no fija el centro, y la pegatina blanca del borde derecho de la mesa
  arrastraba el ajuste 13 cm. La A tiene la pared interior a la derecha de la mano izquierda y se ve. Cambios: los
  puntos a menos de 2 cm del borde de la superficie no cuentan, y una bandeja con menos de dos lados vistos
  (15 puntos de pared) sale "solo veo una pared: colócala con regla" en vez de una indicación falsa. La C se
  colocó con regla respecto de la D (misma línea, 3.4 cm entre paredes).
- **La naranja se detecta con el rango supuesto.** Cada objeto tiene el color de una bandeja (cilindro y azul,
  prisma y naranja, cubo y blanca: mismos H); no molesta porque la percepción no mira dentro de las bandejas. El
  rango del prisma (H 3–24) toca la roja (H 1–3) y la pelota (H 24–29): ajustar con más capturas. El suelo y el
  dorso blanco de la mano caen en el rango del cubo, pero fuera de altura.
- **Percepción**: 192 de 192 reconocidos (confianza 0.71–0.98, 29–44 ms por fotograma). Alto medido: pelota
  6.2 cm, cilindro 8.5 (8), prisma 4.0 en el centro con los extremos difuminados (2–3 cm al pie de cada salto de
  profundidad), **cubo blanco 4–5.5 cm de 6** (la peor profundidad: su planta varía 85×54 → 68×38 mm entre
  fotogramas). La imagen de color termina a x ≈ 0.36 m en la mesa (la profundidad, a 0.20): un objeto en el
  borde cercano de la zona sale cortado en color. Un "desconocido" de 20 cm de alto en (0.47, 0.20) es la mano
  izquierda: `evaluar_percepcion.py` no conoce los brazos (la tirada sí).
- **La mesa medida con wincha** (`mesa_fisica` en `montaje.yaml`): 77.8 × 48.8 cm entre las caras interiores del
  labio, ArUco de 5 cm de negro con 6 mm de blanco y el blanco contra el labio: **centros a 3.1 cm de cada labio**
  (se suponían 4 cm), 71.6 cm entre los de 2 y 3. La percepción contra la wincha (centros estimados a ojo) daba
  +1.6 cm hacia delante y −2.1 cm lateral en los cuatro objetos: el lateral era el margen de 4 cm (el labio
  izquierdo quedaba 2 cm hacia fuera).
- **La cámara ve la mesa ~3 % más grande**: entre los ArUco 2 y 3, 736.5–739.7 mm en todas las grabaciones desde
  el 17 (×1.029–1.033; 5–3 ×1.020–1.040; 4–5 del 13, ×1.020). **Igual en IR y en color** (fx 379 y 604): no son los
  intrínsecos. Con esa escala la percepción coincide con la wincha a ≤ 5 mm en los cuatro objetos. Lo que no está
  claro es su origen: la altura de la mesa sobre el suelo solo con profundidad (plano del suelo − plano de la mesa)
  sale 0.858–0.869 m frente a 0.855 de wincha (+0.4…+1.7 %, con los dos planos a 1.5–1.9°); con la postura de pie
  del registro de `arnes.py` (torso a ~0.99 m de la suela) y el URDF, la mesa queda 1.3 cm más alta que la wincha;
  y a 0.6 m (las manos a 0) la separación entre manos es +0.6 % y la profundidad +1.8 % frente al gemelo (con la
  pose real de los brazos algo distinta: la cámara las ve 2.5–3 cm más bajas). Compatible con una profundidad que
  mide de más y crece con la distancia, sin poder separarla de los errores del URDF. **Para el robot: 1–1.5 cm en
  la zona**, sobre todo hacia delante. Se mide en la calibración cámara–torso con el ArUco de las manos por la zona
  (añadir la comparación de la profundidad en el marcador) o con la calibración en placa del D435i (on-chip +
  tare, que reescribe la calibración de la cámara).
- **El marco de la mesa sale ahora de la wincha y de esa escala**: `escala_mesa` = distancia 2–3 vista / 71.6 cm
  en cada fotograma, y el fondo, el margen y la superficie se escalan con ella; así queda donde la cámara ve los
  objetos y las bandejas, y vuelve a 1 sola cuando la cámara esté corregida. El `[g]` del 18 da **borde cercano a
  0.286 m** (el código anterior decía 0.274: restaba 4.83 cm de fondo, deducidos con los 4 cm), −0.7 cm, +0.77°,
  superficie 0.504 × 0.804 en la escala de la cámara. Al medir con la wincha el carrito se movió: 0.291 → 0.300 m.
  En el gemelo los ArUco de la mesa van en sus esquinas con 6 mm de blanco (los medidos caían bajo el labio).
- **Con la mesa a menos de ~0.29 m, el paso reposo (brazos colgando, FSM 201) → observación no pasa**: el índice
  izquierdo a 13 mm del borde cercano (holgura 2 cm). Ya pasaba con el `[g]` y el código anterior (0.274 m); con
  el montaje del 17 (0.282, giro −0.71°, plano 4.6 mm más bajo) pasaba justo. Desde los brazos a 0 (arnés,
  rt/lowcmd) pasa con todas las poses probadas (0.274–0.30 m). Fallan las cinco pruebas de `test_supervisor.py`,
  que arrancan del reposo de la FSM: se resuelve al llevar el supervisor al arnés (empezar y acabar con los
  brazos a 0).

## 23. Mesa recolocada y calibración en el arnés (2026-10-18)

- **Mesa** (con `desplegar.sh`, el marco de la wincha y `escala_mesa`): el usuario la llevó con
  `marcar_montaje.py` hasta "MESA EN SU SITIO" y pulsó `[g]` (`dataset_20261018_080019_mesa_marcar`): **borde
  cercano 0.273 m, +0.4 cm a la izquierda, +0.69°**, escala ×1.029. Una grabación aparte unos minutos después da
  0.272–0.273 m, +0.3/+0.4 cm, +0.7°. En `config/montaje.yaml` de la PC.
- **Poses de calibración para el arnés** (`poses_calibracion.py`, ahora solo este modo): el robot de pie con la
  postura de `cuerpo.postura_pie_deg` y los brazos a 0 delante de esa mesa, con las bandejas; cada brazo sale de
  0 y vuelve a 0 con el otro quieto en 0. Además de la IK, la holgura de 2 cm y el marcador dentro de la imagen,
  ahora se exige que **el camino** entre poses (recta articular, la de la mínima sacudida de `cuerpo.py`) no se
  acerque a nada, también desde y hasta 0, y que el detector **lea el marcador en la imagen del gemelo** (con
  los brazos a 0 la otra mano podría taparlo; no pasó en ninguna). 16 por mano: izquierda 457 con IK, 382 libres
  y leídas; derecha 511 / 364. Marcador a 13–30 cm de la mesa, salto máximo 71°; ~2.3 min por brazo sin pausas.
  `poses_calibracion.yaml` lleva `modo: arnes`, `inicio` (0) y la mesa con la que se generaron.
- **`arnes.py` [k]** (de pie, con el estabilizador y después de [n], como [m]): cada brazo por sus 16 poses y de
  vuelta a 0, 3 s quieto en cada una (las teclas no acortan la pausa; ESPACIO y el mando sí paran). No arranca si
  el yaml no es del arnés o si la mesa de `montaje.yaml` está a más de 1 cm o 1° de la de las poses.
- **`grabar_calibracion.py`**, en otra terminal (solo lee la cámara y `rt/lowstate`, fuera del lazo de 500 Hz):
  guarda color, profundidad, q medida y la pose mandada más cercana cada vez que un brazo está fuera de 0 y
  quieto 1 s (ninguna junta se mueve más de 4 mrad: el último segundo de una mínima sacudida de 8 s aún se mueve
  ~16 mrad) en una pose nueva (> 0.05 rad de la anterior). Prueba con trayectorias de mínima sacudida y ruido de
  encoder: una muestra por pose, ninguna en 0.
- **`calibrar_camara_torso.py`** compara además la profundidad en el centro del marcador con la que tiene que
  tener con la cámara calibrada: con la sintética da 0.9998 (0.9984–1.0012) a 0.55–0.73 m. En el robot dirá si
  la profundidad mide de más a la distancia de las manos (§22).
- **Con el gemelo:** la calibración sintética con las poses nuevas (el otro brazo a 0) recupera la cámara a
  0.9 mm y 0.09° y los dorsos a 1.6 mm y 0.9° (32 de 32 muestras con el marcador).
- **Con física** (`arnes.py --sim --operador-auto --con-calibracion`): de pie con la cuerda llevando ~28 %, [m] y
  luego [k] entero (6.2 min): roll ≤ 0.13°, par de tobillo ≤ 36 N m (33 en [m]), carga de los pies 0.71–0.73,
  estabilizador siempre al 100 %, sin avisos; el error de pitch máximo, 1.76° (1.86 en [m]), es casi todo el
  desvío que ya tenía la simulación con esa cuerda (integral al tope). Cuelga y termina normal.

## 24. Calibración cámara–torso en el robot (2026-10-18)

`datos/calibracion_20261018_083411`: [k] en `arnes.py` con `grabar_calibracion.py`, **32 muestras, una por pose**,
el marcador leído en todas. `config/calibracion_20261017_1944.yaml` (el nombre lleva el reloj de la PC).

- **Ajuste**: 1.57 px / 1.7 mm de media en el centro del marcador (izquierda 1.28 px, máx. 2.2; derecha 1.87 px,
  las tres últimas poses 3.4–4.7 px). Kabsch de los centros frente al ajuste completo: 7.5 mm, 0.55° (la
  dirección del eje óptico es la peor determinada).
- **Cámara respecto del URDF: 13.9 mm y 2.25°**: el eje óptico a 48.5° bajo la horizontal (50.5° en el URDF),
  −0.8° de guiñada; traslación (−9.8, −3.4, +9.3) mm en el torso.
- **Marcadores del dorso: girados ~177° en su plano** respecto de la semilla (pegados al revés); la calibración
  los estima, no molesta.
- **Seguimiento en el arnés**: q mandada − medida ≤ 0.8° de media por junta, máx. 2.9° (codo derecho).
  Ajustada con la q mandada: 8.1 mm de residuo y la cámara 41 mm movida: se calibra con la q medida.
- **Comprobación independiente** (manos a 0 del `[g]`, no usadas en el ajuste): la nube de las manos frente al
  modelo pasa de 2.2–3.0 cm de diferencia en altura (URDF) a 0.9–1.0 cm, y en la punta de 1.5–2.5 a 0.4–1.5 cm.
- **Los 2° están en los brazos, no en la cámara**: con el URDF la mesa sale con pitch +0.3/+0.5° y el suelo −0.3°,
  como la IMU (+0.41°); con la calibración, mesa +2.3/+2.6° y suelo +1.7° (roll también ~2° peor). Una mesa y un
  suelo inclinados 2° no son creíbles: el marco de la cinemática de los brazos (ceros de junta o flexión con su
  peso) está ~2° girado respecto del torso real, y la calibración lleva la cámara a ese marco. Para coger es el
  bueno (alinea lo que ve la cámara con donde la cinemática pone la mano); no es un marco "a nivel".
- **El montaje pasa al marco calibrado**: el `[g]` del 18 da 0.292 m, +1.5 cm, +0.55° (con el URDF 0.273, +0.4,
  +0.69). El paso URDF → calibrado lleva todo lo de la mesa (+1.9, +1.0) cm. `objetivo_mesa` = el 0.275/0/0 del
  usuario pasado a este marco: **0.294 m, +1.05 cm, 0°** (el mismo sitio físico; la mesa está dentro). **Las
  bandejas, colocadas con el URDF, quedan ~2 cm de su sitio** (1.9 hacia el fondo, 1.0 a la izquierda): volver a
  colocarlas con `marcar_montaje.py` cuando se despliegue.
- **Profundidad**: en el marcador de la mano, 0.9895 de la esperada (0.985–0.996) a 0.58–0.77 m, sin pendiente.
  La wincha confirma 71.6 cm entre los centros de los ArUco 2 y 3 (77.8 × 48.8 es el interior; 79.5 × 50 era
  por fuera): la cámara da 736–739 mm con el plano y **728–732 con la profundidad del propio papel a 1.01 m**
  (+1.7–2.2 %); el plano ajustado al tablero negro brillante lo aleja ~1 % más. Es decir: la profundidad mide
  ~2 % de más a 1 m y ~1 % de menos (o bien) a 0.7 m, y el tablero negro empeora el plano. En la zona (0.85–1.0 m
  de la cámara) cabe ~1–1.5 cm de error; las poses de calibración no bajan de 13 cm sobre la mesa (0.58–0.77 m).
- **Gemelo**: la cámara calibrada quedaba dentro de la visera (malla de `torso_link`, con la cabeza) y su borde
  cruzaba la imagen: tapaba la bandeja A en `test_montaje` y el marcador en 200 de las poses candidatas.
  `CamaraVirtual` ya no pinta el cuerpo que la lleva. Poses de calibración regeneradas para el montaje calibrado
  (378 y 365 válidas). `test_captura_real` usa el plano de su propia captura (no el del montaje de hoy). Con el
  montaje calibrado (mesa 1.9 cm más lejos en el marco de los brazos) **vuelven a pasar las 5 de
  `test_supervisor.py`**: 62 de 62.

## 25. Bandejas medidas, comprobación al empezar y salida más legible (2026-10-18, en la PC)

Respuesta a "¿se recalcula todo con el montaje?": los objetos se miden en cada tirada; el plano, el labio, las
asas y el contorno de la mesa salen del último `[g]`; las bandejas, de su objetivo. Con la tolerancia de
colocación (1.5 cm) eso se come el margen del prisma (10 cm en un hueco de 12: ±0.6 cm en la peor orientación).

- **`[g]` en la fase BANDEJAS de `marcar_montaje.py`**: guarda la mediana de las últimas 15 medidas de cada
  bandeja (centro en el torso y giro respecto de la mesa) como `medida` en `montaje.yaml`, más una copia con los
  fotogramas en `datos/dataset_<fecha>_bandejas_marcar/`. Solo las vistas enteras en ≥ 8 fotogramas y dentro de
  tolerancia; la C (a medias con los brazos a 0) se queda en su objetivo. El `[g]` de la mesa borra las
  medidas de las bandejas. `manip/montaje.py: bandeja_real()` da la medida o el objetivo, y la usan el
  planificador (punto de suelta y lado), la percepción (lo que no mira), el gemelo (posición y giro) y el
  mundo simulado.
- **`manipulacion.py` comprueba el montaje antes de pedir la palabra**: 6 fotogramas; la mesa contra el último
  `[g]` (±1 cm, ±1°) y cada bandeja que se vea entera contra su `medida` (±1 cm) o su objetivo (±1.5 cm). Si algo
  se movió, no empieza: "vuelve a pasar marcar_montaje.py". `--sin-comprobar-montaje` lo salta. En la
  simulación de nivel 2: mesa a 0.0/−0.1 cm, bandejas a ≤ 0.3 cm (la A no se ve entera con los brazos colgando
  y el marco calibrado), y los cuatro objetos a su bandeja al primer intento.
- **Salida**: `arnes.py` y `marcar_montaje.py` dejan una línea en blanco tras cada mensaje; `marcar_montaje.py`
  pone la mesa en una línea y cada bandeja en la suya.
- Pruebas nuevas (`test_montaje.py`): el `[g]` de las bandejas guarda solo las que debe; el gemelo pone la
  bandeja medida con su giro; la comprobación ve la mesa corrida 2 cm y una bandeja movida 2 cm de su medida.
  65 de 65.

## 26. Video de la simulación: primer avance (2026-10-29, en la PC)

Para enseñar el avance: la tirada de nivel 2 en el gemelo, desde que el robot está de pie con los brazos a 0 y
el montaje a su alcance hasta que los cuatro objetos están en su bandeja y los brazos vuelven a 0.

- **`manipulacion.py --sim --arnes`**: el robot simulado empieza de pie en el arnés con los brazos a 0
  (`postura_arnes`, lo que deja `arnes.py`). El supervisor no cambia: su reposo es la postura medida al
  empezar, así que va de 0 a la observación y en la SALIDA vuelve a 0. Con las cuatro colocaciones de la
  simulación (cada objeto del lado de su bandeja): 4/4 al primer intento, ~280 s de brazo a velocidad real
  (tramos ≥ 8 s, pico de 0.5 rad/s, `--escala-tiempo 1`). En el robot falta integrar `Cuerpo`.
- **`herramientas/video_simulacion.py`** corre esa tirada sin cambios (percepción sobre la cámara virtual,
  planificador, supervisor, manos simuladas) con un reloj de la tirada: los movimientos del brazo cuentan lo que
  durarían en el robot (la simulación acelerada no los espera) y lo demás (percibir, planificar, la mano), lo que
  tarda en la PC. Guarda el gemelo hasta 60 veces por segundo de ese reloj y después renderiza:
  - vista general (todo el robot y la mesa); en cada agarre, de cerca desde el lado del brazo que trabaja,
    primero el objeto y luego su bandeja; al final, las cuatro bandejas; transiciones suaves de 1.2 s;
  - la cámara de la cabeza en un recuadro; rótulos "Simulación MuJoCo · ×2", lo que está haciendo (p. ej.
    "Llevando el cilindro a la bandeja roja") y "Objetos en su bandeja: n/4";
  - la caída: el gemelo no tiene física y al abrir la mano el objeto pasaba al fondo de la bandeja de golpe. En
    el video sigue en la mano 0.2 s mientras los dedos se abren y cae con la gravedad, a velocidad real
    (4.7–6.9 cm, ~0.1 s), girando hasta su pose final.
- Video a ×2: **2 min 38 s**, 1920×1080 a 30 fps (H.264, 30 MB), en `datos/videos/` (fuera de git). La tirada
  tarda ~30 s y el render ~5.5 min.
- Pruebas (`test_video.py`): el reloj de la tirada, la caída, la cámara sin saltos al cambiar de toma y la
  tirada desde los brazos a 0 (4/4, empieza y acaba a 0, cuatro caídas, el guion del video).

## 27. Montaje colocado con el robot de pie, mano nueva y comprobación sin plano vivo (2026-11-10)

Robot de pie con `arnes.py` y el montaje colocado con `marcar_montaje.py`, en cuatro intentos.

- **El robot inclinado.** En el primero, desde que se cargó la cuerda la integral del estabilizador estaba al
  tope (+15) y el torso a +1.5…+1.6° con la referencia en +0.5° (el 18-oct, +0.4°). El `[g]` de la mesa se hizo
  así: 1.1° son 1.8 cm del torso (está a 0.93 m sobre el tobillo). Con el pórtico retrasado, +0.3…+0.45° e
  integral 0.55 el resto de la mañana. El aviso "INTEGRAL AL TOPE" de `arnes.py` hay que atenderlo antes del `[g]`.
- **Distancias con regla.** Las del 17-oct (centros a 12.5 / 28.5 cm del labio cercano, pared exterior a 6.5 cm
  del lateral) eran de la mesa a 0.275 m con la cámara del URDF; las bandejas quedaban 1.2–2.5 cm lejos y 1–2.6
  cm hacia el centro. Ahora se calculan de cada `[g]` (pared → cara interior del labio, con el escalado y el giro
  de la mesa): con la mesa girada 1°, las de un lado quedan ~1 cm más cerca del labio cercano que las del otro, y
  las laterales no son simétricas porque el marco de los brazos está ~1 cm a la derecha del centro del tablero.
  "Izquierda" es la del robot: la primera vez las bandejas quedaron en espejo (vistas desde el otro lado).
- **La mesa se movió al colocar las bandejas** entre el `[g]` de las 08:52 y las 09:10 (1.7 cm y 1.2°; la wincha
  del usuario lo confirma) y algo entre las 09:17 y las 09:33 (1.4 px en la imagen): frenos y cuidado al colocar.
- **Montaje final:** `[g]` con la mesa vacía de las 09:36, borde 0.292 m, +1.0 cm, −0.68° (escala 1.0114).
  `[g]` de las bandejas: B (roja) guardada en (0.556, +0.277), −0.3°; D (naranja) vista entera en 7 de 15
  fotogramas (hacen falta 8), a ≤ 0.8 cm de su sitio cuando se ve: se usa su objetivo; A (azul) fuera de la
  imagen bajo la mano izquierda y C (blanca) con una sola pared: colocadas con regla, y su borde coincide a pocos
  mm con su contorno objetivo pintado a la altura del borde (4 cm; `marcar_montaje` lo pinta a ras de mesa).
  Después se repitió el `[g]` de la mesa con las bandejas encima (`marcar_montaje` arranca en MESA) y dio 0.295 m,
  +1.7 cm, −1.25°: se restauró el de las 09:36 con la medida de B (respaldo en
  `datos/montaje_respaldo_20261110_0944_mesa_con_bandejas.yaml`) y se trajo a `config/` de la PC.
- **La mesa medida cambia con lo que hay encima, sin moverse.** Entre las grabaciones de las 09:36 (vacía) y las
  09:43 (con bandejas) los ArUco 2 y 3 se mueven ≤ 0.4 px en el color (~0.5 mm): la mesa no se movió. La
  diferencia sale de la profundidad: con las bandejas, el plano ajustado sale 0.46° más alabeado, y los ArUco
  del IR se corren 2.5 mm (los del color, 0.1 mm). Cambios:
  - el plano de la mesa se ajusta sin la huella de las bandejas (+3 cm): la diferencia baja a 0.25°, 0.3 cm y
    0.2° (lo que queda es de la profundidad del resto del tablero: el `[g]`, con la mesa vacía);
  - `montaje.comprobar()` (antes de cada tirada) corta los ArUco del color con el plano del `[g]` en vez de
    ajustar uno: con la mesa quieta da lo mismo vacía que con bandejas (+0.1 cm, −0.1 cm, 0.0°; antes hasta
    +0.9 cm y −0.6° en el robot), y la mesa de las 08:52 (otra posición, 4 px) sale movida 1.0 cm.
- **Mano montada como en el robot** (commit 0d5e452 de `h1_2_inspire_description`: anillo adaptador de 10 mm en
  la muñeca y brida de 16 mm, montaje en x = 0.161 en vez de 0.165). La yema pasa de x 0.2954 a 0.2914 en la
  muñeca y el punto de agarre de `objetos.yaml` de 0.215 a 0.211 (también en `poses_brazos.py` y las pruebas;
  `sim/editor_observacion/nucleo.py` sigue con 0.215). La calibración cámara–torso no cambia: los marcadores están
  ajustados en `*_wrist_yaw_link`. En el gemelo la mano derecha tapa la bandeja C a medias o entera según el
  modelo: la prueba lo acepta (se coloca con regla).
- **`herramientas/distancias_bandejas.py`**: la tabla con regla de cada bandeja desde el montaje guardado (pared
  exterior → labio lateral, pared cercana → labio cercano y pared lejana → labio lejano, con la más corta
  marcada, y el hueco entre las dos de cada lado); `marcar_montaje.py` la imprime al guardar la mesa. Las
  cuentas, en `montaje.distancias_bandejas()`; reproducen las que se dieron a mano el 10-nov.
- Pruebas: `test_montaje` mide los desvíos de las bandejas respecto de su objetivo aunque el montaje traiga
  medidas, y comprueba la tabla de distancias. 70 de 70.

## 28. manipulacion.py con el robot de pie en el arnés: arnes.py presta los brazos (2026-11-10, en la PC)

Decisión del usuario (opción B del PLAN): `arnes.py` sigue sosteniendo al robot por `rt/lowcmd` y es el único
que publica; `manipulacion.py --arnes` le pide los movimientos de los brazos. Se pone de pie una vez para varias
tiradas, y si `manipulacion.py` falla las piernas siguen sostenidas.

- **`manip/servidor_brazos.py`** (un hilo de `arnes.py`, TCP en 127.0.0.1:`red.puerto_brazos`, una línea JSON
  por petición): `estado`, `tomar` (solo de pie, nivelado, con el estabilizador, los pies cargados, sin parada y
  con los brazos a 0), `mover` (los 14 de los brazos con `Cuerpo.mover`: mínima sacudida ≥ 8 s, ≤ 0.5 rad/s, a
  los límites; uno cada vez), `congelar` y `soltar`. Cada petición es un latido: sin ninguno en
  `cuerpo.latido_max_s` (2 s) los brazos se congelan y se quita el control. Con el control dado, `[m]`, `[k]` y
  `[c]` no se aceptan. Una parada de `arnes.py` (ESPACIO, botón del mando, SSH cortado, temperatura) congela los
  brazos, solo acepta movimientos de salida, espera a que `manipulacion.py` haga su salida segura y suelte (180 s
  como mucho) y después cuelga como siempre. L2+B, igual que antes (amortigua en el hilo de `Cuerpo`).
- **`manip/brazo_arnes.py`**: la interfaz de `Brazo` para el supervisor sin publicar nada. "Peso" = tener el
  control. `ir_a` pide el movimiento y espera mirando el estado y llamando a `comprobar`; si algo salta (el
  contacto de la yema, una parada) **congela antes de dejar salir la excepción** (con `arm_sdk` el brazo se
  paraba solo porque los pasos los daba ese bucle). El vigilante pregunta `motivo_parada()` (parada o
  amortiguación en `arnes.py`, control perdido, `arnes.py` sin contestar); la salida segura marca sus movimientos
  como de salida. En Debug no hay FSM ni el joystick hace andar: no se vigilan.
- **`manipulacion.py --arnes`** en el robot: reposo = brazos a 0; `--recuperar`: solo la salida segura desde
  donde estén los brazos (si `manipulacion.py` murió a medias, `arnes.py` los dejó congelados).
- **En la PC**, con el `Cuerpo` de verdad sin publicar sobre el robot simulado y un reloj ×40
  (`tests/test_brazo_arnes.py`, 10 pruebas): tomar, mover y soltar; brazos fuera de 0 solo con `--recuperar`; un
  `comprobar` que salta congela el brazo a medio camino; sin latido se congela y se pierde el control; parada de
  `arnes.py` → solo la salida; amortiguado → nada; `arnes.py` caído. En el gemelo, la tirada de nivel 2 a través
  del servidor: **4/4**, acaba con los brazos a 0 y el control devuelto; ESPACIO en `arnes.py` con el cilindro en
  la mano → salida segura por un punto alto y la observación a 0; `--recuperar` trae a 0 un brazo dejado sobre la
  zona.
- **`arnes.py --sim --operador-auto --con-remoto`** (con física): el cliente toma los brazos, los dos a la
  observación y a 0, y otra vez con ESPACIO a los 4 s: congelados, su salida a 0, suelta y `arnes.py` cuelga y
  termina. Con los dos brazos a la observación el torso se va +0.5° (pitch respecto de la referencia, sobre el
  +1.7° de la simulación) y +0.1° de roll, tobillos ≤ 30 N m, pies al 72–74 %; con `[m]` (un brazo), +0.15°.
- **Ganancias:** los brazos se mueven con las de `arnes.py` (`tuned_gff`), con el hombro derecho (pitch) a kp 280,
  por encima del tope de 150 de la sección 10 que aplicaba `manipulacion.py` con `arm_sdk`. Decisión del usuario:
  se quedan como se probaron en el Hito 0.
- Pruebas: 80 de 80.

## 29. Primera tirada en el robot a través de arnes.py: observación y vuelta (2026-11-11)

Desplegado `74682be`; `arnes.py` de pie y nivelado; `[g]` de la mesa vacía en `marcar_montaje.py` (02:44: borde
0.297 m, +0.6 cm, +0.70°, dentro de `objetivo_mesa`) y `./manipulacion.sh --nivel 1 --arnes` sin dianas
(`datos/tirada_20261111_024454_nivel1`, `arnes_20261111_023839`).

- **Completada**: toma los brazos, los dos a la observación en 16 s (escala 2), 0 objetos y 0 dianas (47 ms),
  vuelta a 0 en 16 s, suelta. 32 s de tirada; `arnes.py` siguió de pie.
- **Brazos** (consigna de `arnes.py` frente a la medida): error máximo 3.7° (codo derecho) y 3.2° (hombro izquierdo,
  roll) durante el movimiento; en la observación, el hombro (pitch) 1° por debajo (0.599 de 0.616 rad); par máximo
  15 N m.
- **Cuerpo**: con los brazos adelante el torso pasa de +0.62° a +0.20° de pitch (la integral del estabilizador,
  de −1.84 a −2.97, y vuelve a −2.08) y el roll no se mueve (±0.05°); pies al 72–73 % todo el tiempo (izquierdo
  275 N, derecho 215 N, el reparto de siempre); tobillos ≤ 12 N m; ciclo ≤ 2.8 ms; 52 °C. Como en la simulación
  (+0.5°). Ojo: 0.4° de torso (girando sobre los tobillos, 0.93 m) son ~6 mm en la zona entre la mesa del `[g]`
  (brazos a 0) y la que ve el robot con los brazos adelante; dentro de las tolerancias, pero se suma al error.
- **El primer intento (02:33) no llegó a moverse**: `arnes.py` estaba en COLGAR. Se había pulsado la **flecha
  derecha**, que el terminal manda como `ESC [ C`: el ESC no es imprimible y se saltaba, el `[` no hacía nada y la
  `C`, pasada a minúscula por el Bloq Mayús, era la `[c]` de colgar (`eventos.csv`: `tecla '['` y `tecla 'C'` en
  el mismo instante). Arreglo: `manip/teclado.py` quita las secuencias de escape (flechas, Inicio/Fin, F1–F12,
  AvPág, Alt+tecla; partidas entre lecturas si llegan en < 0.1 s) antes de pasarlas a `arnes.py` y
  `marcar_montaje.py`. Probado también con un pty de verdad.
- Ese mismo `arnes.py`, parado con Ctrl+C mientras llevaba los brazos a la postura colgada, avisó "los brazos
  están a 79 grados de 0 … `--recuperar`": falso, iban de 0 a la postura colgada. Ya no se avisa si la parada fue
  en FIN.
- El segundo `arnes.py` (02:35) terminó sin `resumen.json` y con `eventos.csv` vacío (el `ciclo.csv` llega hasta
  kp a 0, así que se soltó bien): salió sin pasar por el cierre y los eventos seguían en el búfer. Los
  `eventos.csv` de `arnes.py` y de las tiradas se escriben ahora al momento (`CsvSimple(al_momento=True)`).
- La mesa de este `[g]` está girada +0.70° y la del 2026-11-10 −0.68° (el robot se pone de pie cada vez un poco
  distinto): con las bandejas donde estaban, C y D quedan ~1.2 cm más cerca del robot de lo que toca. Para los
  niveles con bandejas: `herramientas/distancias_bandejas.py` y la fase de bandejas de `marcar_montaje.py`.
- Pruebas: 85 de 85 (`tests/test_teclado.py`, nueva).

## 30. Camino a la observación desde los brazos a 0 y curva de pasos (2026-11-11, en la PC)

**El camino de los brazos a 0 a la observación pasaba a 13 mm de la mesa.** Al preparar la curva de pasos, el
gemelo (con la mesa del 10-nov y con la del `[g]` de las 02:44) dio el meñique izquierdo a 13–15 mm del tablero
hacia el 30 % de la recta articular, el mismo movimiento de la tirada del §29. Con ±3° en el hombro (pitch) o el
codo (en el robot hubo hasta 3.7° de error de seguimiento) baja a 4–8 mm con un brazo, y con los dos en espejo
llega a tocar (−13 mm). La comprobación del planificador no lo veía: 8 muestras por tramo (0.22 rad entre muestras
en un tramo de 1.8 rad) y el mínimo caía entre la del 25 % y la del 37 %. La pose se había elegido en octubre desde
el reposo de la marcha, no desde los brazos a 0.

- **Planificador:** además de `muestras_tramo` (8, como mínimo), una muestra cada `paso_comprobacion_rad` (0.03)
  de la junta que más se mueve. Con eso la recta a la observación ya no pasa (18 mm < 20).
- **Punto de paso** (`poses_brazos.py --solo-paso`, `paso_inicio` en `config/poses_brazos.yaml`): se mantiene la
  pose de observación (zona, vista y niveles ya probados) y se busca qué juntas mover primero. Con **el hombro
  (roll) primero** y luego el resto, los dos tramos quedan a más de 5 cm de todo (el tope de la medida), también
  con ±3° en hombro y codo; es lo que hace `arnes.py` al colgar ("hombros fuera"). La entrada y la salida pasan a
  dos movimientos (16 + 16 s a escala 2). Solo con `--arnes` (el reposo de la marcha no es 0).
- **Supervisor:** `_ambos` recorre los puntos de paso de los dos brazos uno a uno (antes iba directo al último:
  un punto de paso se habría saltado), también en la salida segura "observación → reposo".
- **Nueva pose desde 0 (lo que pedía el PLAN):** con el inicio en 0 y el camino exigido con ±3° no salió ninguna
  en 8 × 750 + 3000 intentos; el criterio que las tumba es la recta de la observación a la esquina lejana del otro
  lado de la zona (la pose actual tampoco lo cumple con la mesa de ahora, y el planificador lo resuelve con otras
  soluciones de la IK). `poses_brazos.py` queda con `--desde cero` por defecto, el camino desde el inicio robusto
  a ±3° y el otro brazo en su observación al probar la zona.
- **Gemelo:** nivel 2 con `--sim --arnes`, 4/4 al primer intento por el punto de paso.
- **`desplegar.sh`** ya no pisa un `config/montaje.yaml` del robot más nuevo que el de la PC (el `[g]` de las 02:44
  se perdió así): lo deja y lo trae a `datos/montaje_robot_<fecha>.yaml`.

**Curva de pasos (6.3.1–2)**, `herramientas/curva_pasos.py` y `analizar_pasos.py`:

- Cliente de `arnes.py` como `manipulacion.py --arnes`. `arnes.py` le da lo que mide en su ciclo de 500 Hz
  (`estado` con `cuerpo`: IMU con yaw, carga y fuerza de los pies, integral, q y par de las 27), así que no hace
  falta DDS y funciona igual contra `arnes.py --sim`. `ciclo.csv` de `arnes.py` lleva ahora también yaw y gz.
- Movimientos: `observacion` (por el punto de paso; centro de masas 4.0 cm atrás y 2.9 arriba, lo que explica el
  −0.4° de pitch del §29) y `alcance_izq`/`alcance_der` (el `[m]`; centro de masas +0.3 cm): parecida velocidad
  articular con desplazamientos del centro de masas muy distintos. A 30, 20, 10 y 8 s, de la más lenta a la más
  rápida, con 8 s quietos antes, en la pose y al volver.
- Antes de mover: caminos limpios en el gemelo (holgura del plan y ±3° a 1 cm) y la palabra. Para con ESPACIO,
  Ctrl+C, parada de `arnes.py` o la IMU a más de 3° (roll/pitch) o 3° de yaw del inicio: deshace el camino (al
  principio del tramo cortado y por los puntos de antes) y suelta.
- Análisis por movimiento, duración y sentido: pico, lo que queda y el exceso dinámico de pitch y roll, giro,
  carga y tobillos; velocidad y aceleración articulares y desplazamiento, velocidad y aceleración del centro de
  masas (Dinámica con las masas identificadas); r² de cada candidato; duración mínima (exceso ≤ 0.2°, giro ≤ 0.5°);
  `curva.png`. Pasos: 0 (de pie por `rt/lowcmd` no hay marcha).
- **Con física** (`arnes.py --sim --operador-auto --esperar-cliente` y `curva_pasos.py --sim --rapido`, los dos
  alcances a 20/10/8 s): completada; pitch ≤ 0.13°, exceso ≈ 0, yaw ≤ 0.1°, lo que más explica es el
  desplazamiento del centro de masas, duración mínima 8 s. Falta en el robot.
- Pruebas: `tests/test_curva_pasos.py` (5, con el banco ×40 de la opción B).

## 31. Curva de pasos en el robot (2026-11-11)

Desplegado `ed2d56e`; `arnes.py` de pie y nivelado (pies al 71–74 %), `[g]` de la mesa (04:39: borde 0.295 m,
+0.8 cm, −0.67°) y `curva_pasos.py` con lo de por defecto: la observación (por el punto de paso, dos tramos de T
cada uno) y el alcance izquierdo, a 30, 20, 10 y 8 s, ida y vuelta, 8 s quietos entre medias
(`datos/pasos_20261111_044016`, `arnes_20261111_043610`). **Completada** en 10 min, sin paradas. El usuario vio
las manos más lejos de las asas que en la tirada del §29 (en recta); en el gemelo, con esta mesa, la recta pasa a
30 mm del tablero y por el punto de paso a más de 100 mm de mesa y asas.

| | T (s) | pitch pico (°) | queda (°) | exceso (°) | roll (°) | yaw (°) | carga | tobillos (N m) | error brazo |
|---|---|---|---|---|---|---|---|---|---|
| observación, ida | 30 / 20 / 10 / 8 | +0.10 / −0.10 / −0.14 / −0.14 | 0.00 a −0.12 | ≤ 0.09 | ≤ 0.09 | ≤ 0.31 | +2.7 % | −24 a −25 | ≤ 3.9° |
| observación, vuelta | 30 / 20 / 10 / 8 | +0.19 / +0.15 / +0.15 / +0.17 | +0.10 a +0.15 | ≤ 0.05 | ≤ 0.08 | ≤ 0.26 | −2.8 % | +25 | ≤ 3.1° |
| alcance izq. | 30 / 20 / 10 / 8 | ≤ 0.09 | ≤ 0.05 | ≤ 0.08 | ≤ 0.07 | ≤ 0.15 | ±0.5 % | ±3 | ≤ 4.3° |

- **Pasos: 0** (de pie por `rt/lowcmd`). **Giro ≤ 0.31°**, con una deriva del yaw de la IMU de −0.7° en los 10 min
  (~0.07°/min, que entra en las ventanas largas). **Inclinación ≤ 0.19°** de pitch y ≤ 0.09° de roll.
- **No depende de la duración entre 8 y 30 s.** Lo que manda es el desplazamiento del centro de masas (r² 0.74 con
  el pico de pitch y 0.67 con el giro): la observación lo lleva 4 cm atrás y los tobillos cambian 25 N m y la carga
  2.7 % a cualquier velocidad; el alcance (+0.3 cm), 3 N m. El exceso dinámico es ≤ 0.09° y no sigue a la
  velocidad ni a la aceleración (r² ≤ 0.3: ruido). En la simulación con física, lo mismo (≤ 0.13°, §30).
- **Duración mínima: 8 s**, el tope del PDF (exceso ≤ 0.2° y giro ≤ 0.5° en todo): `t_min_mov_s` se queda en 8 y
  las primeras tiradas, a escala 2 (16 s por tramo), como pide el PDF.
- El error de seguimiento de los brazos sube con la velocidad: 2.1–2.4° a 30–20 s y 3.6–4.3° a 10–8 s (alcance).
- Ojo: la integral del estabilizador estuvo **al tope (+15 N m) toda la sesión** (la cuerda tirando del torso
  hacia delante: retrasar el pórtico). El torso se quedó dentro de la banda (+0.30…+0.63° con referencia +0.5) y
  la integral no se movió: la curva mide lo que hacen la parte proporcional y la derivada.

## 32. vibracion_camara.py: cuánto se mueve la cámara con el robot quieto (2026-11-11, en la PC)

Para 6.2.5 ("medirlo con un marcador fijo en la mesa"): en cada fotograma de color, los ArUco de la mesa (2–5, los
que se vean; con la mesa a ~0.29 m, el 2 y el 3) con esquinas a subpíxel y solvePnP. Lo que mueve a la vez a todos
es la cámara (modo común); la mitad de la diferencia entre dos, el ruido de la detección (σ por marcador = √2 ×
su desviación; en el modo común de N, σ/√N). Da el modo común en px, minutos de arco (px/f) y mm en los marcadores
y en el centro de la zona (con la calibración y el montaje), el **movimiento neto** (modo común sin el ruido), la
frecuencia dominante y la profundidad de solvePnP; con `arnes.py` corriendo, la IMU del torso (estado con `cuerpo`,
sin tomar los brazos) en la misma ventana y la correlación del pitch de la imagen con el de la IMU.

- Pruebas con imágenes sintéticas (`tests/test_vibracion_camara.py`, 3): una senoide común de 1.5 px a 2 Hz sale
  con su desviación (amplitud/√2), su frecuencia y su amplitud; un ruido propio de 0.3 px por marcador sale como
  ruido y no como movimiento; una IMU que gira como la imagen da r > 0.95 y pendiente 1.
- Sobre las dos grabaciones de 1 s del robot de pie en el arnés (2026-10-17, 30 fotogramas a 30 fps, 2 y 3 a
  1.04–1.05 m): modo común 0.02–0.11 px, igual que el ruido de la detección (0.05–0.12 px por marcador): **el
  movimiento neto no se distingue del ruido** (< 0.1 px ≈ 0.5' ≈ 0.15 mm en los marcadores).

**En el robot** (2026-11-11 05:06, `datos/vibracion_20261111_050647`): 60 s con `arnes.py` de pie y nivelado, pies al
84 %, la integral del estabilizador en +0.1 N m (tras la curva se movieron el pórtico y el polipasto hasta que dejó de
salir INTEGRAL AL TOPE, sin reiniciar `arnes.py`: la misma sesión que la curva del §31), nadie tocando. 1799
fotogramas a 30 fps con el 2 y el 3 en todos, a 0.83 m (la zona, a 0.83 m de profundidad).

- **La cámara no se mueve de forma medible.** Fotograma a fotograma, el modo común (0.09 px) es el ruido de la
  detección (0.12–0.13 px por marcador, 0.09 en el modo común): movimiento neto 0. Con medias de 1 s (que quitan
  ese ruido): 0.015 / 0.022 px de desviación (horizontal / vertical) = 0.1', 0.08 / 0.10 px pico a pico, ~0.03 mm en
  la zona, y sin tendencia (0.000 / −0.004 °/min). Respuesta a 6.2.5: con el robot quieto, el equilibrio no mueve
  la cámara más de 0.1 px (< 0.2 mm en la mesa): despreciable frente al resto de errores de la percepción (la
  profundidad, ~1–2 %). Ojo: es con nuestro estabilizador en Debug, no con la política de marcha (FSM 201).
- **La IMU sí "se mueve": su yaw deriva −0.45°/min** (28' en el minuto) mientras la cámara no gira nada
  (+0.000°/min): es la IMU, no el cuerpo. En la curva de pasos (§31) derivó −0.07°/min: cambia de una sesión a otra.
  El vigilante para con 5° de yaw desde el inicio de la tirada: a −0.45°/min, a los ~11 min, y una tirada de
  cuatro objetos a escala 2 puede acercarse. Roll y pitch de la IMU, 3–3.5' pico a pico en medias de 1 s y la
  cámara 0.6': lo de la IMU es sobre todo su ruido. Sin movimiento real, la correlación de la imagen con la IMU
  sale 0.

## 33. El giro del vigilante sin la deriva del yaw de la IMU (2026-11-11, en la PC)

En la sesión de `arnes.py` de la curva y la vibración (40 min, `arnes_20261111_043610`, que ya guarda el yaw), el
yaw de la IMU fue de +4.3° a −1.6° con un ritmo que cambiaba solo entre +0.43 y −0.59°/min (minuto a minuto), con
el robot quieto; en el minuto de `vibracion_camara.py`, −0.45°/min con la cámara sin girar (+0.000°/min). No
basta con medir la deriva al empezar y restarla. El vigilante paraba con 5° desde el inicio de la tirada.

`manip/giro.py`: el giro que se vigila sigue siendo el yaw de la IMU (lo que pide el PDF), menos su deriva medida
con los ArUco 2 y 3 de la mesa, que no derivan:

- **Giro respecto de la mesa:** el ángulo en planta de la recta entre los ArUco 2 y 3 (el cruce de las diagonales de
  cada uno en el color, llevado al plano de la mesa del `[g]` con la calibración cámara–torso). La mesa vista gira
  al revés que el cuerpo.
- **Deriva medida** = lo que gira la IMU desde el inicio − lo que gira el cuerpo respecto de la mesa, a 2 Hz en un
  hilo con el último fotograma (`LectorContinuo`). **Deriva(t)** = la recta de las medidas de los últimos 30 s
  en t: si un brazo tapa los marcadores, sigue con su ritmo (como mucho 1°/min).
- **Giro vigilado** = IMU − deriva(t). Un giro de verdad lo ven la IMU y la cámara a la vez y no se resta; si se
  mueve la mesa (un golpe al carrito) sale como giro y se para, que también es lo que toca. El mensaje dice las dos
  cosas ("giro del cuerpo +6.0 grados (IMU +0.2, menos su deriva −5.8 medida con la mesa)"); `lowstate.csv` lleva la
  columna `deriva_yaw_deg` y `resumen.json` la deriva final y cuántas medidas hubo. Solo en el robot
  (`supervisor.giro_con_mesa`); en la simulación la IMU no deriva.
- **Con los 60 s reales de `vibracion_camara.py`:** la IMU giró −0.448° y el giro corregido −0.044° (máximo
  0.10°). En el gemelo, la mesa girada 1.5° se mide con ±0.2° (los ArUco lejanos miden ~23 px y el render no
  suaviza: 0.5–2 mm por marcador; en el robot, 0.17 mm).
- Pruebas (`tests/test_giro.py`, 5): el gemelo; el hilo con el último fotograma, como en el robot; 10 min de deriva a −0.45°/min con un giro real de 3° a los 5 min y
  un minuto con los marcadores tapados (giro corregido a ±0.05°); el tope del ritmo extrapolado; el vigilante no
  para con casi 6° de deriva y sí con un giro real de 6°.

## 34. Manos: probar_manos.py, el corte de fuerza con persistencia y lo que ya midió inspire_hand_interface (2026-11-11, en la PC)

**Cambio de objeto (usuario):** la pelota de tenis se cambió por una **esfera de PLA naranja de 7 cm** (el color de
la bandeja D y del prisma; peso sin medir). No afecta al Hito 3; antes de los niveles 2–4: color, masa y FORCE_SET
en `objetos.yaml` (rígida y lisa: rueda y resbala más que el fieltro), el gemelo y los datos sintéticos, y
distinguirla del prisma por la forma (7 × 7 × 7 cm frente a 10 × 4 × 4; la profundidad ve las esferas más bajas).

**Lo que ya está medido en banco** (`/home/utec/Documents/inspire_hand_interface`, resumen de sus documentos):

- La fuerza es **FORCE_ACT** (gf, tarado con la mano abierta), sin báscula ni sensor externo: ningún gramo está
  calibrado contra una referencia (lo tienen pendiente). Residual de flexión sin objeto: 70–232 gf según la
  posición del dedo.
- **Lecturas sueltas corruptas** (3187 gf, 1873 gf, una POS fuera de sitio): todo umbral necesita persistencia
  (ellos exigen 3 ciclos).
- **Sobreimpulso:** lo manda la velocidad (índice con FORCE_SET 500: 54 gf a SPEED 25, hasta ~3000 gf a 750; el
  pulgar, 0.3–0.6 de eso). FORCE_SET bajo no protege: por debajo de ~150 (meñique), 80 (anular), 60 (medio) y 53
  (pulgar) el dedo se frena en el aire. Tras el pico el firmware no sostiene FORCE_SET (se relaja ~10 %).
  **Techo sostenible:** 455–745 gf según dedo y pose.
- **Enclavamiento** al llegar a FORCE_SET (no acepta ANGLE_SET): ellos suben FORCE_SET antes de abrir, como
  `mano.py` (a 800, el tope de la sección 10; nuestros cierres a 300–350 no llegan ahí).
- **Táctil:** 17 zonas; yemas 3×3 (la del índice en 4110, la que lee `mano.py`), ruido casi nulo (el 93 % de los
  taxeles con σ = 0), umbral `max(5σ, 41 cuentas)`, ~250 cuentas de residuo tras apretar fuerte. Sin calibrar a
  fuerza, no se usó en ningún agarre ni se probó el contacto con una superficie; el ritmo por TCP, sin medir.
- **Agarres:** cubo de PLA con tres dedos 295/179/341 gf estable (con dos, inestable); bola de espuma 7 cm con tres
  dedos a ~280 gf, 60 s; mandarina ~120 g resbala a 150 gf y pide ~300. **Ninguna esfera rígida y lisa.**

**`herramientas/probar_manos.py`** (Fase 0 y Hito 3, "la mano abre y cierra en el aire"): con `manip/mano.py`, por
mano, leer (ángulos, fuerza, error y estado del firmware), línea base, ruido del táctil de la yema del índice con la
mano quieta y **ritmo de lectura por Modbus TCP** del estado y del táctil (la latencia del contacto del nivel 1);
cada preforma de `objetos.yaml`, cerrar en el aire con el FORCE_SET de un objeto que la usa y comprobar que
`mano.agarre()` dice "vacío" en todos los dedos (un falso "lleno" en el aire rompería la pista del PDF), y abrir
comprobando que abre; `--tactil S` para apretar la yema y ver si pasa el umbral. No mueve los brazos. Probado con la
mano simulada (las dos manos, las tres preformas: abre y todo "vacío", 160 gf de flexión en el aire).

**Corte de fuerza con persistencia:** `mano.py` (al esperar a que la mano se pare) y el vigilante disparaban la
salida segura con **una** lectura por encima de 1200 gf: una corrupta paraba la tirada. Ahora hacen falta
`mano.persistencia_corte` (3) lecturas seguidas; el vigilante cuenta lecturas distintas (por su t), no vueltas
suyas. Pruebas en `tests/test_mano.py` (2 nuevas).

## 35. Las manos reales en el aire (2026-11-11)

`probar_manos.py --tactil 10` en el robot (`datos/manos_20261111_061312`), las dos manos, las tres preformas.
**Las dos manos contestan, hacen las preformas y abren siempre** (1.8–2.5 s), sin errores del firmware. Una lectura
del estado tarda 2 ms (máx. 11) y la del táctil del índice 1–2 ms (máx. 11) por Modbus TCP.

- **Cierran a ~150 cuentas/s con SPEED 100** (1000 → 0 en ~7 s; la simulada iba al doble). `cerrar()` esperaba
  4 s: a los 4 s el meñique, el anular y el medio **seguían moviéndose**, y el "flojo" de `agarre()` no decía nada.
  Ahora espera a que se paren hasta `mano.t_cerrar_s` (10 s), y "parada" es no apartarse más de 6 cuentas en
  0.3 s (antes, entre lecturas seguidas: despacio, un dedo en marcha parecía parado).
- **El índice choca con la punta del pulgar** (lo vio el usuario): con el pulgar enfrente (rotación 0) y cerrando a
  300, el índice se para en 445 (derecha, la fuerza salta a ~250 gf) y ~498 (izquierda). Es un contacto limitado
  por FORCE_SET, pero **en vacío los dedos no llegan a 0**: la pista del PDF ("un dedo que se para lejos de 0 con
  fuerza tiene algo") daría "lleno" sin objeto. Hay que medir dónde se para cada dedo cerrando en vacío (más
  despacio) y comparar con eso, o cerrar sin que el pulgar se cruce.
- **El anular derecho no se movió con FORCE_SET 300** (se quedó en 992): abierto marca +210 gf y el firmware
  compara con FORCE_ACT en bruto (le quedaban 90 gf). Con 800 (preformas, abrir) iba bien. `mano.py` escribe ahora
  FORCE_SET + la base de la mano abierta (`compensar_base`), entre 100 y 800. Bases medidas: izquierda
  [85, 93, −4, 74, −36, 25] y derecha [33, 210, 38, 106, −10, −150] gf.
- **El táctil no respondió:** apretando la yema del índice subió 4 (izquierda) y 0 (derecha), y la izquierda,
  sin tocarla, varió hasta 254 cuentas (desviación 52). Los registros (4110 y 4128) son los de la interfaz de
  `inspire_hand_interface` (que lo leía por serie). `probar_manos.py --mapa-tactil S` lee las 17 zonas para ver
  cuál responde y cuál varía sola.
- Estado del firmware: 1 moviéndose, 2 parado; el pulgar (flexión) dio 255 al leerlo con la mano quieta.
- Pruebas: `tests/test_mano.py` (la base en FORCE_SET y el cierre lento).

## 36. Cierre en vacío, calibración por mano y fuera el táctil (2026-11-11)

`probar_manos.py --speed 50 --preformas abierta_opuesta tres_dedos --mapa-tactil 30`
(`datos/manos_20261111_062824`), con la espera larga y la base en FORCE_SET del §35:

- **A SPEED 50 cierran a ~80 cuentas/s** (12 s de 1000 a 0). Meñique, anular y medio llegan a 0–24. **Solo el
  índice choca con la punta del pulgar**, siempre en el mismo sitio: izquierda 498–502, derecha 445–447 (también
  a SPEED 100 en el §35), con 170–233 gf.
- **Fuerza de cada dedo cerrado en vacío (su propia flexión):** izquierda 149/110/131/205/47 gf (meñique, anular,
  medio, índice contra el pulgar, pulgar); derecha 231/392/223/233/225. El **anular derecho** marca 392 gf
  cerrado del todo (con FORCE_SET 800, en la preforma) y con 305 se paró en 965: no cierra en el aire. La base de
  la mano abierta cambia de una vez a otra (el anular derecho, 210 gf en el §35 y 5 aquí).
- **El táctil no responde por Modbus TCP:** de los 1062 taxeles de cada mano, solo ~30 cambian en toda la
  grabación apretando la punta y la yema de cada dedo y la palma (como mucho 22–60 cuentas en la izquierda y 12 en
  la derecha, por debajo del umbral de 41) y algunos varían solos (la punta del medio izquierdo, 226). El índice
  izquierdo varió 254 sin tocarlo en el §35. En banco, por serie, sí respondía (`inspire_hand_interface`).

Cambios:

- **`config/manos.yaml`** (`probar_manos.py --guardar-cierre <registro>`): por mano y dedo, dónde se para
  cerrando en vacío (`tope_vacio`) y su fuerza ahí (`residuo_gf`). `manip/mano.py` (solo con la mano real):
  - el índice cierra hasta su tope + `margen_tope` (40): izquierda 542, derecha 487. **En vacío ya no choca con
    el pulgar y llega a su consigna** ("vacío"); con un objeto se para antes, con fuerza ("lleno");
  - FORCE_SET = el del objeto + la base + el residuo (`compensar_residuo`), entre 100 y 800: para 300 gf,
    izquierda ~400–505 y derecha ~510–700 (el anular, 692).
- **Fuera el táctil** (`contacto.usar_tactil: false`): el toque del nivel 1 se nota solo con la fuerza del índice
  (`contacto.umbral_gf`); antes bastaba con que el táctil subiera, y una lectura suelta de 254 lo habría
  disparado. El agarre nunca lo usó (ángulo y fuerza, la pista del PDF). Para 6.4.4 (táctil, fuerza de los dedos o
  error del brazo): el táctil queda descartado con estos datos; falta medir la fuerza del índice y el error del
  brazo bajando sobre la mesa.
- Pruebas: `tests/test_mano.py` (3 nuevas: la calibración en `Mano`, sin ella con la mano simulada, y sacarla de un
  registro).

## 37. Las manos con la calibración: todo bien (2026-11-11)

`probar_manos.py --preformas abierta_opuesta tres_dedos` (`datos/manos_20261111_064335`), a SPEED 100 con
`config/manos.yaml`: **las dos manos "bien"**: cierran en ~6.4 s (156 cuentas/s), todos los dedos "vacío" y abren
en 2.4–2.5 s.

- **El índice se para en su consigna, antes del pulgar:** izquierda 541–542 (consigna 542), derecha 488 (487).
- **El anular derecho ya cierra** (a 2, con FORCE_SET 660); marca 492 gf cerrado (392 en el §36: el residuo se
  mueve; quedan ~170 gf de margen).
- Fuerza neta con los dedos cerrados en vacío: izquierda 110–225 gf (el índice, 201–225 en su consigna), derecha
  66–291 y el anular 492. FORCE_SET escrito: izquierda 413–526, derecha 490–660.

Con esto, `mano.py` en el robot queda probado (Fase 0) y "la mano abre y cierra en el aire" del Hito 3.

## 38. arnes.py empieza con las manos abiertas; probar_toque.py (2026-11-11, en la PC)

- **`arnes.py` abre las dos manos del todo** al empezar (pedido del usuario): tras escribir COLGADO y antes de tomar
  el robot, los seis DOF a 1000 (primero la flexión del pulgar y luego su giro: `Mano.preforma`), que es la mano
  abierta del gemelo con la que se comprueban los choques. Si una mano no contesta o no abre, no sigue
  (`--sin-manos` para saltarlo). En `--sim`, con las manos simuladas cerradas: las dos abren.
- **La bajada del nivel 1, a su velocidad:** `contacto.vel_descenso_m_s` no se usaba (bajaba en la duración mínima,
  4–5 mm/s con la escala 2). Ahora la bajada dura recorrido / velocidad, contada la escala de tiempo.
- **`herramientas/probar_toque.py`** (6.4.4 sin táctil): una tirada del nivel 1 con una sola diana fija
  (`--diana x,y`; la cámara no decide nada) y límites de prueba: aproxima a 4 cm, baja a 3 mm/s y para con la
  fuerza del índice (`--umbral`, 80 gf) o a 1 cm bajo la mesa del `[g]` (el nivel 1, 2). Graba cada lectura de la
  mano y el brazo a 50 Hz, y después compara, frente a la altura de la yema sobre la mesa (cinemática con las q
  medidas), cuándo pasan la fuerza del índice (30/60/120 gf), el error de seguimiento del brazo (0.5/1/2°) y el par
  de la muñeca (+0.3/0.6/1 N·m): qué nota antes la mesa. Con `--espesor` cuenta una espuma o un trapo. Probado en
  el gemelo (`--sim`) de punta a punta.

## 39. Primer toque en el robot: el índice vertical no nota la mesa; toque inclinado (2026-11-11)

`probar_toque.py --espesor 1` (`datos/tirada_20261111_073740_nivel1`): `[g]` nuevo, espuma de 1 cm (36 × 27 cm)
sobre el tablero, diana (0.48, 0.10) con la mano izquierda. La tirada fue bien (observación, sobre la diana, bajada,
vuelta, 0), la yema bajó a 3.6 mm/s y llegó a 1.9 mm sobre la mesa del `[g]` (8 de los 10 mm de espuma
aplastados), pero **"sin contacto": ninguna señal lo notó**. La fuerza del índice no pasó de 30 gf; el error del
brazo (>2° a 22 mm de la yema sobre la mesa) y el par de la muñeca (+0.3 N·m a 34 mm) se movieron antes de tocar,
por el propio movimiento. En la foto (`montajes/probar_toque_espuma.jpeg`), **el índice vertical y extendido del
todo** (preforma `senalar`, 1000): la espuma lo empuja a lo largo del dedo, contra su tope de extensión, y el
actuador no recibe la carga. Sin espuma, el nivel 1 habría empujado el dedo contra el tablero hasta 2 cm.

Cambio (propuesta del usuario: el dorso de la mano al frente y la mano inclinada sobre el tablero):

- **Preforma `tocar`** (`objetos.yaml`): el índice en 900 (algo flexionado: la mesa lo abre contra su actuador y le
  quedan 100 cuentas antes del tope), los demás recogidos y el pulgar abierto sin girar.
- **Orientación** (`planificacion.toque`): la mano 30–55° bajo la horizontal, hacia delante y con la palma hacia
  abajo; la falange del índice queda a ~50–75°, con 3–6 cm de palanca sobre el nudillo. La IK y la comprobación
  usan una cinemática con los dedos de la preforma (la yema se mueve con la flexión). Barrido en el gemelo (8 puntos
  de la zona): con la mano a 15–45° solo 2–4 limpios (los dedos recogidos y el pulgar rozan la mesa o las
  bandejas), con 30–55° 7–8. Con varias soluciones de la IK de "sobre" (como `coger`), 16 de 18 puntos de una
  rejilla 3 × 3 por mano; fallan las dos esquinas cercanas izquierdas (x = 0.40, y = 0.10 y 0.20: el hombro contra
  el torso).
- **El contacto, también por posición:** la mesa abre el índice (su ANGLE_ACT sube más de `contacto.umbral_angulo`,
  25 cuentas) o la fuerza pasa de `umbral_gf`, en 2 lecturas seguidas (hay lecturas sueltas corruptas de las dos).
  `TOCAR` dice por qué y cuánto se abrió.
- **La comprobación del toque** deja de mirar los choques del resto de la mano mientras la yema está por debajo
  del tablero: la bajada para al notarlo y la subida empieza ahí.
- **El gemelo** calcula la fuerza de la yema con los dedos donde están (antes, con la mano abierta, el índice
  flexionado no llegaba).
- `probar_toque.py` analiza también la apertura del índice (10/25/50 cuentas) con la cinemática de la preforma.
- Pruebas: `tests/test_supervisor.py` (el toque inclinado con la palma abajo en cuatro puntos de la zona).

## 40. Segundo toque: el índice en 900 tampoco lo nota; en 800 y prueba a mano (2026-11-11)

`probar_toque.py --espesor 1` con el toque inclinado del §39 (`datos/tirada_20261111_081540_nivel1`; `[g]` nuevo;
la mano derecha siguió con el índice extendido de la prueba anterior porque `arnes.py` no se reinició): la yema
bajó a 3.9 mm/s hasta 2 mm sobre la mesa (8 mm de espuma) y **otra vez "sin contacto"**: 8 gf en el índice al
parar y 0 cuentas de apertura (el actuador no se deja mover: el ángulo no sirve para notar). El par de la muñeca
subió +0.6 N·m 2.5 s después de llegar a la espuma.

- **Error del brazo en la vertical de la yema** (la yema mandada frente a la medida, con la cinemática de la
  preforma): la medida va **8–19 mm por encima** de la mandada todo el rato (error estático y retraso de la bajada),
  así que el apoyo en la espuma no se separa de eso. Para los agarres importa: el brazo se queda ~1 cm por encima de
  donde se le manda (6.3.4).
- **Propuesta del usuario:** el índice más cerrado, para que la carga de la yema vaya contra el actuador fijo (con el
  dedo abierto no cambia la fuerza aunque se apriete la punta). Barrido en el gemelo (rejilla 3 × 3 por mano): con el
  índice en 800 y la mano a 30–55°, **18/18**; en 700 (falange a 55–85°), 16/18; en 600, 5–6/18 (los dedos no
  caben). La preforma `tocar` pasa a 800; `planificacion.toque` admite fijar la falange (`indice_deg`) en vez de la
  mano.
- **`probar_manos.py --presionar A [A ...]`:** el índice quieto en cada ángulo y unos segundos para apretar su yema
  con un dedo: cuánto suben su fuerza y su ángulo, sin mover el brazo. Para elegir el ángulo antes de bajar sobre
  la mesa.
- **La tirada abre las dos manos del todo al empezar** (si no, heredan la postura de la anterior) **y la que tocó al
  volver a la observación** (la mano abierta del gemelo).

## 41. La fuerza del índice sí responde con el dedo quieto: umbral de contacto a 40 gf (2026-11-11)

`probar_manos.py --manos izq --sin-preformas --presionar 1000 900 800 700 --segundos 8`
(`datos/manos_20261111_085636`; el usuario apretó la yema del índice como lo haría la mesa; la de las 08:53 se
descarta, se apretó en otro sitio): leyendo solo la mano, a ~270 Hz.

| índice | sin apretar varía | apretando sube | lecturas > 40 gf |
|---|---|---|---|
| 1000 (abierto) | 0 gf | +68 gf | 58 % |
| 900 | 1 | +203 | 14 % |
| 800 | 0 | +149 (8 lecturas corruptas, hasta −5791) | 42 % |
| 700 | 0 | +142 | 39 % |

**La fuerza del índice es una señal muy limpia** (0–1 gf quieto) y sube con claridad al apretar la yema en cualquier
ángulo. Que en las dos pruebas con espuma no pasara de 8–30 gf apunta a la espuma (blanda: 8 mm aplastados) o a que
no apoyaba la yema, no a que el sensor no responda. `contacto.umbral_gf` pasa de 120 a **40 gf** (con 2 lecturas
seguidas) y `probar_toque.py` lo toma por defecto. El índice se queda en 800 (18/18 puntos de la zona).

## 42. Tercer toque (espuma y madera): el dedo se dobla; el nivel 1 por posición (2026-11-11)

`probar_toque.py --espesor 1` (espuma) y `--espesor 1.8` (tabla de madera de 1.8 cm)
(`datos/tirada_20261111_090457_nivel1` y `_090826_nivel1`), con el índice en 800 y la mano inclinada: **otra vez sin
contacto** (8 y 11 gf en el índice). En las fotos (`montajes/probar_toque2_espuma.jpeg`, `_madera.jpeg`) el índice
real en 800 va **mucho más doblado** que en el modelo (la falange casi vertical o hacia atrás) y toca con el extremo
de la punta: la superficie lo dobla más (hacia la palma) en vez de abrirlo contra su actuador, y la fuerza no sube.
Con la madera, la yema del modelo se quedó a 13 mm (la tabla mide 18): con el dedo flexionado el modelo la pone
≥ 5 mm fuera de sitio. El usuario: los ángulos que se mandan al modelo de la mano no son los de los dedos reales;
corregirlo, en otra sesión.

**Decisión (con el usuario): el nivel 1 por posición.** De la cámara, la mesa (`[g]`) y las dianas; la bajada para
cuando la **yema medida** (encoders del brazo y cinemática, sobre el plano del `[g]`) llega a la superficie
(`contacto.parar_por_posicion`, `altura_toque_m`, `superficie_m`); la fuerza del índice (40 gf, 2 lecturas) queda de
respaldo. Para 6.4.4: el táctil no responde, la fuerza de los dedos no recibe la carga con esta mano y el error del
brazo queda tapado por su desfase de 8–19 mm.

- **El índice extendido y casi vertical** (preforma `senalar`, falange 60–90° bajo la horizontal): el único ángulo
  en que el modelo del dedo es el real, y la punta justo encima de la diana. Con el índice extendido y la mano
  inclinada hacia delante, 11/18 puntos de la zona (el dedo largo se mete sobre las bandejas); casi vertical,
  **17/18** (falla la esquina junto a la bandeja A, x 0.40, y 0.20). Se quita la preforma `tocar`.
- **Detener, no congelar:** el brazo va ~1 cm por encima de su consigna; congelado en lo medido, la yema subiría ese
  cm justo al tocar. `Cuerpo.detener()` para la trayectoria donde va la consigna; orden `detener` del servidor de
  brazos; `ir_a(..., al_parar="detener")` en la bajada. `TOCAR` apunta la yema al parar y 0.3 s después.
- `contacto.superficie_m` (lo que se toca sobre la mesa; `probar_toque.py --espesor` ya lo usa): aproximar y bajar
  se cuentan desde ella.
- En el gemelo, la madera de 1.8 cm: "tocada por posición" con la yema a 18.0 mm.
- Pruebas: `test_brazo_arnes` (detener deja la consigna donde iba; congelar, en lo medido) y `test_supervisor`
  (el toque vertical en cuatro puntos de la zona). 109 de 109.

## 43. El contacto sí se nota (la fuerza BAJA) y la mesa del [g] está ~10 mm baja (2026-11-11)

`probar_toque.py --espesor 1.8` (tabla de madera medida con regla; `datos/tirada_20261111_095646_nivel1`; fotos
`montajes/probar_toque2_madera*.jpeg`) con el nivel 1 por posición del §42: **"sin contacto"**: la yema medida
paró a 22 mm (la tabla, 18) y el usuario vio la punta tocar la tabla y la mano seguir un poco.

- **La fuerza del índice BAJA al tocar** (yo solo miraba si subía): en las cinco pruebas de toque,
  −12 (espuma, 1000), −23 (espuma, 900), hasta −94 (espuma, 800), **hasta −121 (madera, 800)** y −23 gf (madera,
  1000); sin tocar varía 0–1 gf. Con `|fuerza − base| > 10 gf` en 3 lecturas seguidas se nota en todas.
- **La mesa del `[g]` está ~10 mm por debajo de la real:** en las dos de madera (superficie dura) la fuerza lo notó
  con la yema medida a **+9.8 y +10.0 mm** sobre la tabla según el plano del `[g]`. Es la profundidad de la D435i
  (la mesa más lejos y más baja: +1–2 %, el pendiente del Hito 2). Por eso la parada por posición llegaba tarde y
  doblaba el dedo.
- **El brazo va 13–16 mm por encima de su consigna** durante toda la bajada (error estático y retraso).

Cambios:

- **Contacto por `|fuerza del índice − base| > contacto.umbral_gf` (10 gf), en 3 lecturas seguidas**; la parada por
  la yema medida queda de límite (`altura_toque_m`, 3 mm por debajo de la superficie).
- **`montaje.correccion_altura_m: 0.010`** y `manip.montaje.corregir_altura`: en el robot, `manipulacion.py` sube la
  mesa 10 mm a lo largo de su normal (plano, marco y ArUco) para el planificador, el gemelo y el toque. La
  comprobación al empezar sigue con el `[g]` tal cual (la compara con la mesa en vivo, con la misma profundidad).
  No corrige el xy (los ArUco cortados con un plano 1 cm bajo quedan algo corridos hacia el fondo): eso es el
  pendiente de la escala de profundidad.
- `probar_toque.py` mira la fuerza en valor absoluto (5/10/30 gf) y dice dónde estaba la yema medida cuando la notó:
  **la superficie real respecto de la mesa corregida** (lo que habría que sumar a la corrección). Reanalizada la
  de la madera: +9.6 mm sin corrección.
- Pruebas: `test_montaje` (la corrección de altura). 110 de 110.

## 44. Con la mesa corregida, la yema se quedó 3.5 mm encima: la bajada, más honda (2026-11-11)

`probar_toque.py --espesor 1.8` con la corrección de 10 mm (`datos/tirada_20261111_101829_nivel1`; foto
`montajes/probar_toque4_madera.jpeg`): **"sin contacto", la punta a unos milímetros de la tabla**. La yema medida
acabó a 21.5 mm sobre la mesa corregida (la tabla, 18): **3.5 mm por encima, como en la foto: la corrección de la
mesa cuadra**. Lo que faltaba es el brazo: la bajada se mandaba hasta 1 cm bajo la superficie y el brazo va
13–16 mm por encima de su consigna (al empezar a bajar, 74.1 mm medida frente a 58 mandada).

- **`contacto.margen_brazo_m` (2 cm):** la bajada se manda `bajada_max_m` (ahora 1 cm) más ese margen por debajo de
  la superficie (antes, como mucho 2 cm y sin margen); la paran la fuerza del índice (|Δf| > 10 gf) o, de límite, la
  yema medida 3 mm por debajo de la superficie. La velocidad de bajada cuenta el recorrido entero.
- En el gemelo, "tocada" (por posición: allí no hay tabla). 110 de 110.

## 45. Toque en la tabla: la altura ya cuadra; parar donde está la yema (2026-11-11)

`probar_toque.py --espesor 1.8` con la mesa corregida y la bajada más honda (`datos/tirada_20261111_103327_nivel1`):
**"tocada"** (por el límite de posición). El usuario: la punta se apoyó en la tabla sin doblarse unos segundos y
luego la mano bajó un poco más y dobló algo el dedo.

- **La altura de la mesa corregida cuadra:** la fuerza del índice empezó a cambiar (|Δf| > 5 gf) con la yema medida a
  **18.4 mm** (la tabla, 18): 0.4 mm. La corrección de 10 mm vale.
- **Con el índice extendido sobre madera la señal es pequeña:** −6 gf al apoyarse; no llegó a 10 y paró el límite
  (la yema medida a −3 mm de la superficie).
- **Después de parar, la yema bajó de 15 a 11.1 mm:** `detener` deja la consigna donde iba, pero bajando el brazo va
  retrasado respecto de ella (además de su error estático) y, parado, recupera ese retraso contra la tabla.

Cambios:

- **`contacto.umbral_gf` a 5 gf** (quieto varía 0–1 gf; 3 lecturas seguidas): aquí habría parado al apoyar.
- **Parar donde está la yema:** al empezar la bajada (quieto en `aproximar`) se mide el error estático del brazo
  (consigna − medida); al notar el contacto, la consigna pasa a lo medido más ese error (`fijar`, sin trayectoria y
  con saltos ≤ 0.05 rad: `Cuerpo.fijar`, orden `fijar` del servidor de brazos, `BrazoArnes.fijar`, `Brazo.fijar`).
  Ni sube los 13 mm del error estático (congelar) ni sigue bajando lo retrasado (detener). `TOCAR` dice si se fijó.
- Pruebas: `test_brazo_arnes` (fijar: saltos pequeños sí, grandes no). 111 de 111.

## 46. Toque en la tabla y en la mesa: quieto y sin doblarse; la corrección, a revisar (2026-11-11)

`probar_toque.py` con el umbral de 5 gf (`datos/tirada_20261111_104910_nivel1`, tabla, `--espesor 1.8`; y
`_105405_nivel1`, la mesa sin tabla, `--espesor 0`). **En las dos, la punta se apoyó y se quedó quieta, sin doblarse.**

- **Tabla:** "tocada por fuerza" (−8 gf) con la yema medida a **18.2 mm**, y 18.2 mm 0.3 s después.
- **Mesa:** "tocada por posición" a −3.1 mm; la yema bajó luego a −4.7 mm, donde se apoyó (la fuerza no pasó de 1 gf).
  La tabla mide 1.8 cm en total (el usuario), así que **la corrección de 10 mm es buena** (la tabla, a 18.2). En la
  mesa desnuda la yema se hundió unos 5 mm sin fuerza: lo más probable, que el punto (0.48, 0.10) caiga sobre una de
  las alfombrillas grises antideslizantes (goma blanda, como la espuma; la tabla apoya encima y su cara es dura). Una
  diana sobre una alfombrilla se toca hundiéndola hasta el límite de posición (3 mm), con una fuerza despreciable.
- **`fijar` no se aplicó** en ninguna (`consigna_fijada: false`): el servidor de brazos va dentro de `arnes.py`, que
  no se había reiniciado tras desplegar. Aun así, en la tabla la yema no bajó; en la mesa, 1.6 mm. Ahora el supervisor
  avisa si `arnes.py` rechaza `fijar`.

## 47. Fijar con el error de la aproximación bajaba la yema; mantener donde tocó; el dorso al frente (2026-11-12)

`probar_toque.py --espesor 0` con `arnes.py` reiniciado y `[g]` nuevo (`datos/tirada_20261112_060004_nivel1`):
"tocada por posición" (−3.1 mm, sin fuerza), `consigna_fijada: true`, pero **la yema bajó hasta −6.9 mm después
de parar**. Al fijar, la consigna pasó a lo medido más el error medido quieto en `aproximar` (18.7 mm); en la
bajada ese error era ~9 mm (cambia con la postura y con hacia dónde se mueve el brazo): la consigna saltó 9 mm
abajo y la yema la siguió.

- **Mantener donde tocó** (`supervisor._mantener`): al notarlo, la postura medida del contacto, y durante
  `contacto.mantener_s` (1 s) la consigna se corrige cada 50 ms con la mitad de lo que la medida se aparta de ella
  (`fijar`, pasos ≤ 0.04 rad; `BrazoArnes.fijar` pide antes la consigna de ahora). No adivina el error del brazo.
- **El dorso al frente** (pedido del usuario: la mano girada 180° respecto de la prueba, para tocar las dianas):
  `planificacion.toque.palma_hacia: [-1, 0, 0]` y `palma_min: 0.7`, la palma mirando al robot dentro de ~45°
  (antes la orientación era libre: de mirar al robot a mirar al frente). En el gemelo, 17/18 puntos de la zona,
  igual que sin límite.
- Pruebas: `test_brazo_arnes` (mantener: el brazo retrasado y corto se queda en la postura del contacto, con su
  error) y `test_supervisor` (la palma al robot). 112 de 112.

## 48. Nivel 1 con las 4 cruces de cinta: 1 detectada y tocada a 4 cm; detector, recta y bajada guiada (2026-11-12)

`./manipulacion.sh --nivel 1 --arnes` con cuatro cruces de cinta de carrocero (~7.5 cm, brazos de 1.9 cm;
`montajes/nivel1_dianas.jpeg`) (`datos/tirada_20261112_062247_nivel1`): **1 diana detectada** de 4, tocada "por
posición" con el dorso al frente, pero **a 4.1 cm a la izquierda del robot de su centro** y 0.8 cm hacia el robot
(`montajes/nivel1_diana_toque.PNG`; la yema medida, proyectada en la imagen del robot, cae 26 px a la izquierda de
la cruz: cámara y cinemática cuadran). Errores, de la cruz real a la yema:

| | x (mm) | y (mm) |
|---|---|---|
| detección: la cruz cortada por la máscara de la zona | −22 | +19 |
| consigna de la bajada (la recta en articulares entre dos IK) | +5 | −2 |
| el brazo, por detrás de su consigna al bajar (medida − consigna) | +9…+14 | +8…+18 |
| `_mantener` tras tocar (la yema resbaló) | −1 | +6 |
| **total** (yema medida − cruz) | **−8** | **+41** |

- **Detector** (`manip/dianas.py`, `config dianas:`): estaba hecho para pegatinas de 2 cm (`lado_m` hasta 4 cm):
  las cruces de 7.5 cm no pasaban, y la que se vio era el trozo de una cruz que la máscara de la zona (+3 cm)
  cortaba: centro 3 cm movido. Ahora `lado_m [0.01, 0.10]`, se mira la zona ampliada `mascara_m` 0.12 (la cruz
  entera) y se quedan las de centro dentro de la zona ampliada `margen_m` 0.06. Sobre la imagen de la tirada, las
  4: (0.607, −0.012), (0.539, −0.113), (0.497, 0.229) y (0.435, 0.081). Dos están fuera de la zona de recogida
  (x 0.40–0.56, y ±0.20): la del fondo, 4.7 cm más allá (se alcanza con la mano derecha), y la de la izquierda,
  2.9 cm, donde va la bandeja B (roja, (0.56, 0.28)): el índice choca con ella en el gemelo y no hay plan.
- **La bajada en línea recta** (`Cinematica.recta`): una postura cada `paso_recta_m` 1 cm por IK diferencial 6D
  desde la anterior, con la orientación de `aproximar` (si una junta llega a su límite, ese punto con las bandas
  del toque). Con una sola IK en `bajar`, la yema se apartaba de la vertical 5 mm en esta diana y **56 mm** en la
  del fondo con la mano derecha; con la recta, < 2 mm. La vuelta, por la misma recta. Rejilla de la zona con el
  montaje del robot: **18/18** (antes 17: la esquina (0.40, 0.20), ahora con la derecha).
- **Bajada guiada** (`manip/guia.py`, `supervisor._guiar`, `contacto.guiado`): la consigna (con `fijar`, cada
  20 ms) es la postura de la recta a la altura que toca más una corrección que integra lo que la postura medida se
  aparta (junta a junta, `guiado_ganancia` 1.5/s, tope 0.10 rad; el error medido era 0.02–0.035 rad). Quieto en
  `aproximar` hasta que la yema medida está a 2 mm de su sitio (como mucho 5 s), baja a `vel_descenso_m_s`, al
  notarlo se queda guiado a la altura del contacto (`mantener_s`; sustituye a `_mantener`) y sube por la recta a
  `vel_subida_m_s`. Si `arnes.py` no acepta `fijar`, sigue sin guiar por los puntos de la recta. Los eventos
  DESCENSO y TOCAR registran el ajuste y dónde queda la yema respecto de la diana (`yema_diana_*_mm`).
- Gemelo: `Brazo.fijar` manda ya la consigna en modo síncrono (antes solo `ir_a`); `construir(dianas=[(x, y, lado,
  ancho)])`. Pruebas: `test_gemelo` (las 4 cruces de cinta, una pasado el borde), `test_brazo_arnes` (bajada guiada
  con el brazo corto: la yema medida a < 2 mm en aproximar, < 3 mm de la vertical bajando y quieta donde toca) y
  `test_supervisor` (la recta: < 2.5 mm de la vertical). 113 de 113.

## 49. Nivel 1 guiado: 3 de 4 cruces tocadas en su centro; la cuarta, sobre la bandeja B del gemelo (2026-11-12)

`./manipulacion.sh --nivel 1 --arnes` con el detector, la recta y la bajada guiada de §48
(`datos/tirada_20261112_070004_nivel1`; fotos en `montajes/nivel 1/`): **4 dianas detectadas**, 3 tocadas, la
yema en el centro de la cruz (fotos y datos):

| diana (torso) | mano | ajuste en aproximar (xy, mm) | al parar (xy, h) | quieta (xy, h) | fuerza |
|---|---|---|---|---|---|
| (0.435, 0.081) | izq | (8.5, 9.4) → (−3.0, −2.9), 5.0 s (el tope) | (−3.0, −0.4), −3.4 posición | (0.3, 2.6), −6.3 | 18 gf |
| (0.539, −0.113) | der | (2.3, −1.6) → (0.2, −0.8), 4.5 s | (0.0, 4.3), −3.1 posición | (−2.1, 1.2), −5.2 | 129 gf |
| (0.607, −0.012) | der | (−6.2, 0.2) → (−0.6, −0.3), 1.8 s | (0.8, −3.1), −0.5 fuerza | (3.0, −4.6), −2.1 | 53 gf |

- La bajada guiada va a su velocidad (10 mm/s) pero a saltos (rozamiento estático: la corrección se acumula y la
  junta se suelta), con la yema bailando ±5–9 mm en xy; al llegar a la mesa, a ≤ 5 mm. En aproximar, con la mano
  izquierda, el ajuste se pasó (de +9 a −5 mm) y no bajó de 2 mm en 5 s.
- Tras parar, la yema queda 2–3 mm más abajo (la de 129 gf, apretando); dentro del límite de 300 gf.
- **Diana 1** (0.497, 0.229), la cercana de la izquierda: sin plan. Con la izquierda el índice pasa a 5 mm de la
  bandeja B (roja) del gemelo; con la derecha no hay recta. Sin las bandejas en el gemelo, la izquierda la toca
  (orientación del toque). Hoy las bandejas no están en la mesa, pero en el reto esa cruz quedaría bajo la esquina
  de la bandeja B (que ocupa x 0.50–0.62, y 0.22–0.34).

## 50. El brazo baja a saltos (rozamiento): pausa a 15 mm, final despacio y sin apretar al tocar (2026-11-12)

Mirando §49 a 50 Hz: **el brazo baja a saltos** de 4–6 mm a ~30 mm/s: se queda pegado mientras la consigna sigue
bajando (p. ej. la yema quieta en 14 mm mientras la consigna pasa de 7.5 a 0.2 mm) y se suelta de golpe. En la
bajada sin guiar de §48 pasaba igual (de 21.7 a 11.4 mm en 0.5 s): es el rozamiento estático de las juntas, no el
lazo. De ahí los dos detalles:

- **La yema baila ±5–9 mm en xy al bajar**: cada salto mueve varias juntas a la vez.
- **Al parar sigue 2–3 mm más abajo**: la parada cae a mitad de un salto (en la diana 2, (0.435, 0.081), la medida estaba
  2 mm por debajo de la altura objetivo y el hold la puso más abajo aún), y la corrección integrada en marcha lleva la parte
  del rozamiento en movimiento: quieto, sobra y aprieta (la consigna seguía ~10 mm bajo la medida; 129 gf en la
  diana 3).
- En (0.435, 0.081) la parada por posición (−3 mm) no tocaba: la yema bajó a −6.1 y solo entonces 18 gf. Como en
  §46 en (0.48, 0.10): ahí la mesa queda ~6 mm bajo el plano corregido.

Cambios (`contacto`):
- La corrección se integra **solo quieto**: en aproximar (`ajuste_tol_m` 3 mm en xy, antes 2 mm en 3D, que con
  los saltos no se alcanzaba en 5 s) y en una **pausa a `pausa_m` 15 mm** sobre la superficie (`pausa_max_s` 3);
  en marcha, al tocar y al subir se mantiene la de reposo (`Guia.consigna(integrar=False)`).
- El **final despacio**: de la pausa a la mesa a `vel_final_m_s` 4 mm/s (antes todo a 10).
- **Al tocar**, la consigna a la altura medida del contacto con la corrección de reposo: deja de bajar y no
  aprieta más.
- `altura_toque_m` −3 → **−6 mm**: la fuerza para antes donde la mesa está en su sitio (en §49 lo notó a −0.5 mm a
  10 mm/s); la posición, donde está más baja.
- `manos.csv` registra cada lectura de las manos en todas las tiradas (antes solo `probar_toque.py`; en §49
  quedó vacío y no se pudo ver la fuerza al tocar).
- En el gemelo: ajuste y pausa en 0.3 s, tocada por fuerza a −0.3 mm y quieta ahí; `manos.csv` con 1228 lecturas. Pruebas: `test_brazo_arnes` (la bajada y el hold sin integrar no cambian la corrección y siguen la vertical). 113 de 113.

## 51. Con §50, 1 de 4: sin apretar no se nota, congelar la corrección desvía y la consigna saltaba (2026-11-12)

`./manipulacion.sh --nivel 1 --arnes` con §50 desplegado y la cruz cercana izquierda movida
(`datos/tirada_20261112_084915_nivel1`; `[g]` nuevo de las 08:49): dianas en (0.422, 0.024), (0.523, 0.123),
(0.539, −0.106) y (0.606, −0.005). **Solo la 4 bien** (en su centro, 16 gf); la 1 rozada a 2 cm; la 2 y la 3 sin
tocar (el usuario). Con `manos.csv` ya registrado:

- **La fuerza del índice no cambió (0–1 gf) en las dianas 1–3**: pararon por posición a −6 mm sin apretar (la 1, quieta
  a −10.3 mm, rozando). Rozar no da fuerza; apretar sí (16 gf en la 4 a −7.6; 129 gf con 2 mm en §49). El plano del
  `[g]` de las 08:49 queda 3 mm más alto que el de las 05:59 y la mesa medida varía unos mm con el sitio: el límite
  por posición no puede ser el que pare.
- **Congelar la corrección en marcha (§50) desvió la yema** hasta 2 cm en el tramo final (la 1: de (−1.3, −3.0) mm
  en la pausa a (4.9, 20.2) al parar); con ella integrando en marcha (§49), ≤ 5 mm.
- **Diana 3: `fijar` rechazado por un salto de 0.092 rad** (> 0.05): el brazo, pegado, iba 1 cm por encima de su
  consigna; al parar, la consigna a la altura medida subía de golpe 1 cm (el codo cambia 0.05–0.17 rad por cm de la
  recta; no es un cambio de rama). Lo mismo al empezar a subir (0.084). Sin guiar, la vuelta fue punto a punto y
  cada tramo dura como poco `t_min_mov_s` × escala = 16 s: **~3 min sobre la diana**.

Cambios:
- La corrección se integra **también en marcha** (como §49); la pausa a 15 mm y el final a 4 mm/s se quedan.
- `Guia`: cada consigna se aparta de la anterior **como mucho `guiado_salto_max_rad` 0.04** (fijar rechaza > 0.05).
- `altura_toque_m` −6 → **−10 mm**: que pare la fuerza, que sube enseguida al apretar.
- Sin guiar, la bajada y la vuelta **de una vez** (al final de la recta; a aproximar), no punto a punto.
- Pruebas: `test_la_guia_no_salta` (la altura salta de 6 a 0 cm: pasos ≤ 0.04 rad), `test_nivel1_sin_fijar` (sin
  fijar, una bajada y una vuelta, tocada) y `test_bajada_guiada` integrando en marcha. 115 de 115.

## 52. Nivel 1: las 4 cruces tocadas (2026-11-12)

`./manipulacion.sh --nivel 1 --arnes` con §51 (`datos/tirada_20261112_091911_nivel1`): **4 dianas detectadas y las 4
tocadas** (el usuario), con `fijar` aceptado en todas (sin saltos de consigna) y sin respaldo.

| diana (torso) | mano | paró por | fuerza | al parar (xy mm, h mm) | quieta (xy mm, h mm) |
|---|---|---|---|---|---|
| (0.419, 0.025) | izq | posición | 3 gf | (−3.9, 3.9), −10.0 | (0.5, −3.2), −10.9 |
| (0.520, 0.124) | izq | posición | 0 gf | (−7.7, 4.7), −10.0 | (−2.3, −5.3), −12.7 |
| (0.537, −0.106) | der | posición | 1 gf | (1.8, −3.2), −10.0 | (1.5, −2.8), −10.2 |
| (0.604, −0.004) | der | fuerza | 25 gf | (−1.6, 0.7), −8.3 | (2.0, −5.9), −10.2 |

- La yema quedó a ≤ 9 mm del centro de la cruz (dentro del brazo de cinta de 1.9 cm).
- Tres pararon por posición a −10 mm con 0–3 gf: el dedo llegó a la mesa pero rozando, sin apretar, y la fuerza del
  índice no lo nota. Con el `[g]` de hoy la mesa real queda unos 10 mm por debajo del plano corregido en esa zona:
  la corrección de altura (`correccion_altura_m` 10 mm, medida con la tabla en §43-45) habría que revisarla con
  cada `[g]`. Pendiente.

## 53. Hacia el nivel 2: toque de prueba, agarre guiado y la esfera gris (2026-11-12)

**Toque de prueba** (`./manipulacion.sh --medir-mesa --arnes`, `prueba_mesa:`). La altura de la mesa cambia unos mm
con cada `[g]` (el de las 08:49, 3 mm más alto que el de las 05:59) y con el sitio; con la corrección fija de 10 mm,
en §52 la mesa real quedaba a ~−10 mm del plano corregido. Ahora la yema de cada mano baja en un punto libre
(`puntos`: (0.48, ±0.10); se salta si hay un objeto a menos de `libre_m` 8 cm) hasta que **la fuerza** del índice la
nota, como mucho a `limite_m` −25 mm (rozando la fuerza no cambia; apretando sube enseguida). Lo que la yema quede
bajo el plano se suma a la corrección; la media de los dos puntos (aviso si difieren más de 6 mm) se guarda en
`config/montaje.yaml` como `mesa.toque` (y en la carpeta de la tirada). El siguiente `[g]` reescribe la mesa y la
borra; al empezar, cada tirada dice qué corrección usa ("altura de la mesa: …"). Un punto que para por posición no
vale. En el gemelo: los dos por fuerza, corrección ~0, nada guardado.

**Agarre guiado.** Los tramos finos de coger eran movimientos sueltos entre IK, con la desviación del toque antes
de §48 (hasta 56 mm) y el error de 1–2 cm del brazo, suficiente para tumbar un cubo de 6 cm. Ahora (como §48–51):
- `Planificador._rectas_agarre`: de aproximar al agarre (calculada desde el agarre hacia atrás, así acaba justo en
  él; de lado sustituye a `entrar`), levantar, bajar a la bandeja y retirarse (la bajada al revés), en línea recta
  del punto de agarre con la orientación del agarre, una postura por `paso_recta_m` 1 cm, a `vel_recta_m_s`
  (agarrar 0.01, levantar y bajar 0.02, retirar 0.03). La del agarre es obligatoria; con la orientación fija, la de
  levantar (13 cm el cilindro) o la de la bandeja a veces llegan a los límites de las juntas: entonces levantar va
  en recta solo los primeros `despegue_m` 5 cm y de ahí a la IK, y la bandeja como antes (3 cm de holgura por
  lado). `sobre_bandeja` se resuelve desde donde acaba levantar: desde la IK suelta, el tramo bajaba a 13.9 cm de la
  mesa (seguridad 15) y el cilindro se quedaba sin plan.
- `Supervisor._recta_guiada`: cada recta con `Guia` (ahora para cualquier recta: `Guia.por_puntos`; empieza con la
  corrección del tramo anterior y su última consigna, sin saltos). Al agarre: ajuste quieto en aproximar (en la
  recta, `ajuste_tol_m` en perpendicular), recta y ajuste en el punto de agarre; el evento CERRAR registra dónde
  quedó la mano (`mano_desde_el_plan_mm`). Con el objeto, `sujetar` en cada ciclo. Sin `fijar`, de una vez.
- En el gemelo, nivel 2 2/2 en sus bandejas; los tramos a < 2.5 mm de su recta.

**La esfera gris** sustituye a la pelota de tenis (`objetos.yaml`: `esfera`, 6 cm, 41.4 g, base aplanada; misma
bandeja). Masas: cilindro 32.9 g, prisma 47.8 g (el usuario). Mide como el cubo (6 cm) y **la forma no los separa
con la profundidad real**: probé la fracción de la mitad de arriba a < 8 mm de lo más alto (en el gemelo, esfera
0.41 frente a 0.78–0.87 de las tapas planas), pero en las capturas reales del 18-oct el cubo y el cilindro dan
0.24–0.46 y la pelota 0.42: descartado. Los separa el color: el cubo blanco en la cámara del robot tiene S 4 (5–95 %:
4–37) y V 255; la esfera es azulada (H ~110) y algo saturada (S ~39 en el gemelo). Cubo S ≤ 14 (sigue reconocido
en los 48 fotogramas reales del 18-oct), esfera H 90–130, S 15–90. **Rangos estimados: hay que ajustarlos con
imágenes de la cámara del robot.**
- Pruebas nuevas: `test_medir_mesa_en_el_gemelo`, `test_coger_en_linea_recta` (los cuatro tramos a < 2.5 mm de su recta) y la esfera en `test_percepcion` (sintética; en la captura real del 9-oct, el cubo no sale esfera). 117 de 117.

## 54. Toque de prueba en el robot y el color de la esfera (2026-11-12)

- **Bandejas**: con el `[g]` de la mesa de las 10:36 (con las bandejas puestas; el plano se ajusta sin su huella),
  B y D guardadas (medidas); A y C no se ven con los brazos a 0 (las manos las tapan, como el 10-nov): colocadas con
  regla (`distancias_bandejas.py`), el robot usa su objetivo.
- **`--medir-mesa`** (`datos/tirada_20261112_104248_medir_mesa`): **los dos puntos pararon por fuerza**: izq en
  (0.48, 0.10) a −13.1 mm del plano corregido (54 gf; quieta a −15.4), der en (0.48, −0.10) a −10.1 mm (11 gf).
  Corrección de este `[g]`: +10.0 → **−1.6 mm** (el plano del `[g]` queda 1.6 mm por encima de la mesa real). Entre
  los dos puntos, 3 mm.
- **Color de la esfera** (`datos/dataset_20261112_104835_esfera_cubo`, 50 fotogramas, la esfera a la izquierda y el
  cubo a la derecha): en la cámara del robot la esfera sale más oscura que en las fotos: H 93–107, S 24–59,
  V 47–136 (5–95 %); el cubo S 4–8 (sus caras en sombra, cálidas, hasta 75) y V 255. Ya con los rangos de §53, 50/50
  esfera y 50/50 cubo; con el de la esfera ajustado a lo medido (H 85–115, S 15–90, V ≥ 35) la esfera tiene 0.94 de
  su color (antes 0.73: el V ≥ 90 dejaba fuera la sombra) y 0 del cubo; el cubo, 0.53 del suyo y 0 del de la esfera.
- El toque de prueba ya no se anuncia como "Nivel 0" ni habla de dianas.

## 55. Primer nivel 2 en el robot: el cubo visto con poca confianza (2026-11-12)

Tres `./manipulacion.sh --nivel 2 --arnes` con el cubo en la mesa (`datos/tirada_20261112_105616_nivel2`,
`…105756…`, `…105933…`): "no veo ninguno de [...]". En las dos primeras **sí lo vio, como cubo, con confianza 0.31 y
0.41**, por debajo de `percepcion.confianza_min` 0.5; en la tercera, en (0.595, −0.044) (3.5 cm pasada la zona),
"desconocido". Causa: el rango del cubo de §53 (S ≤ 14, para separarlo de la esfera) deja fuera sus caras en sombra,
cálidas (H 11–53, S hasta 75): solo la mitad de sus puntos casaban con su color. Ahora el cubo es H 0–60, S ≤ 80,
V ≥ 60 (lo separa de la esfera el tono, 93–107): confianza del cubo 0.48 → **0.84** en `esfera_cubo` (mínimo 0.79) y
0.54 → 0.81 en `objetos_a` del 18-oct; esfera, prisma, cilindro y pelota igual que antes (todos bien).

- Cada observación guarda su imagen (`observar_N.png`): en estas tiradas no quedó ninguna.
- El aviso dice lo que se vio y con qué confianza ("vistos: cubo (0.41) en (…)").

## 56. El modelo nuevo de las manos: cero en la mano abierta y acople medido (2026-11-12)

El paquete `h1_2_inspire_description` trae la revisión de las manos (colores, conector y **acople por software**,
sección *Software coupling* de su README): `q = 0` es la mano abierta real (ANGLE_SET 1000) en las 12 juntas de
cada mano, y cada falange sigue su propia curva medida en nueve fotos por DOF (`config/hand_coupling.yaml`,
`scripts/hand_coupling.py`). Antes el cero era el del manual, la tabla ANGLE_SET → rad tenía tres puntos y las
falanges iban con `<mimic>` lineal (0.85, 0.40, 0.60).

- **El reto usa el URDF sin `<mimic>`** (`h1_2_with_RH56DFTP_hands_software_coupling.urdf`). `Cinematica` pone las
  12 juntas de cada mano con las curvas (`juntas_mano`), y `angle_set_a_rad` sale de ellas (antes, en `sim/escena.py`,
  la tabla de tres puntos). En el gemelo, `Estado.poner_mano` pone las 12 juntas con esas mismas curvas, sin `Mimic`.
  En física (`cuerpo_simulado`), las falanges siguen una restricción polinómica de MuJoCo, a menos de 0.007 rad de
  la curva. `desplegar.sh` lleva al PC2 `scripts/hand_coupling.py` y `config/`.
- **La yema del índice abierto** queda en (0.2907, ∓0.0139, 0.0358) en `mano` (antes 0.2914, ∓0.0085, 0.0358):
  **5.4 mm más hacia la palma**, que en el toque, con la palma abajo, es hacia abajo (el índice real a ANGLE_SET 1000
  está 0.057 / 0.011 rad doblado respecto del cero del manual). El cuerpo, la muñeca y el marco `mano` no cambian
  (Pinocchio: los 116 frames del URDF anterior en q y del nuevo en q − desplazamiento coinciden a 6e-12 m; masa
  68.664 kg las dos). **La corrección de la mesa (`--medir-mesa`, §54) se midió con la yema anterior: hay que
  repetirla.** La punta del pulgar se mueve 3–7 mm según la preforma; el punto de agarre está en `mano` y no cambia.
- **Toque:** en (0.40, −0.20), la esquina junto a la bandeja C, con la orientación de la configuración el índice
  pasa a 5 mm de la bandeja: el planificador usa la relajada (palma al robot ≥ 0.4 en vez de 0.7).
  `test_el_toque_con_el_indice_vertical` la acepta solo ahí. Las cuatro cruces del nivel 1 (§48–52) se planifican
  igual que con el modelo anterior (mano y orientación).
- **Fuera del reto:** `h1_2_mujoco` (editor, modo física, `ver_manos.py`) usa el mismo URDF.
  `selector_poses_manos/conversion_angle_set.py` lleva las curvas (q_max 1.517 / 0.624 / 1.200), y las rutinas se
  convirtieron conservando el ANGLE_SET que manda el selector.
- Pruebas: 117 de 117 (en HEAD, antes del cambio, también 117).

## 57. Nivel 2 en el robot: el cubo cogido girado y la vigilancia del escape (2026-11-12)

`./manipulacion.sh --nivel 2 --arnes` (`datos/tirada_20261112_231953_nivel2`, con el modelo nuevo de §56 y su
`--medir-mesa`: +2.8 mm). El cubo, visto con 0.88 en (0.522, −0.015), se planificó con la mano derecha y se cogió con
tres dedos (`agarre comprobado`: medio 214, índice 257 y pulgar 563 gf), pero **se escapó a los ~2 s de levantar** y
quedó 3 cm a la derecha, en (0.526, −0.044). El brazo lo "llevó" a la bandeja D y lo dio por colocado. Fotos del
usuario en `montajes/nivel 2/`: la mano en diagonal sobre el cubo.

- **La mano iba girada ~35°** respecto de las caras del cubo, y la palma a 34.9° de la vertical (su tolerancia, 35).
  Sale de la FK de la q medida y de la consigna: el ancho de la mano (meñique → índice) a 125° en planta. El cubo
  estaba alineado con la mesa (`observar_1.png`). Dos causas:
  1. El planificador pedía el ancho de la mano a lo largo de una cara con 20° de tolerancia (y la del objeto, 30°, si
     no había plan) y la palma con 35°. **La IK se para en cuanto entra en las bandas**, así que el agarre queda en su
     borde. En el gemelo, con el cubo en el mismo sitio: 8–19° del cubo y la palma a 30–33°.
  2. **El yaw del cubo salía de un solo fotograma de profundidad** (el rectángulo mínimo de la tapa). Con el cubo
     quieto, de −12 a +21° (`esfera_cubo`, 50 fotogramas, desviación 6°). No se registraba.
- **Guiñada en escalones** (`planificacion.guinada_tols_deg: [5, 10, 15, 20]` y luego la del objeto): gana la primera
  tolerancia con plan. En el gemelo, en 6 puntos de la zona con el cubo a −4, 0 y 20°, el agarre queda a 2–7° del cubo
  en casi toda la zona. En el centro (0.522, −0.015) queda a 10.5° con el cubo a 0° y a 17° con el cubo a −4°: es el
  punto más difícil para la derecha (más estricto, el hombro roza el torso). Planificar tarda 2–9 s en la PC (antes
  2–3). Con la palma también a 15–20°, la derecha casi nunca tiene plan: la palma se queda en 35°.
- **Observación con 5 fotogramas** (`percepcion.fotogramas_observar`, `Percepcion.procesar_varios`): por clase, la
  mediana de la posición y el yaw más central (módulo la simetría); lo visto en menos de la mitad de los fotogramas no
  cuenta. El yaw del cubo con 5 fotogramas: −3.2…−3.8° en `esfera_cubo` y −1.4…−6.4° en `objetos_a_regla` (con uno:
  −12…+21° y −12…+1°). En los tres datasets el cubo sale a ~−4°; si es un sesgo de la calibración, no se ha
  comprobado. El evento OBSERVAR registra el yaw (en grados) y cuántos fotogramas se juntaron.
- **La vigilancia del transporte no saltó.** Pedía que la suma de las fuerzas de los cinco dedos bajara del 30 % de la
  del cierre, pero en `tres_dedos` el meñique y el anular recogidos marcan ~460 gf sin tocar nada: la suma bajó de
  1462 a ~750 gf. Tras el escape, en ~0.6 s, el medio cerró hasta 1, el índice hasta su consigna (487) y el pulgar
  hasta 305. Ahora `Mano.escapado`: el pulgar, o todos los dedos que agarran, en su consigna ("vacío" de `agarre`) en
  dos lecturas seguidas. El brazo no se para, porque sigue el plan comprobado:
  - si se escapa al levantar, abre arriba, vuelve a la observación y el intento cuenta como fallido (se reintenta,
    `intentos_por_objeto` 2);
  - si se escapa más tarde, abre sobre la bandeja y también cuenta como fallido.

  Antes era una Parada (salida segura) que no llegaba a saltar.
- Pruebas nuevas: `test_el_objeto_que_se_escapa` (mano simulada), `test_varios_fotogramas_se_juntan`,
  `test_el_cubo_se_coge_alineado` (a ≤ 10° del yaw visto) y `test_el_cubo_que_se_escapa_al_levantar_se_reintenta`.
  121 de 121.
- Pendiente: probarlo en el robot. Si el agarre alineado aún resbala, cerrar con los cuatro dedos y el pulgar (la otra
  opción del usuario; en banco, el cubo con tres dedos fue estable a ~300 gf, §34).

## 58. Segundo cubo: la pinza de tres dedos no estaba centrada en el cubo (2026-11-13)

`./manipulacion.sh --nivel 2 --arnes` con §57 (`datos/tirada_20261113_003250_nivel2`). El cubo, visto con 5
fotogramas en (0.514, 0.000) y yaw 0.7°, se planificó en 9.2 s (intento 10 de la IK: tolerancia 20°). Lo cogió y lo
levantó "a las justas" (el usuario), y **se escapó llegando a la bandeja D**. La vigilancia nueva lo vio
("cubo se escapa de la mano" a los 116.6 s) y abrió sobre la bandeja: intento fallido. En la siguiente observación ya
no había cubo en la zona.

- **La mano, a 17° del cubo** (ancho a 107.5° en planta, palma a 33°): en el centro de la mesa (y ≈ 0) la derecha no
  alinea mejor (§57).
- **El índice resbaló por el borde.** Al cerrar: medio 723 (435 gf), índice 615 (314 gf), pulgar 391 (557 gf). Al
  levantar, el índice siguió cerrando hasta su tope (487, en ~4 s) y desde ahí el cubo quedó sujeto solo por el medio
  contra el pulgar (650/550 gf), que es la pinza de dos dedos que en banco lo hacía girar (§34). Aguantó 17 s.
- **Causa: el punto de agarre era el de los cuatro dedos.** `agarres.arriba` pone el centro del objeto en z +0.010
  (hacia el índice), centrado en los cuatro dedos (de −0.029 el meñique a +0.034 el índice). Con `tres_dedos` el
  meñique y el anular van recogidos, y en las mallas, con la mano cerrando sobre el cubo, las yemas quedan en z +0.034
  el índice, +0.011 el medio y **+0.032 el pulgar** (enfrente del índice, no del medio). El cubo de 6 cm iba de −0.020 a
  +0.040: el pulgar y el índice apretaban a 6–8 mm de su borde, y la pinza (del pulgar a la resultante de índice y
  medio) pasaba a ~1.6 cm del centro de masas. El usuario lo vio así: los dedos deben tocar el cubo centrados, no a
  un lado. En la mano izquierda las yemas están en las mismas z.
- **Ahora** `objetos.yaml` admite un centro por objeto (`centro_agarre`, `planificacion.centro_agarre`). El cubo y
  el prisma (los dos con `tres_dedos`) lo tienen en z **+0.026**: la pinza, con las fuerzas medidas (resultante de
  índice y medio en +0.021, pulgar en +0.032). El índice queda 2.2 cm dentro de la cara y el medio 1.5 cm. La esfera
  y el cilindro siguen con el de los cuatro dedos.
- En el gemelo, la alineación también mejora porque la mano se aparta 1.6 cm: en el sitio de §57, (0.522, −0.015), de
  10–17° a 5–8°; en el de esta tirada, con la derecha, 12.5° (en el robot fueron 17°). En el resto de la zona, 2–12°.
- **La tirada usó la corrección de la mesa de la configuración (+10 mm), no la medida.** El `[g]` nuevo borra el toque
  de prueba del anterior (+2.8 mm): sin repetir `--medir-mesa`, el agarre pudo ir hasta ~7 mm más alto en el cubo.
- Si aún resbala: cerrar con los cuatro dedos y el pulgar (`abierta_opuesta`, FORCE_SET en los cuatro dedos y sin
  `centro_agarre`), la otra opción del usuario.
- Pruebas: 121 de 121 (las de coger y el gemelo, con el centro nuevo).

## 59. Tercer cubo: colocado, pero con la mano 2.5 cm a un lado; el cubo, con la tapa en el color (2026-11-13)

`./manipulacion.sh --nivel 2 --arnes` tras `[g]` y `--medir-mesa` (−0.2 mm; `datos/tirada_20261113_011538_nivel2`).
El cubo, visto en (0.541, −0.057) a −0.9°, se planificó en 2.7 s (intento 1). **Lo cogió, lo llevó y lo dejó en la
bandeja D**, pero solo con el pulgar y el índice: la mano iba bien girada y a la derecha del cubo, y el medio quedó en
su borde (foto del usuario `montajes/nivel 2/nivel2_pose2_agarre_cubo.jpeg`; en ella, el centro del cubo a ~2 cm del
de la pinza, hacia el índice). El brazo hizo el plan: la mano a 2.4 mm de él (`mano_desde_el_plan_mm`).

- **La calibración cámara–brazo está bien** en esa dirección. El marcador del dorso derecho, detectado en
  `agarre_cubo_cerrado.png` de §57 (a ×2), queda a 1.6 px (~2 mm) del predicho con la q medida a lo ancho, y a 7 px
  (~9 mm) en vertical. En la tirada de hoy no se detecta.
- **La profundidad pone mal el cubo blanco.** Sus puntos sí caen sobre el cubo en el color (el extrínseco de fábrica
  está bien), pero la tapa tiene un hueco en medio y alturas de 2 a 6 cm (en planta, una U). El centro del rectángulo
  de los puntos más altos (`h > h_max − 12 mm`) cae donde la tapa sale más alta, y el yaw sale a −4° con el cubo
  recto. Lo comparé con un ajuste de la tapa en el color (un cuadrado de 6 cm a 6 cm sobre la mesa, por IoU con lo
  claro saturado, 0.83–0.97): la profundidad lo ponía +10/+17 mm en `esfera_cubo`, +3/+20 en `objetos_a` y +11/+3 en
  `objetos_a_regla` (x/y, hacia el robot y hacia su derecha).
- **Las tres tiradas, con la tapa en el color:** la primera, +13 mm a la izquierda del robot de donde la vio la
  profundidad (más los 35° de §57); la segunda, −2 mm (la posición estaba bien y falló el centrado de la pinza, §58);
  la tercera, **+25 mm**: lo que muestra la foto.
- **Ahora** (`objetos.yaml` `tapa_color`, solo el cubo; `Percepcion._tapa_en_color`): tras la profundidad se busca en
  el color la mancha clara (V ≥ 235, S ≤ 60) más cercana a donde la puso. Sus píxeles se retroproyectan al plano de
  la tapa (la mesa subida el alto del objeto), y su rectángulo mínimo da el centro y el yaw. Vale si mide 0.75–1.25
  veces la tapa y está a menos de 4 cm de la profundidad; si no, se queda lo de la profundidad. Las cruces de cinta y
  la bandeja blanca salen como manchas aparte. Con varios fotogramas, si la mayoría tienen tapa, solo esos. El evento
  OBSERVAR dice "tapa en color".
  - Contra el ajuste por IoU: 1–3 mm y 1–3° en los tres datasets, refinados todos.
  - En el gemelo: 2.4/1.0 mm y el yaw exacto.
  - El yaw del cubo recto sale a 0–2° (no −4°).
- Prueba nueva: `test_la_tapa_del_cubo_en_color_corrige_la_profundidad` (estimación movida 2 cm y girada 10°: vuelve
  a < 4 mm y 2°).
- El prisma (naranja) también es caja y su profundidad no se ha comparado con el color; se podría refinar igual con
  su color.
- Pruebas: 122 de 122.

## 60. Cuarto cubo: colocado con los tres dedos; el pulgar cedía. La cámara externa (2026-11-13)

`./manipulacion.sh --nivel 2 --arnes` con §59 (`datos/tirada_20261113_014820_nivel2`). El cubo, con la tapa en color
en (0.547, −0.041) a −0.7°, se cogió con los tres dedos (al cerrar: medio 439, índice 281 y pulgar 548 gf, los tres
"lleno") y **se dejó en la bandeja D** al primer intento. El usuario vio el agarre más centrado, pero aún con un
pequeño desfase hacia la izquierda del robot, y el cubo resbaló un poco al levantarlo.

- **El pulgar cedía.** Durante el transporte, el índice y el medio siguieron cerrando (736→614 y 705→590) y el pulgar
  se abrió (415→595), con su fuerza subiendo hasta ~800 gf: los dos dedos (300 + 300 gf) empujaban el cubo contra un
  pulgar de 350. El FORCE_SET escrito es el nominal más ~225 gf de base y residuo (en la derecha: 523/533/575), con el
  tope de 800. **Ahora el cubo cierra con `[-1, -1, 300, 300, 575, -1]`**: el pulgar como los dos dedos juntos (575 + su
  compensación = 800, el tope). El prisma sigue con 350 (va con la izquierda; sin probar).
- El desfase de unos mm no se puede medir con lo que hay (la cámara del robot ve el agarre desde arriba y la mano lo
  tapa): se medirá con la cámara externa.

**Cámara externa** (`herramientas/camara_externa.py`, `camara_externa:` en `manipulacion.yaml`). Es una Logitech BRIO
en la laptop, de frente al robot, a ~1.4 m del cubo y elevada.
- **Conexión:** va en USB 3 (en USB 2 solo daba 1080p).
- **Encuadre:** 1080p con el zoom de la cámara ×2.18 sobre su sensor 4K (~1 píxel del sensor por píxel, ~0.6 mm/px en
  el cubo) e inclinación digital −10° (su tope). Se ven las manos en la observación, la zona y las cuatro bandejas.
- **Foco y velocidad:** foco fijo en 5 (de 0 a 20 la nitidez es la misma; el automático busca al pasar el brazo) y
  30 fps fijos.
- **Sincronía:** cada fotograma lleva su instante real (`-use_wallclock_as_timestamps`, `-vsync passthrough`; el
  primero, del `start:` de ffmpeg). El desfase del reloj del robot se mide por ssh al empezar (+0.04 s, 3 ms de ida y
  vuelta; la hora de 13 h de diferencia era la zona horaria del robot, UTC+8). El resumen de cada tirada guarda ahora
  `inicio_epoch` y `t0_monotonic`: cualquier t de sus CSV pasa a la hora del vídeo.
- **`manipulacion.sh` desde la laptop:** si la cámara está conectada, graba mientras dura la tirada
  (`datos/externa/externa_<fecha>.mp4` y `.json`, H.264 ~1–4 Mbit/s). Al acabar trae del robot la última tirada y
  deja en `datos/<tirada>/externa/`:
  - una foto por evento (el posicionamiento en CERRAR, el cierre, a +1.5, +3 y +6 s del transporte, al soltar y
    +1.5 s);
  - `mosaico.jpg` con todas;
  - `agarre_N.mp4` de cada agarre (del descenso a 3 s después de soltar).

  `CAMARA_EXTERNA=0` lo apaga; `camara_externa.py ver` guarda una foto del encuadre.
- La ffmpeg de la laptop es la 4.4 (`-vsync`, no `-fps_mode`).

## 61. Quinto cubo: la pinza por debajo del centro de masas; el corte de fuerza lo soltó (2026-11-13)

`./manipulacion.sh --nivel 2 --arnes` con el pulgar a 575 (§60), grabado con la cámara externa
(`datos/tirada_20261113_023622_nivel2`, `externa/`). El cubo (tapa en color, (0.525, 0.007)) se cogió bien centrado
(en el vídeo, el medio y el índice a los lados del centro de la cara): al cerrar, medio 467, índice 463 y pulgar 735 gf.
A los 3.4 s de levantar (~7 cm) saltó **"fuerza de la mano 1571 gf > 1200 en 3 lecturas seguidas"**: la salida
segura abrió la mano y el cubo cayó casi en su sitio.

- **El cubo volcaba dentro de la mano.** En el vídeo, al cerrar, las yemas del medio y del índice aprietan la cara en
  su parte baja y el pulgar, detrás, más arriba; al levantar, el cubo va girando en la pinza. Con la postura medida
  del brazo y los ángulos de los dedos, las yemas quedan bajo el punto de agarre: el medio 31 mm, el índice 24 y el
  pulgar 9. Con el punto a 35 mm de la mesa, el medio y el índice apretaban a 4–11 mm y el pulgar a ~26: **la pinza,
  por debajo del centro de masas (30 mm)**, un péndulo invertido. Es también el "resbaló un poco" de §60.
- El pulgar, con su FORCE_SET en el tope (800 escrito), marcaba 770–830 gf y volvía a empujar. Al girar el cubo,
  avanzó de 399 a 312 en 0.4 s y se clavó (1439–1571 gf; el medio, 925). Además dio un escape falso: estaba en su tope
  de cierre (300 + 60), pero apretando.
- **Cambios:**
  - el cubo, `altura_agarre_m` 0.035 → **0.055**: el medio a ~24 mm, el índice a ~31 y el pulgar a ~46, con la pinza
    ~7 mm por encima del centro de masas;
  - el descenso final empieza como poco `descenso_agarre_m` 2.5 cm sobre el agarre (`altura_aproximacion_m` 6 cm
    lo dejaba en 5 mm): el cubo baja desde 8 cm, el cilindro desde 7.5 y lo demás igual;
  - pulgar del cubo **450** (escrito 675: margen frente al tope y al corte);
  - `Mano.escapado`: un dedo "vacío" de `agarre` solo cuenta si además marca menos de la mitad de su FORCE_SET, sin
    su residuo de flexión. El pulgar de esta tirada (312, 1526 gf) no da escape; el escape de §57 sí; el agarre de
    §60 no.
  - `test_coger_en_linea_recta` medía cada recta (el punto de agarre) contra la línea de su primer punto a su último.
    Con el cubo a 5.5 cm, al final de levantar el codo se estira (de 0.23 a −0.13 rad) y la orientación de la mano gira
    1.5–3° dentro de sus bandas (12°): la muñeca sigue recta, pero el punto de agarre, a 23 cm, se aparta 6–11 mm en
    los dos últimos centímetros. Ahora mide contra la normal de la mesa (por donde van las cuatro), con los mismos
    2.5 mm, y de levantar solo el despegue (`despegue_m`, 5 cm), que es lo que el planificador garantiza.
- **La cámara externa:** el usuario la inclinó a mano; con la inclinación digital de −10° quedaba demasiado baja (el
  tablero cortado arriba). Ahora `tilt_absolute: 0`: las manos en la observación, todo el tablero y las bandejas; el
  cubo, ~135 px.
- El prisma (4 cm de alto, `altura_agarre_m` 0.030, `tres_dedos`) tendría el mismo problema: el medio quedaría a la
  altura de la mesa. Sin cambiar ni probar.
- Pruebas: 122 de 122.

## 62. Marcadores para situar la cámara externa (2026-11-13)

Para un script que diga cuánto mover la BRIO (altura, adelante/atrás, inclinación y giro) con el robot en reposo,
hay que situarla respecto de la mesa. **Los ArUco de las esquinas no sirven desde ella**: en 4K y sin zoom no se
detecta ninguno, ni con el detector más permisivo ni ampliando la imagen. Los dos de su lado (2 y 3) se ven muy de
canto y el labio frontal (2.4 cm) tapa su borde cercano; los dos del lado del robot (4 y 5), las bandejas blanca y
azul.

- El usuario puso dos de 5 cm en el poste delantero de cada asa, mirando a la cámara: se detectan bien (80–90 px en
  4K), pero con los ID 0 y 1, que son los del dorso de las manos (calibración cámara–torso), y la BRIO también ve las
  manos. Solos, además, dan mal la altura y la inclinación (casi en un mismo plano vertical y de 5 cm) y no permiten
  estimar el campo de visión.
- **Propuesta** (`docs/aruco_6_7_camara_externa.pdf`, con línea de corte y regla de control de 100 mm):
  - ID 6 y 7, de **7 cm**, planos sobre el tablero con el centro a **11 cm del labio frontal** (lado de la cámara) y a
    24 cm de cada labio lateral (el 6, a la izquierda del robot), con 1 cm de blanco (el tablero es negro: sin el
    blanco no se ve el contorno). Ocupan de 33.3 a 42.3 cm desde el labio del robot (fondo interior 48.8):
    - 1.5 cm por fuera de la zona de recogida ampliada, que llega a 31.8 cm (x 0.61 en el torso);
    - con el blanco a 6.5 cm del labio frontal, que tapa ~6.6 cm detrás de él con la cámara a 20° bajo la horizontal
      (~4.2 cm a 30°).

    La primera propuesta (10 cm, 1.5 de blanco y centro a 15 cm) se metía 4.5 cm en la zona ampliada: lo vio el
    usuario. Su blanco quedaba al alcance de la tapa del cubo en color y del detector de las cruces del nivel 1.
  - ID 8 y 9, de 5 cm, en las asas en lugar del 0 y el 1, con 0.5 cm de blanco.

  Todo el código del robot filtra los ArUco por ID (0–1 manos, 2–5 mesa) y la percepción no ve lo que mide menos de
  12 mm: los nuevos no le afectan.
- Colocados y con su script: §63.

## 63. Donde poner la cámara externa: `situar_camara_externa.py` (2026-11-13)

El usuario colocó el 6 y el 7 como en §62 y el 8 y el 9 en los postes delanteros de las asas (el blanco de abajo a
2.5 cm de la base), con el robot en reposo en el arnés y un montaje nuevo (07:28).
`python3 herramientas/situar_camara_externa.py` (en la laptop) dice cuánto mover la BRIO, en vivo, mientras se mueve
(`--una`: una medida; `--ventana`: la imagen anotada; `--guardar`: el zoom que encuadra, en `manipulacion.yaml`).

- **La pose de la cámara.** En 4K sin zoom, con el 6 y el 7 en su sitio de wincha (marco físico de la mesa: origen
  en el centro del labio del robot, x hacia el labio frontal). Prueba los cuatro giros de cada uno, porque al
  recortarlos se pierde cuál era su arriba (estaban con el arriba del marcador hacia la cámara).
  - Error de 0.7–0.8 px con los cuatro.
  - La focal, 2211 px estimada con el 6 y el 7 (los 90° de diagonal de la BRIO dan 2203): `focal_4k_px: 2210`.
  - El 1080p con `zoom_absolute` Z es el centro de la imagen 4K / Z (con 218, ×2.18 justo, por correlación).
  - Las bandejas del montaje, proyectadas con esa pose, caen sobre las reales.
- **El 8 y el 9** se sitúan la primera vez desde el 6 y el 7 (`datos/externa/marcadores_asas.json`, mediana de 10
  fotogramas). El negro, con el borde de abajo a 3.0 cm (el 8) y 2.3 cm (el 9) sobre la mesa.
  - Medidos así, desde otra vista se apartan 3–6 px: de un ArUco de 5 cm solo, la profundidad y el giro salen peor.
  - Con ellos, el error de la pose subía de 0.7 a 1.3 px (0.3° de giro). Ahora solo se usan si falta el 6 o el 7.
- **Lo que se quiere ver:** la zona de recogida ampliada (5 cm) y las cuatro bandejas, de la mesa a 20 cm (luego
  30).
  - **Altura y adelante/atrás.** La elevación de la cámara vista desde el centro de la zona, 25–40° (ideal 32°): desde
    25 se ven las tapas, el fondo de las bandejas cercanas y la profundidad del agarre; por encima de 40, la mano que
    baja tapa los dedos. El zoom con el que la zona y las bandejas llenarían el 1080p (crece con la distancia),
    ×1.4–3.0 (ideal 2.2): por debajo, el vídeo no gana detalle y crece la perspectiva; por encima de 3 (más de
    ~1.5 m), el cubo tiene menos de ~90 px del sensor.
  - De cada una se da su rango con la otra como está, y lo que hay que mover: lo más corto que entra en el rango
    (solo la altura, solo la distancia o las dos al sitio ideal), con 3 cm de margen.
  - **Inclinación y giro:** los que centran la zona y las bandejas en la imagen desde donde está la cámara (±2°).
    También dice el ladeo si pasa de 2°.
- **Ahora** la cámara está a 59 cm sobre la mesa, a 95 cm del labio frontal y 6 cm a la derecha del eje (mirando
  hacia el robot), con 14.8° hacia abajo, 3.5° a la izquierda y 3.1° de ladeo. Se ve a 24° de elevación y a 1.37 m
  (zoom ×2.72). Indicaciones:
  - subirla 6 cm (bien de 62 a 87 cm);
  - la distancia, como está;
  - inclinarla 7.3° más hacia abajo;
  - el giro, como está;
  - bajar el lado izquierdo 3°.

  Tal como apunta, el zoom de 218 corta por abajo la zona cercana y las bandejas D y B (el que cabe es 182).
- **Después de moverla** el usuario (subida ~4 cm, inclinada hacia abajo y `--guardar`), las cuatro indicaciones salen
  "bien":
  - 63 cm sobre la mesa (bien de 61 a 87), a 94 cm del labio frontal;
  - 23.0° hacia abajo (lo que centra, 23.5°) y 4.7° a la izquierda (3.0°);
  - 26° de elevación (en el borde del rango) y zoom ×2.74.

  Sigue el ladeo de 3.5° (bajar el lado izquierdo). Con `--guardar`, `zoom_absolute` 218 → 248: entran la zona, las
  cuatro bandejas y las manos en la observación. Después la giró al centro (2.8° a la izquierda; lo que centra, 2.6°)
  y volvió a guardar: 273, el zoom con el que la zona y las bandejas llenan el 1080p con un 10 % de margen.
  Con 273 las manos en reposo quedaban cortadas por la muñeca (y al levantar quedaría fuera la parte alta de la mano):
  `situar.altura_m` 0.20 → **0.30** y **`zoom_absolute` 240**. En la foto real a 240, los ArUco 6 y 7 caen a 1 px de
  lo previsto y se ven las manos con la muñeca, los objetos y las cuatro bandejas.
  Queda margen para acercarla al sitio ideal (64 cm de alto y a 68 cm del labio, 32°): ~25 % más de píxeles del
  sensor en el cubo.
- Pruebas (`tests/test_situar_camara.py`, con una cámara virtual):
  - la pose con el 6 y el 7 en cualquier giro;
  - el 8 y el 9 medidos y luego solos;
  - en el sitio ideal no hay que mover nada;
  - baja y torcida dice súbela, inclínala y gírala;
  - `--guardar` solo toca el zoom de la cámara externa.

## 64. Sexto cubo: giró dentro de la pinza al levantarlo, sobre la línea pulgar–índice (2026-11-13)

`./manipulacion.sh --nivel 2 --arnes` (`datos/tirada_20261113_081121_nivel2`, grabada con la BRIO a zoom 240). El
cubo (tapa en color, (0.515, −0.040)) se cogió con los tres dedos: al cerrar, medio 401, índice 325 y pulgar 631 gf,
con la mano a (−1.8, 0.6, −3.3) mm del plan. Al levantarlo fue girando dentro de la pinza: ~10° a 1 s del transporte,
~15° a 2.5 s y ~35° a 5 s. Entonces el medio perdió el contacto (40 gf) y el índice se clavó (2073 → 3225 gf). El
corte de fuerza (1200 gf en 3 lecturas) hizo la salida segura y el cubo cayó cerca de su sitio.

- **La mano no giró.** En el vídeo, el ArUco del dorso se mantiene dentro de ±1° todo el levantamiento, y la mano sube
  recta (±5 mm de lado).
- **Giró el cubo, alrededor de la línea pulgar–índice, con el lado del medio hacia abajo** (a la derecha del robot).
  - El pulgar está enfrente del índice (z +0.032 y +0.034 en el ancho de la mano), no de la resultante de índice y
    medio que usaba §58.
  - Con el centro en +0.026, el centro de masas quedaba 7 mm hacia el medio de esa línea. La línea (del pulgar a
    46 mm sobre la mesa al índice a 31) pasaba solo ~8.5 mm por encima de él.
  - Es un péndulo que cuelga a atan(7/8.5) ≈ 40°, lo que se vio antes de clavarse. Solo lo frenaba la fricción del
    medio.
- **Cambios** (`objetos.yaml`, el cubo):
  - `centro_agarre` z 0.026 → **0.030**, 3 mm hacia el medio desde la línea pulgar–índice: el índice y el pulgar
    quedan a 2–4 mm del centro de la cara, y el medio a 1 cm de su borde.
  - `altura_agarre_m` 0.055 → **0.058**: la línea, a ~11.5 mm sobre el centro de masas; el pulgar, 11 mm bajo la tapa.
  - Sin el medio, el cubo colgaría a atan(3/11.5) ≈ 15° (antes, 40°). Con el error de altura de esta tirada (la mano
    3.3 mm más baja), a ~19°.
- El prisma sigue con 0.026 (va con la izquierda; sin probar).
- Pruebas: 128 de 128 en el PC remoto (2 min 10 s con `-n 16`), sobre una copia aparte con este cambio.

## 65. Séptimo cubo: colocado y recto en la mano. La inclinación al volver a reposo (2026-11-13)

Con §64 desplegado. La tirada anterior (`tirada_20261113_082639_nivel2`) se lanzó sin `--medir-mesa` después de un
`[g]` nuevo, y la mano cogió el cubo por su borde de arriba (el usuario). Antes de esta, el usuario hizo
`marcar_montaje.py`, `--medir-mesa --arnes --escala-tiempo 1` y `situar_camara_externa.py --una --guardar`.

`./manipulacion.sh --nivel 2 --arnes` (`datos/tirada_20261113_090609_nivel2`, escala 2): **cubo colocado en la
bandeja D al primer intento.**
- El cubo, con la tapa en color, en (0.512, −0.028) a 2.3°. La mano, a (−0.2, −2.9, −2.3) mm del plan.
- Al cerrar: medio 421, índice 402 y pulgar 647 gf. En el vídeo, el índice está cerca del centro de la cara.
- **El cubo va recto en la mano todo el transporte** (+2 a +30 s en el vídeo): no hay el giro de §64.
- Como en §60, el índice y el medio siguen cerrando (711 → 646 y 730 → 667) y el pulgar cede (394 → 495). Las
  fuerzas suben hasta 858/813/943 gf: ~260 gf por debajo del corte de 1200.

**La inclinación al volver a reposo** (SALIDA, observación → reposo; registro del arnés `arnes_20261113_084853`):
- **0–2 s: −4.5° (hacia atrás), sin que el robot hiciera nada.**
  - Las consignas de brazos y piernas no cambian (< 0.02 rad) y las articulaciones tampoco: el robot entero se
    balanceó sobre los talones.
  - La carga de los pies baja de 0.78 a 0.57 (290/234 → 210/164 N): ~150 N pasaron de los pies a la cuerda, una
    fuerza externa.
  - En toda la sesión del arnés no hay otra inclinación de más de 2° (incluido el `--medir-mesa` a escala 1), ni en
    las salidas de las tiradas anteriores (≤ 2.7°).
- **La integral del estabilizador se dio la vuelta.** Estuvo quieta en +12.15 N·m toda la tirada (dentro de su
  banda; el tope es 15) y, peleando contra esa inclinación, bajó a −8 N·m.
- **11–19 s: +5.5° (hacia delante)**, lo que vio el usuario.
  - Los brazos pasan extendidos por delante y el tirón se suelta (la carga sube a 0.87).
  - La integral está 20 N·m por debajo de lo que el robot necesitaba.
  - Vuelve a −1° a los 23 s, al recargarse la integral (+7.8).
- **Después, la carga queda en 0.67** (antes, 0.78): la cuerda cambió de estado.
- Con +12 N·m de integral toda la sesión, la cuerda tiraba del torso hacia delante (§31; `arnes.py` avisa en el tope:
  retrasar el pórtico). El usuario va a mover el pórtico y el polipasto, y a repetir `marcar_montaje` y
  `--medir-mesa`.

## 66. A escala 1, sin inclinarse. El cilindro, con la receta del cubo (2026-11-13)

El usuario movió el pórtico y el polipasto, y repitió el montaje, `--medir-mesa` y el zoom (247).
`./manipulacion.sh --nivel 2 --arnes --escala-tiempo 1` (`datos/tirada_20261113_092451_nivel2`): **el cubo,
colocado en D al primer intento, en 114 s** (196 a escala 2).
- El cubo, en (0.544, −0.039); la mano, a (−0.7, −3.2, −2.4) mm del plan.
- El torso, entre −0.1 y +1.5° en toda la tirada (la salida, ≤ 1.2°): ya no se inclina como en §65.
- Llevando el cubo, el pulgar llegó a **1069 gf** (el índice, 885): ~130 gf por debajo del corte de 1200. A
  escala 2 eran 943.

**El cilindro** (azul, 5 × 8 cm, 33 g, a la bandeja B con la izquierda). El usuario lo puso en ~(0.53, −0.05)
(situado con la BRIO y su pose).
- **De lado no hay IK con el montaje de hoy:** en el gemelo, 0 de 8 sitios de la zona. Se coge desde arriba.
- **Con lo de antes** (cuatro dedos y pulgar, `abierta_opuesta`, a 5 cm, centro z 0.010), los dos fallos del cubo:
  - la línea pulgar–índice, a ~33 mm de la mesa: por debajo del centro de masas (40 mm), como el cubo de §61;
  - el pulgar, a 22 mm del eje de un cilindro de 25 mm de radio: aprieta casi de canto.
- **Ahora**, como el cubo:
  - `tres_dedos` y `centro_agarre` z 0.030: el pulgar y el índice, a 2–4 mm del eje; el medio, a 19 mm del otro
    lado, empuja hacia la pinza;
  - `altura_agarre_m` 0.068: pulgar ~59, índice ~44 y medio ~37 mm; la línea, a ~51 mm (11.5 sobre el centro de
    masas) y el pulgar, a 21 mm de la tapa;
  - FORCE_SET `[-1, -1, 250, 250, 400, -1]`: 33 g, y el pulgar del cubo ya roza el corte;
  - `agarre: [arriba]`.
- **En el gemelo** (montaje del robot de las 09:20, escala 1), colocado en 5 de 8 sitios. Entre ellos, el suyo
  (0.53, −0.05), (0.50, 0), (0.56, 0.10), (0.42, 0.15) y (0.44, 0.05).
  - Sin plan cerca del cuerpo y a la derecha: (0.42, 0.01), (0.40, −0.05), (0.45, −0.12), (0.48, −0.15) y
    (0.55, −0.17). La mano izquierda pasa a 14–20 mm de la mesa yendo a 'sobre' (la holgura es 20), o la bandeja
    baja de 15 cm.
  - Lo de antes llegaba a 4 de esos 5. Con la altura o el centro de antes se recupera alguno, pero son márgenes
    al límite que cambian con la postura de la IK.
- `test_salida_segura_en_el_transporte_sale_por_la_observacion`: el cilindro, de (0.42, 0.01) a (0.44, 0.01).
- Pruebas: 128 de 128 en el PC remoto.

## 67. El cilindro, "agarre fallido" con el pulgar apretando (2026-11-13)

Con §66 desplegado. Cuatro intentos, todos "agarre fallido": el índice y el medio, "lleno"; el pulgar, "vacío".
- `tirada_20261113_094940_nivel2`: cilindro en (0.537, −0.059), con `--sin-comprobar-montaje` (ver abajo).
- `tirada_20261113_095442_nivel2`: el usuario lo movió a (0.492, 0.052).
- El robot abrió, volvió a la observación y lo reintentó.

- **El pulgar sí apretaba el cilindro.**
  - Bajó de 998 a 322 sin frenarse y se paró por fuerza (459 gf; escrito 447) en el mismo instante que el índice y
    el medio (685 y 687), contra el cilindro. En la cámara externa, los dedos están sobre el cilindro.
  - En los cuatro intentos, el pulgar quedó en 302–331 con 289–459 gf. En vacío, el pulgar izquierdo cierra a 302
    con 47 gf (su residuo de flexión, `config/manos.yaml`).
- **La comprobación miraba solo el ángulo:** "vacío" a menos de 60 de su consigna de cierre (300).
  - Con el cubo de 6 cm, el pulgar se paraba en ~400.
  - El cilindro de 5 cm lo lleva a su consigna: no cierra más, porque más allá choca con los dedos. Sujetan el
    índice y el medio, apretando contra un pulgar quieto en su posición. Para algo más estrecho no queda margen.
- **Cambio** (`Mano.agarre`): con el residuo medido (en el robot), un dedo en su consigna de cierre solo es "vacío"
  si su fuerza, sin la base ni el residuo, no llega a la mitad de su FORCE_SET. Sin él (la mano simulada de las
  pruebas, 70–230 gf de flexión), solo el ángulo, como antes.
  - En los cuatro intentos, el pulgar marca 242–412 gf netos (≥ 200): "lleno".
  - `escapado` sigue pidiendo lo mismo.
  - Prueba nueva con los datos de la tirada.
- **"Montaje movido"** en las cinco tiradas anteriores.
  - La mesa se veía 1.3 cm a la derecha del robot y girada 0.5°, y las bandejas B y D, a 1.1–1.2 cm. Todo junto: el
    usuario no tocó la mesa, así que se movió el robot respecto de ella.
  - Con `--sin-comprobar-montaje`, la bandeja se apunta con ~1.2 cm de error (el plan admite 1.5).
  - Lo correcto es repetir `marcar_montaje` (`[g]`) y `--medir-mesa`.
- Pruebas: 129 de 129 en el PC remoto.

## 68. "0 objetos" con el cilindro a la vista: la profundidad de la D435i, parada (2026-11-13)

El usuario hizo `[g]`, `--medir-mesa` y el zoom, y lanzó dos tiradas: `tirada_20261113_235058_nivel2` y
`235217` (con `--sin-comprobar-montaje`). Las dos, **"OBSERVAR: 0 objetos"**. Sin embargo:
- el cilindro se ve en la imagen de color de la observación, en (0.50, 0.02) según la BRIO;
- la percepción, con el montaje nuevo y la máscara de los brazos de la observación, lo ve con 0.78–0.91 en la
  captura del `[g]` (23:45) y en una grabada después.

- **La corrección de giro, con 0 medidas** en las dos tiradas (310 a las 09:06). El hilo mide cuando cambia el
  número del fotograma `n`, y no cambió en toda la tirada.
- **El módulo de profundidad estaba parado.** Leyendo el servidor en vivo (23:58): el color cambia entre fotogramas
  (ruido, diferencia media 1.6), pero el IR y la profundidad son idénticos (diferencia 0). Todos llevan
  `n` = 3389 y el mismo `t_cam`, y el servidor los publica a 30 Hz con `t_rx` nuevo.
  - `wait_for_frames` devuelve el color nuevo con el IR y la profundidad viejos.
  - La percepción veía la profundidad de antes, sin el cilindro en la zona, y el color de ahora.
- **Nadie lo notaba:** el lector continuo, el vigilante (`fotograma_max_s`) y `Supervisor._fotogramas` miraban
  `t_rx`, que seguía llegando.
- **Cambios:**
  - `camara_servidor.py`: publica solo los fotogramas con un IR nuevo. Si el IR no cambia en 2 s, reabre la
    cámara; a los tres fallos seguidos, `hardware_reset`. Los fallos vuelven a cero al llegar 30 fotogramas
    nuevos (antes, al abrir: con el IR parado nunca habría llegado al reset).
  - `LectorContinuo`: la edad cuenta desde el último `n` nuevo. Con la profundidad parada, el vigilante para la
    tirada y observar dice "cámara sin fotogramas".
  - `Supervisor._fotogramas`: los fotogramas distintos, por `n`.
  - Prueba nueva: el mismo `n` con `t_rx` nuevo no es un fotograma nuevo.
- Para que valga, hay que reiniciar `camara_servidor.sh` después de desplegar: es un proceso aparte, con sudo.
- Pruebas: 130 de 130 en el PC remoto.

## 69. El cilindro gira en la pinza de tres dedos: el medio lo empuja de lado. Dos dedos (2026-11-14)

Con el servidor de la cámara reiniciado (§68), `./manipulacion.sh --nivel 2 --arnes --escala-tiempo 1`
(`datos/tirada_20261114_000944_nivel2`). La cámara, bien: el giro midió 167 veces.
- El cilindro, visto con 0.83 en (0.498, 0.020).
- "Agarre comprobado" (§67): medio 266, índice 290 y pulgar 401 gf; el pulgar, en 347. La mano, a (0.6, −1.3, 0) mm
  del plan.
- En el vídeo, **el cilindro va girando en la mano mientras sube recta**: ~15° a los 5.5 s del cierre, ~25° a los
  7 s y ~50° a los 11 s. Lo sujetan solo las yemas por arriba.
- A los 16 s, "cilindro se escapa de la mano" (el pulgar, en su tope de 302 con 163 gf; el medio, con 9). Abrió
  sobre la bandeja B: el cilindro cayó en el borde y acabó tumbado dentro de B, pero cuenta como fallido.

- **Por qué gira y el cubo no.** En el cilindro, el medio aprieta a ~19 mm del eje, a ~50° de la pinza. Su fuerza,
  normal a la superficie, tiene una parte de lado (~0.8 de la suya): empuja la parte baja del cilindro hacia el
  índice, con el pulgar sujetándolo arriba.
  - Ese par (~5 gf·m) es mucho mayor que lo que lo devuelve la gravedad (33 g a ~1 cm, ~0.3 gf·m).
  - En el vídeo gira hacia ese lado: la base, hacia el índice.
  - En el cubo, el medio aprieta una cara plana, de frente: sin empuje de lado.
- **Ahora, el cilindro, con dos dedos** (preforma nueva `dos_dedos`: meñique, anular y medio recogidos):
  - el pulgar y el índice, enfrente, con el eje entre los dos (`centro_agarre` z 0.033). En una superficie redonda,
    la pinza lo centra en su línea;
  - a 7.2 cm (pulgar ~63 mm, 17 mm bajo la tapa; índice ~48): la pinza, ~15 mm sobre el centro de masas;
  - FORCE_SET `[-1, -1, -1, 300, 450, -1]`.
  - En banco, el cubo con dos dedos era inestable (§34). Con el cilindro, la pinza lo lleva a su línea y el medio
    ya no lo empuja.
- **El gemelo daba fallido el agarre a 7.2 cm.** Daba la mano por cerrada sobre el objeto con el punto de agarre a
  menos de 3 cm de su centro, y el del cilindro quedaba a 3.2: cerraba en el aire. Ahora, a menos de 3 cm de su eje
  y dentro de su altura.
- En el gemelo, con el montaje del robot y escala 1, colocado en 7 de 8 sitios, entre ellos los dos del usuario
  ((0.50, 0.02) y (0.53, −0.05)). Sin plan en (0.45, −0.12).
- Pruebas: 130 de 130 en el PC remoto.

## 70. Cilindro colocado con dos dedos; el medio recogido lo empujaba al soltar (2026-11-14)

Con §69 desplegado, `./manipulacion.sh --nivel 2 --arnes --escala-tiempo 1 --sin-comprobar-montaje`
(`datos/tirada_20261114_002656_nivel2`): **cilindro colocado en la bandeja B al primer intento**, en 106 s.
- El cilindro, visto con 0.83 en (0.531, 0.038). La mano, a (−0.2, −1.2, −0.8) mm del plan.
- Al cerrar: índice 510 y pulgar 483 gf, los dos "lleno"; el pulgar, en 303 (su tope).
- En el vídeo va recto en la mano todo el transporte: la pinza de dos dedos lo centra (§69).
- **Al soltar, el medio lo empujó.** La mano abría los cinco dedos a la vez. El medio, recogido junto a la parte de
  arriba del cilindro (a ~22 mm de su eje, radio 25), barría hacia abajo al estirarse y lo apretaba contra la pared
  de la bandeja (lo mismo, en menos, el anular y el meñique).
  - **Ahora**, al soltar se abren solo los dedos que sujetan (los de FORCE_SET > 0: pulgar e índice). Los recogidos
    por la preforma se abren después de `retirar_de_bandeja`, con la mano 8 cm más arriba.
  - `Mano.abrir(dedos=...)` abre solo esos DOF. En `abierta_opuesta`, que no recoge ninguno, todo igual que antes.
  - Prueba nueva en el gemelo: la primera apertura es `[-1, -1, -1, 1000, 1000, -1]` y la completa, después.
- **"Montaje movido": la bandeja B, de verdad.** En las tiradas anteriores, solo B estaba 1.1–1.7 cm más al fondo.
  Probablemente la movió el cilindro, que en §69 cayó sobre su borde. El usuario lanzó con `--sin-comprobar-montaje`:
  B está donde la midió `[g]` + 1.5 cm, dentro de su hueco. Hay que volver a ponerla en su sitio o medirla otra vez.
- Pruebas: 131 de 131 en el PC remoto.

## 71. Cilindro: el pulgar, en su tope, no apretaba. Su centro, 5 mm hacia el pulgar (2026-11-14)

El usuario guardó las bandejas B y D (`marcar_montaje.py --fase bandejas`; la mesa y el toque, sin tocar) y lanzó
`./manipulacion.sh --nivel 2 --arnes --escala-tiempo 1` (`datos/tirada_20261114_004925_nivel2`).
- El cilindro, visto con 0.77 en (0.536, 0.063). "Agarre comprobado": índice 512 y pulgar 466 gf.
- A los 5 s del transporte, "cilindro se escapa de la mano" (al levantar): abrió y volvió.

- **El pulgar, en su tope desde el cierre:** 303–306 durante todo el levantamiento (su consigna de cierre es 300).
  No apretaba con su FORCE_SET: hacía de tope fijo mientras el índice empujaba (400–530 gf).
  - En el vídeo, el cilindro va girando y bajando por el índice hasta soltarse a los ~8 s del cierre. Mientras, la
    fuerza del pulgar cae de 467 a 169 gf.
  - En la tirada buena (§70) el pulgar también estaba en 303: fue suerte.
- **Por qué:** en el marco de la muñeca (gemelo, para la dirección), el pulgar llega por −y (abierto, en ~−107 mm) y
  el índice por +y (abierto, en ~−12).
  - El cubo, con el centro en y −0.070 y 30 mm de medio lado, lo toca el pulgar en −100 mm: ~400.
  - El cilindro, con 25 de radio en el mismo centro, en −95 mm: 5 mm más de recorrido, y el pulgar llega a su tope.
- **Ahora**, el centro del cilindro, en y **−0.075** (arriba; lado, −0.070): el pulgar lo toca donde al cubo y el
  índice cierra ~5 mm más (tiene recorrido: 711 de 1000).
- En el gemelo, con el montaje del robot (bandejas de las 00:4x) y escala 1, colocado en los 8 sitios probados,
  entre ellos (0.536, 0.063), (0.50, 0.02) y (0.44, 0.01).
- Pruebas: 131 de 131 en el PC remoto.

**¿Simular el agarre con física (MuJoCo: dedos con fuerza, objetos libres, fricción)?** Ayudaría a ver cosas como
el empuje del medio (§69) o un dedo en su tope, pero no rápido ni con fiabilidad:
- El gemelo pone los dedos a ~1 cm de donde están: con el cubo da el pulgar en 300 y el índice en 890 al tocar,
  y en el robot son ~400 y ~730.
- Los fallos de §64–§71 dependen de milímetros y de la fricción de las yemas blandas, que habría que ajustar con
  el robot.
- Haría falta además el control de fuerza del firmware en los dedos y los objetos con su masa.

Estimado: más de un día y, sin calibrar, resultados dudosos. Por ahora mandan el robot, `manos.csv` y la cámara
externa.

## 72. Cilindro colocado; la cámara, ocupada por otro programa (2026-11-14)

- **La cámara:** a las 00:57 otro programa del robot (`~/h1_sensor_view/bridge.py`, lanzado por `run_view.sh`) abrió
  la D435i con pyrealsense2. La manipulación se quedó sin fotogramas ("No llegan fotogramas") y el servidor, con "No
  device connected". Una RealSense solo la abre un proceso.
  - Parado ese programa, el servidor seguía fallando ("Couldn't resolve requests"). Al tercer fallo hizo
    `hardware_reset` (§68) y arrancó: los tres flujos, con `n` nuevo.
- `./manipulacion.sh --nivel 2 --arnes --escala-tiempo 1` (`datos/tirada_20261114_011303_nivel2`): **cilindro
  colocado en B al primer intento**, en 111 s. La mano se abrió entera después de soltarlo y retirarse (§70).
- **El pulgar, otra vez cerca de su tope:** 303–317 (con 452–511 gf) durante el transporte; el índice, 711–719.
  - Mover el centro 5 mm (§71) no lo cambió: 306 al cerrar, frente a 303 en §71. La explicación de §71 (el pulgar
    llega por −y) no se confirma.
  - El gemelo pone los dedos a ~1 cm de su sitio (§71) y no sirve para esto.
  - El cilindro de 5 cm se coge con el pulgar casi en su consigna de cierre. Esta vez apretó (se paró por fuerza, en
    317) y lo sostuvo, pero el margen es poco.

