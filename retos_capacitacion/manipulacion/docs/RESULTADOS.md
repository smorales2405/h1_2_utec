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
