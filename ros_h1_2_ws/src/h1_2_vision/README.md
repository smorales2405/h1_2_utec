# `h1_2_vision`

La cámara de cabeza del H1-2 —una **Intel RealSense D435i**— publicada en ROS 2
como una cámara normal.

## Por qué no `realsense2_camera`

La D435i **está cableada al PC1**, el computador de locomoción, y a PC1 no
tenemos credenciales. En el PC2 no hay ningún dispositivo de vídeo en el bus
USB: `lsusb` no ve ningún Intel, `/dev/video*` está vacío, `uvcvideo` ni
siquiera está cargado y `rs-enumerate-devices` responde *No device detected*.
Ningún driver de RealSense puede abrir una cámara que no está en la máquina.

Lo que sí existe es el servicio **`videohub`** del propio robot, que publica en
el dominio DDS 0 y responde a cualquiera **sin credenciales**. Este paquete lo
envuelve. El análisis completo —las cinco comprobaciones y qué haría falta para
mover la cámara al PC2— está en
[`h1_2_teleoperation/README_DEPLOY.md` §7](../../../h1_2_teleoperation/README_DEPLOY.md).

Lo que eso implica, y no se puede evitar desde aquí:

| | |
|---|---|
| Resolución | 1920×1080 |
| Fotogramas nuevos | **~15 Hz**, no 30 |
| Formato | JPEG ya comprimido |
| **Profundidad** | **no hay** — el `videohub` solo expone el color |
| Estereoscopía | no: para el operador es monocular |

Los 15 Hz son el techo real del servicio, no de la cámara: devuelve el último
fotograma en caché, así que pedir más rápido solo trae repeticiones. El nodo las
descarta.

### ¿Y la profundidad? La D435i la tiene; nosotros no

La pregunta es obligatoria: una D435i **es** una cámara de profundidad. El
sensor la tiene. Lo que no hay es forma de sacarla del robot, y esto está
comprobado sobre el bus el 2026-09-15, no supuesto:

| Comprobación | Resultado |
|---|---|
| Qué implementa el servicio `videohub` | **solo `api_id 1`** (versión, `1.0.0.0`) **y `1001`** (`GetImageSample` → JPEG de color). Del 1002 al 1012, **silencio**: ni siquiera devuelve `3203 = API_NOT_IMPL` |
| Tópicos de imagen de profundidad en el dominio 0 | ninguno |
| Nubes de LIO-SAM (`/lio_sam_ros2/…`) | **0 publicadores**: el stack de SLAM está parado |
| `/point_in_map` | 1 publicador, pero **no emite** |
| `/pctoimage_local` | 2 publicadores, pero `unitree_interfaces` no está instalado en ningún sitio: ni se puede decodificar. Y es una proyección del **LiDAR**, no de la cámara |
| `/frontvideostream` | emite a 22,9 Hz, pero **no deserializa** como `unitree_go/Go2FrontVideoData` («invalid data size») y lleva 360 bytes: no es vídeo |

Así que la profundidad existe en el sensor y se queda en PC1. Lo único que PC1
saca por DDS es un JPEG del flujo de color.

### PC2 no es especial: nadie «lee la cámara desde PC2»

Aquí hay un malentendido que conviene deshacer, porque cambia toda la respuesta.
El RGB **no se lee desde PC2**. Lo sirve **PC1** por el servicio `videohub`, y lo
lee cualquiera que esté en el bus DDS: esta laptop, PC2, el que sea. Cuando el
puente `videohub_image_bridge.py` corre en PC2, hace exactamente lo mismo que
hacemos aquí — pedirle el JPEG a PC1. **PC2 no tiene la cámara**: es un
espectador igual que la laptop.

La cámara está físicamente en el **USB de PC1**. Y la profundidad de una D435i
se calcula **a bordo, en la cámara**, a partir de su par de infrarrojos, y sale
por el USB. Así que para tener profundidad hay que hablarle a la cámara **por
USB** — no hay forma de sacarla por red si el equipo del otro lado no expone ese
flujo. PC1 tiene el USB pero solo publica el color en JPEG.

Comprobado sobre la red real el 2026-09-15 (dominio DDS 0, subred
`192.168.123.0/24`):

| Dónde busqué la profundidad | Resultado |
|---|---|
| Servicios DDS del robot (`/api/*`) | siete: `bashrunner`, `config`, `loco`, `motion_switcher`, `robot_state`, `sport`, `videohub`. **Ninguno de profundidad** |
| `videohub`, api 1002-1012 | silencio; solo implementa 1001 (color) |
| Otros dominios DDS (1-5) | vacíos |
| PC1 (`.161`) HTTP :80 nginx | error 500 en toda ruta probada |
| PC1 (`.161`) HTTP :8081 aiohttp | 404 en `rgb`, `depth`, `camera`, `image`, `stream`… |
| PC2 (`.164`) en el bus USB (§7) | la RealSense **no está**: `lsusb` sin Intel, `/dev/video*` vacío, `rs-enumerate` «No device detected» |

Es decir: **la profundidad existe en el sensor y se queda en PC1**, que solo
saca un JPEG de color. Ni PC2 ni la laptop pueden leerla, porque ninguno tiene
la cámara conectada.

**Cómo tenerla de verdad**, en orden de esfuerzo, las tres con intervención
física:

1. **Mover el cable USB de la D435i de PC1 a PC2** (o a la laptop). Es la única
   que da profundidad **real**, y por eso es la de referencia. `README_DEPLOY.md`
   §7: el hub Terminus de 7 puertos solo usa 2, PC2 ya tiene `librealsense`
   compilada, las reglas udev puestas y `pyrealsense2` instalado, y
   `wait_for_camera.py` la detecta en cuanto enumere. Da **30 fps y profundidad
   alineada al color**, a cambio de quitársela a PC1. Si esto NO es una opción
   —que es lo normal si PC1 la necesita para locomoción—, entonces no hay
   profundidad de la cámara por ningún otro camino: quedan el LiDAR (punto 3) o
   la estimación monocular.
2. **Enchufarla a esta laptop.** `realsense2_camera` y `rs-enumerate-devices`
   ya están instalados aquí (`ros2 pkg list | grep realsense` los lista), así
   que sería `ros2 launch realsense2_camera rs_launch.py enable_depth:=true
   align_depth.enable:=true` y ya. Mismo coste físico.
3. **Usar el LiDAR del robot**, que sí es un sensor de distancia y está en el
   URDF (`lidar_link`). Habría que arrancar el servicio de mapeo del robot para
   que las nubes empiecen a publicar. No es la cámara: es disperso, no viene
   alineado con la imagen, y hay que registrarlo contra `camera_link`.

Mientras tanto, la distancia se estima por tamaño aparente — ver
[la prueba del objeto azul](#pruebas-pruebasdetector_azulpy).

## Estado

Probado **sobre el robot real** el 2026-09-15, con la cámara mirando al
laboratorio:

| | |
|---|---|
| `camera_test` (modo `service`) | ✔ 180/180 respuestas, 0 perdidas, 0 errores |
| `camera_test` (modo `topic`) | ✔ 15,2 Hz, 55 KB por fotograma |
| `head_camera` sostenido | ✔ **6598/6599 respuestas en ~7 min** a 15,0 Hz clavados |
| `image_raw` en crudo (6,2 MB por mensaje) | ✔ 15,0 Hz en un suscriptor, imagen intacta |
| `image_raw/compressed` | ✔ `ros2 topic hz` da 15,000 Hz, desviación 0,7 ms |
| Reescalado a 1280×720 q=70 | ✔ 37 KB por fotograma, 14,7 Hz |
| `camera_info` | ✔ `fx=1386,4` `fy=1388,6`, centro en el centro |
| TF `camera_link` → marco óptico | ✔ (−90°, 0, −90°) |
| Dos clientes a la vez | ✔ el nodo y un `camera_test` sondeando el mismo servicio, 0 errores en los dos |

Medido contra lo que decía `README_DEPLOY.md` §7:

| | documentado | medido ahora |
|---|---|---|
| Latencia por petición | media 6,4 ms, máx 7,3 | media **5,4 ms**, máx 6,4 |
| Fotogramas nuevos | ~15 Hz | **15,0 Hz** |
| Tamaño por fotograma | 77-82 KB | 54-55 KB (la escena de hoy es más lisa) |

Lo que **no** se ha vuelto a ver: el objeto oscuro que tapaba un tercio del
encuadre en las capturas de `camera_test/`. Hoy la vista está despejada.

## Arrancar

```bash
cd ros_h1_2_ws
colcon build --symlink-install
source setup_env.sh
```

`setup_env.sh` hace falta: fija `RMW_IMPLEMENTATION` y apunta `CYCLONEDDS_URI`
a la NIC del robot. Sin eso el nodo se crea, no oye nada, y el error no dice
por qué.

## Lo primero, siempre

```bash
ros2 run h1_2_vision camera_test
```

Le pregunta al robot directamente, sin nodo de por medio, durante 5 s, y dice
si hay cámara, a qué resolución, a qué tasa real y con cuánta latencia. Si esto
falla, el problema está en el robot o en la red, y no hay nada que depurar en
el nodo. Salida real del robot:

```
  preguntando al servicio `videohub` a 30 Hz durante 5 s...
      150/150 respuestas · latencia media 5.3 ms (máx 6.5) · perdidas 0 · errores 0

  ✔ el servicio `videohub` del robot
      resolución        1920x1080
      tamaño            54 KB por fotograma
      recibidos         30.0 Hz
      fotogramas NUEVOS 15.2 Hz   ← la tasa real
      primer fotograma  6 ms tras arrancar
      guardado en       /tmp/h1_2_head_camera.jpg
```

Guarda el último fotograma para poder mirarlo. Para no guardarlo,
`-p save:="''"`: las comillas dobles hacen falta, porque `-p save:=''` a secas
no lo parsea `rcl` («Couldn't parse parameter override rule»).

## El nodo

```bash
ros2 run h1_2_vision head_camera
```

| tópico | tipo | cuándo |
|---|---|---|
| `/head_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` (JPEG) | siempre |
| `/head_camera/image_raw` | `sensor_msgs/Image` (bgr8) | solo con `raw:=true` |
| `/head_camera/camera_info` | `sensor_msgs/CameraInfo` | siempre |

El tópico comprimido es el camino natural: el `videohub` ya entrega JPEG, así
que se republica **tal cual**, sin decodificar ni recomprimir. La imagen en
crudo cuesta una decodificación por fotograma y 93 MB/s en el bus a tamaño
nativo, y por eso va apagada por defecto.

Para mirarla:

```bash
ros2 run rqt_image_view rqt_image_view /head_camera/image_raw/compressed
ros2 run h1_2_vision camera_test --ros-args -p mode:=topic    # medir, sin ventana
```

⚠ `ros2 topic hz /head_camera/image_raw` **no imprime nada**, y no es que no se
publique: es cosa suya con mensajes de 6,2 MB. `ros2 topic echo` sobre el mismo
tópico responde, y un suscriptor normal recibe los 15 Hz enteros. Para medir,
`camera_test -p mode:=topic`. Sobre el comprimido, `ros2 topic hz` sí funciona.

### Parámetros

| parámetro | por defecto | qué hace |
|---|---|---|
| `poll_hz` | `30.0` | sondeos por segundo al `videohub`; el doble de los 15 Hz reales |
| `repeats` | `false` | publicar también las repeticiones del caché |
| `raw` | `false` | publicar además `sensor_msgs/Image` en bgr8 |
| `camera_info` | `true` | publicar `CameraInfo` |
| `width` / `height` | `0` | reescalar; `0` deja el tamaño nativo |
| `quality` | `80` | calidad JPEG al recomprimir (solo si se reescala) |
| `frame_id` | `camera_color_optical_frame` | marco de las imágenes |
| `optical_tf` | `true` | TF estática `camera_link` → marco óptico |
| `topic_ns` | `/head_camera` | prefijo de los tópicos |
| `best_effort` | `false` | QoS `sensor_data` en vez de la fiable |
| `timeout` | `2.0` | segundos antes de dar una petición por perdida |
| `report_every` | `5.0` | segundos entre informes de tasa; `0` los silencia |

```bash
ros2 run h1_2_vision head_camera --ros-args -p raw:=true -p width:=1280 -p height:=720
```

`best_effort:=true` para enlaces con pérdidas (WiFi). **Tiene que coincidir en
los dos extremos**: con QoS incompatibles el suscriptor no se empareja y parece
que no se publica nada.

### `camera_info` es NOMINAL, no una calibración

Los intrínsecos salen del campo de visión de catálogo del módulo RGB de la
D435i (69,4° × 42,5°), escalado a la resolución que se publique: a 1920×1080
dan `fx≈1386`, `fy≈1389`, centro en el centro. Sirven para dimensionar o
apuntar; **no para medir**. La distorsión va a cero, que es mentira pero es
mejor que inventarse coeficientes. Si algún día hace falta precisión, hay que
calibrar con `camera_calibration` y sustituirlos.

### El marco óptico

El URDF del robot solo trae `camera_link`, con la convención de los cuerpos
(x hacia adelante). Las imágenes usan la óptica (z hacia adelante, x a la
derecha, y hacia abajo), así que el nodo publica una TF estática
`camera_link → camera_color_optical_frame` con la rotación de siempre
(−90°, 0, −90°) y etiqueta las imágenes con ese marco. Sin eso, todo lo que se
proyecte con los intrínsecos sale girado 90°.

Para verlo en RViz hace falta que alguien publique el árbol del robot:

```bash
ros2 launch h1_2_inspire_description display_h1_2_with_hands.launch.py
```

## Pruebas: `pruebas/detector_azul.py`

Encontrar un objeto circular azul y decir a qué distancia está. Es la prueba de
que la cámara sirve para algo más que mirarla.

```bash
ros2 run h1_2_vision head_camera                                    # terminal 1
ros2 run h1_2_vision detector_azul --ros-args -p diametro:=0.067    # terminal 2
rviz2 -d $(ros2 pkg prefix h1_2_vision)/share/h1_2_vision/rviz/objeto_azul.rviz
```

```
  ● azul a  0.75 m  ( 74.8 cm)   radio  60.3 px · redondez 0.98 · centro (1164, 544) · xyz (+0.11, +0.00, 0.75) m
  · sin objeto azul en el encuadre
```

Publica, además de imprimirlo:

| tópico | qué es |
|---|---|
| `/objeto_azul/imagen/compressed` | la imagen con el círculo y la distancia pintados |
| `/objeto_azul/punto` | `geometry_msgs/PointStamped` en el marco óptico |
| `/objeto_azul/marcador` | `visualization_msgs/Marker`: una esfera del tamaño real a la distancia estimada |

### La distancia se ESTIMA, no se mide

Por el `videohub` no llega profundidad, así que lo único que queda es el modelo
pinhole:

    Z = fx · diámetro_real / diámetro_en_píxeles

Tres cosas que hay que tener presentes antes de fiarse de un número:

1. **Hay que decirle el diámetro real** con `-p diametro:=` (en metros). El
   resultado escala **linealmente** con ese dato: 10% de error en el diámetro,
   10% de error en la distancia. Por defecto son 6,5 cm.
2. Los intrínsecos son los nominales de `head_camera`, **sin calibrar**.
3. El radio de una mancha comprimida en JPEG baila un píxel de un fotograma al
   siguiente, y a 3 m un píxel ya son varios centímetros.

Léelo como «está a un metro y pico», no como una medida.

### El radio sale del área, no del círculo que encierra

`minEnclosingCircle` encierra los centros de los píxeles del borde, así que
siempre sobra casi un píxel. Medido sobre círculos sintéticos con ruido y
compresión JPEG:

| radio real | sesgo de `minEnclosingCircle` | sesgo de `√(área/π)` |
|---|---|---|
| 100 px | +1,0% | +0,3% |
| 50 px | +2,0% | +0,6% |
| 25 px | +3,4% | +1,2% |
| 15 px | +5,4% | +2,3% |
| 10 px | +8,2% | +2,8% |

Por eso la distancia usa el radio del área. El círculo que encierra se queda
para pintar y para medir la redondez, que es lo suyo.

### Parámetros

| parámetro | por defecto | qué hace |
|---|---|---|
| `diametro` | `0.065` | diámetro REAL del objeto, en metros |
| `h_min` / `h_max` | `95` / `130` | tono azul en HSV de OpenCV (0-179) |
| `s_min` | `110` | por debajo es gris azulado, como el suelo del laboratorio |
| `v_min` | `60` | por debajo es negro, y el tono ya no significa nada |
| `radio_min_px` | `12` | por debajo de esto es ruido |
| `circularidad_min` | `0.70` | área real / área del círculo que lo encierra |
| `anotar` / `marcador` | `true` | publicar la imagen pintada / la esfera |
| `ventana` | `false` | ventana de OpenCV, si hay pantalla |
| `report_every` | `1.0` | segundos entre líneas por consola |
| `distancia_real` | `0.0` | la distancia medida con cinta, para contrastar; `0` lo desactiva |

### Contrastarlo con una cinta métrica

Sin profundidad, la única forma de saber cuánto te puedes fiar del número es
medirlo a mano una vez:

```bash
ros2 run h1_2_vision detector_azul --ros-args -p diametro:=0.067 -p distancia_real:=1.20
```

Cada línea añade el error, y al salir con Ctrl-C da el factor de corrección,
que absorbe de una vez el error del diámetro supuesto y el de los intrínsecos
sin calibrar:

```
  136/136 fotogramas con objeto (100%)
  distancia: media 0.75 m · mediana 0.75 m · min 0.75 · máx 0.75 · desviación 0.1 cm
  contra la cinta: real 0.75 m, medido 0.75 m, error -0.5%
  para corregirlo: -p diametro:=0.0653  (el factor 1.005 absorbe el diámetro y los intrínsecos)
```

Un cuadrado azul tiene redondez 0,64 y el umbral es 0,70, así que se descarta
solo. Si tu objeto no aparece, baja `s_min` o `circularidad_min` y mira qué
pasa con `-p ventana:=true`.

### Qué se ha comprobado

Círculos sintéticos de radio conocido, pasados por JPEG como los del robot,
contra la distancia que exige la fórmula:

| radio | Z esperada | Z medida | error |
|---|---|---|---|
| 100 px | 0,451 m | 0,449 m | 0,3% |
| 50 px | 0,901 m | 0,895 m | 0,6% |
| 25 px | 1,802 m | 1,779 m | 1,3% |
| 15 px | 3,004 m | 2,942 m | 2,1% |

Y sobre el robot: un cuadrado azul se rechaza, la escena real del laboratorio
no da falsos positivos, y con fotogramas inyectados en los tópicos de la cámara
(un disco de 60 px, que exige 0,751 m) el detector dio **0,75 m** —error −0,5%,
136 de 136 fotogramas, desviación 1 mm— y RViz pintó la esfera en su sitio.

Con un objeto azul real delante, **el detector lo encuentra** (confirmado sobre
el robot el 2026-09-15). Lo que **falta** es contrastar la distancia contra una
medida con cinta: ahí es donde entra `-p distancia_real:=`.

### RViz

`rviz/objeto_azul.rviz` trae la cámara, la imagen anotada, la TF y el marcador.
El marco fijo es el óptico de la cámara, que publica el propio `head_camera`,
así que **funciona sin `robot_state_publisher`**. Si además lanzas el URDF
(`ros2 launch h1_2_inspire_description display_h1_2_with_hands.launch.py`),
cambiando «Fixed Frame» a `pelvis` se ve la esfera colocada respecto del robot.

Los paneles de imagen arrancan estrechos: se agrandan arrastrando el borde, y
`File > Save Config` lo deja guardado.

## Sin el robot delante

```bash
ros2 run h1_2_vision fake_videohub          # en otra terminal
ros2 run h1_2_vision camera_test
```

`fake_videohub` imita el servicio del robot: mismos tópicos, mismo `api_id`,
imagen JPEG en el campo `binary` y el mismo caché de 15 Hz que hace que las
peticiones rápidas devuelvan repeticiones. Sirve para probar el nodo entero
—y para saber si un fallo es nuestro o suyo— con el robot apagado.

⚠ **Nunca con el robot conectado**: habría dos servicios contestando en el
mismo dominio DDS.

## Dos cosas que cuestan una tarde si no se saben

**Los campos `uint8[]` se rellenan con `array.array('B')`, nunca con `bytes`.**
Asignar `bytes` hace que rclpy valide el rango elemento a elemento en Python:
medido aquí, **852 ms** para una imagen de 6,2 MB. Con eso la cámara caía de
15 Hz a 1,2 Hz, y el síntoma no apunta a ninguna parte —el nodo parece
funcionar, solo que lentísimo—. Con `array.array` entra por el camino rápido y
no cuesta nada medible.

**`cv_bridge` no se usa, y no es un descuido.** El que trae Humble está
compilado contra numpy 1.x y en esta máquina hay numpy 2.x: importarlo revienta
con `AttributeError: _ARRAY_API not found`. Rellenar un `sensor_msgs/Image` a
mano son cinco líneas.

## Convivencia con la teleoperación

El `videohub` contesta a todo el que pregunte, así que este nodo y el puente
`videohub_image_bridge.py` del PC2 pueden correr a la vez: cada uno empareja
sus respuestas por el `identity.id` que mandó. Lo que se duplica es el tráfico
—dos flujos de ~1,2 MB/s—, así que conviene no dejarlos corriendo a la vez sin
motivo.

## Estructura

| archivo | qué es |
|---|---|
| `videohub_client.py` | el cliente del servicio `videohub`, en ROS 2 puro (sin `unitree_sdk2py`) |
| `head_camera.py` | el nodo: `videohub` → `CompressedImage` / `Image` / `CameraInfo` |
| `camera_test.py` | la prueba rápida, en modo `service` o `topic` |
| `fake_videohub.py` | el servicio simulado, para trabajar sin el robot |
| `pruebas/detector_azul.py` | prueba de visión: objeto azul y a qué distancia está |
| `rviz/objeto_azul.rviz` | la vista de RViz de esa prueba |
