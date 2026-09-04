# Arquitectura: cómo se le habla a los motores del H1-2

## Los tres canales

| Tópico DDS | Tipo | Quién publica | Para qué |
|---|---|---|---|
| `rt/lowstate` → ROS 2 `/lowstate` | `unitree_hg/LowState` | el robot, 500 Hz | estado de los 35 huecos de motor, IMU, modo |
| `rt/lowcmd` → `/lowcmd` | `unitree_hg/LowCmd` | **el servicio del robot**, 500 Hz | control de los 27 motores. Ocupado salvo en modo debug |
| `rt/arm_sdk` → `/arm_sdk` | `unitree_hg/LowCmd` | nadie | control de brazos + cintura **con el robot en marcha** |

`unitree_ros2` es un puente de nombres: el prefijo `rt/` del DDS crudo aparece
en ROS 2 sin él. Es el **mismo** tráfico DDS que ve `unitree_sdk2py`; los dos
caminos son intercambiables y las ganancias medidas por uno valen para el otro.

## El canal `arm_sdk`

Referencia: `unitree_sdk2/example/h1/high_level/h1_2_arm_sdk_dds_example.cpp`.

El servicio de control del robot escucha `rt/arm_sdk` y **mezcla** lo que
recibe con su propio comando, en vez de sustituirlo:

```
comando_final[j] = (1 − w) · comando_interno[j] + w · comando_arm_sdk[j]
```

para los 15 motores que cede, y `comando_interno[j]` sin más para el resto.
El peso `w` viaja en un sitio poco intuitivo: **`motor_cmd[27].q`**. El motor
27 no existe en el H1-2 (`kNotUsedJoint`), y Unitree reaprovecha su campo `q`.

Consecuencias:

* Las piernas **nunca** se tocan por este canal. El robot sigue equilibrándose.
* Con `w = 0` se puede publicar sin efecto ninguno. Es la prueba prudente.
* Al terminar hay que bajar `w` a 0 poco a poco; si se corta de golpe, el
  brazo pasa de nuestra consigna a la del robot en un ciclo.

Los 15 motores que cede, en el orden del ejemplo oficial:

```
13 L_shoulder_pitch   14 L_shoulder_roll   15 L_shoulder_yaw   16 L_elbow
17 L_wrist_roll       18 L_wrist_pitch     19 L_wrist_yaw
20 R_shoulder_pitch   21 R_shoulder_roll   22 R_shoulder_yaw   23 R_elbow
24 R_wrist_roll       25 R_wrist_pitch     26 R_wrist_yaw
12 waist_yaw
```

Hay que comandar **los 15**, no solo el que se esté probando: los que se dejen
sin comandar quedan con `kp = kd = 0` y, con `w = 1`, se descuelgan. Por eso
`H12Client` clava los otros catorce en la postura que tenían al ceder el
control.

## El PD vive en el motor, no aquí

Cada mensaje lleva, por articulación, cinco números: `q`, `dq`, `tau`, `kp`,
`kd`. La electrónica de la articulación calcula, a su propia frecuencia:

```
tau_motor = kp · (q_des − q) + kd · (dq_des − dq) + tau_ff
```

Es decir: **no escribimos un controlador, elegimos sus ganancias**. De ahí que
«sintonizar» aquí sea buscar kp y kd por articulación, y que tres cosas se
sigan de la fórmula:

1. **No hay término integral.** El error permanente contra la gravedad es
   `tau_gravedad / kp` y no se va nunca. Con el codo a `kp = 50` y 3 Nm de par
   gravitatorio son 0.06 rad = 3.4° de caída. Subir kp lo reduce; eliminarlo
   requiere `tau_ff` (compensación de gravedad), que este paquete deja
   preparado pero no calcula.
2. **`dq_des` importa.** Si se manda 0 mientras la consigna se mueve, el
   término `kd·(0 − dq)` frena el movimiento pedido. Ver
   [`01_DIAGNOSTICO.md`](01_DIAGNOSTICO.md), agravante (b).
3. **kp está limitado por el par del motor.** Un error de `tau_max/kp` ya
   satura. Con `kp = 140` en el codo (18 Nm) basta con **0.13 rad = 7.4°** de
   error para pedirle todo lo que tiene.

## Índices y topes

Los 27 motores, con sus topes del URDF `h1_2_handless.urdf`:

| idx | nombre | rango [rad] | tau_max [Nm] | dq_max [rad/s] |
|----:|--------|-------------|-------------:|---------------:|
| 0–11 | piernas | — | 200–300 (cadera/rodilla), 40–60 (tobillos) | 9–23 |
| 12 | `waist_yaw` (`torso_joint`) | −2.35 … 2.35 | 200 | 23 |
| 13 | `L_shoulder_pitch` | −3.14 … 1.57 | 40 | 9 |
| 14 | `L_shoulder_roll` | −0.38 … 3.40 | 40 | 9 |
| 15 | `L_shoulder_yaw` | −2.66 … 3.01 | **18** | 20 |
| 16 | `L_elbow` | −0.95 … 3.18 | **18** | 20 |
| 17 | `L_wrist_roll` | −3.01 … 2.75 | 19 | 31.4 |
| 18 | `L_wrist_pitch` | −0.4625 … 0.4625 | 19 | 31.4 |
| 19 | `L_wrist_yaw` | −1.27 … 1.27 | 19 | 31.4 |
| 20–26 | brazo derecho | espejo del izquierdo | | |

Dos cosas que sorprenden y conviene tener presentes:

* **El codo es de los motores más flojos del brazo**: 18 Nm, menos que la
  muñeca (19 Nm) y menos de la mitad que el hombro (40 Nm). Y es el que carga
  con el antebrazo y la mano Inspire.
* **`wrist_pitch` casi no tiene recorrido**: ±0.4625 rad = ±26.5°. Cualquier
  ensayo con amplitud 0.15 rad ya usa un tercio de su rango.

### Nomenclatura: tres fuentes, dos discrepancias

| motor | URDF / ejemplo oficial | `xr_teleoperate` |
|---|---|---|
| 1 | `left_hip_pitch` | `kLeftHipRoll` ❌ |
| 2 | `left_hip_roll` | `kLeftHipPitch` ❌ |
| 17 | `left_wrist_roll` | `kLeftElbowRoll` (mismo motor, otro nombre) |

La primera es un error de `xr_teleoperate` en las piernas; no afecta a los
brazos, pero conviene no copiar su enum. La segunda es solo un nombre distinto
para el mismo motor. Este paquete usa los nombres del URDF.

## El CRC

`rt/lowcmd` se descarta si el CRC no cuadra. No es `zlib.crc32`: es un CRC32
propio (polinomio `0x04C11DB7`) sobre el mensaje serializado con el *layout*
exacto del struct C++ —relleno incluido, y con `tau` **antes** de `kp`/`kd`—.

`rt/arm_sdk` no lo comprueba: el ejemplo oficial ni siquiera lo calcula. Aquí
se calcula igualmente en los dos canales, porque no cuesta nada y evita una
diferencia de comportamiento entre ellos.

**Rendimiento.** La implementación literal recorre bit a bit: 250 palabras ×
32 bits = 8000 vueltas de bucle Python por mensaje. Medido: 2.25 ms, con lo que
el lazo se quedaba en **185 Hz de los 250 pedidos**. `crc.py` la tabula
aprovechando que el paso del registro es lineal sobre GF(2), y baja a 0.29 ms
(serialización incluida). El lazo sube a 248 Hz. `crc32_reference()` conserva
la versión literal y hay una prueba que compara las dos sobre datos aleatorios.

## Frecuencia del lazo

| frecuencia | ciclos tarde | veredicto |
|---|---|---|
| 250 Hz | 0.5 % | cómodo. Es la de `xr_teleoperate` |
| 500 Hz | 2.2 % | justo. Python no da más con este mensaje |

Por defecto **250 Hz**, que además reproduce las condiciones de la
teleoperación. El ejemplo oficial de `arm_sdk` va a 50 Hz, así que hay margen
de sobra.
