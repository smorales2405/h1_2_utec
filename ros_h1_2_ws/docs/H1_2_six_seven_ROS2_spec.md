# Especificación para implementar el gesto “six-seven” en Unitree H1-2 con ROS 2

## Objetivo

Implementar un script ROS 2 dentro del paquete:

```text
ros_h1_2_ws/src/h1_2_arm_control
```

que permita al humanoide **Unitree H1-2** ejecutar el gesto conocido como **“six-seven”** usando ambos brazos.

El gesto debe verse como un movimiento de balanza:

- ambas manos frente al torso;
- palmas orientadas hacia arriba;
- cuando la mano izquierda sube, la derecha baja;
- cuando la mano derecha sube, la izquierda baja;
- el movimiento debe ser suave, periódico y simétrico.

La implementación debe reutilizar la infraestructura ya existente en el repositorio, especialmente:

```text
h1_2_joint_control
ros_h1_2_ws/src/h1_2_arm_control
```

y evitar crear un controlador de bajo nivel desde cero si ya existe funcionalidad equivalente en `H12Client`, `trajectories.py`, `joints.py`, etc.

---

# 1. Archivos del repositorio que deben revisarse

Antes de implementar, revisar especialmente:

```text
ros_h1_2_ws/src/h1_2_arm_control/h1_2_arm_control/arm_client.py
ros_h1_2_ws/src/h1_2_arm_control/h1_2_arm_control/joints.py
ros_h1_2_ws/src/h1_2_arm_control/h1_2_arm_control/trajectories.py
ros_h1_2_ws/src/h1_2_arm_control/config/gains.yaml
ros_h1_2_ws/src/h1_2_arm_control/README.md
```

También revisar el subfolder:

```text
h1_2_joint_control
```

para respetar la forma en que el repositorio ya maneja:

- índices de articulaciones;
- `q`;
- `dq`;
- `kp`;
- `kd`;
- `tau`;
- límites de posición;
- límites de velocidad;
- publicación de comandos;
- rampas suaves;
- modo `arm_sdk`;
- protecciones de autocolisión.

---

# 2. Articulaciones de interés

El gesto no necesita mover todas las articulaciones continuamente.

Las articulaciones principales son:

| Articulación | Índice aproximado usado en el proyecto | Uso |
|---|---:|---|
| `L_shoulder_pitch` | 13 | movimiento principal de subida/bajada |
| `L_shoulder_roll` | 14 | separar brazo izquierdo del torso |
| `L_shoulder_yaw` | 15 | mantener inicialmente fijo |
| `L_elbow` | 16 | mantener antebrazo flexionado |
| `L_wrist_roll` | 17 | orientar palma izquierda hacia arriba |
| `L_wrist_pitch` | 18 | mantener inicialmente fijo |
| `L_wrist_yaw` | 19 | mantener inicialmente fijo |
| `R_shoulder_pitch` | 20 | movimiento principal de subida/bajada |
| `R_shoulder_roll` | 21 | separar brazo derecho del torso |
| `R_shoulder_yaw` | 22 | mantener inicialmente fijo |
| `R_elbow` | 23 | mantener antebrazo flexionado |
| `R_wrist_roll` | 24 | orientar palma derecha hacia arriba |
| `R_wrist_pitch` | 25 | mantener inicialmente fijo |
| `R_wrist_yaw` | 26 | mantener inicialmente fijo |

**Importante:** confirmar los índices directamente desde `joints.py` y usar las constantes existentes del paquete en lugar de hardcodearlos si ya están disponibles.

---

# 3. Postura base recomendada

Antes de iniciar la oscilación, llevar ambos brazos suavemente a una postura inicial.

## Shoulder pitch

Posición central:

```text
-0.75 rad
```

equivalente aproximadamente a:

```text
-43°
```

La oscilación deberá realizarse alrededor de este valor.

---

## Shoulder roll

Usar:

```text
L_shoulder_roll = +0.30 rad
R_shoulder_roll = -0.30 rad
```

aproximadamente:

```text
izquierdo: +17°
derecho:   -17°
```

La intención es separar ligeramente ambos brazos del torso y reducir riesgo de autocolisión.

No usar shoulder roll como articulación principal del gesto en la primera versión.

---

## Shoulder yaw

Mantener inicialmente:

```text
L_shoulder_yaw = 0.0 rad
R_shoulder_yaw = 0.0 rad
```

---

## Elbows

Usar inicialmente:

```text
L_elbow = 0.45 rad
R_elbow = 0.45 rad
```

aproximadamente:

```text
26°
```

Mantener los codos fijos durante la primera versión.

Posteriormente se puede añadir una oscilación pequeña para hacer el gesto más natural.

---

## Wrist pitch

Inicialmente:

```text
L_wrist_pitch = 0.0 rad
R_wrist_pitch = 0.0 rad
```

---

## Wrist yaw

Inicialmente:

```text
L_wrist_yaw = 0.0 rad
R_wrist_yaw = 0.0 rad
```

---

# 4. Orientación de las palmas

El gesto requiere que las palmas miren aproximadamente hacia arriba.

Usar principalmente:

```text
L_wrist_roll
R_wrist_roll
```

Como referencia inicial, probar magnitudes de:

```text
1.2 a 1.5 rad
```

equivalentes aproximadamente a:

```text
69° a 86°
```

Los signos probablemente serán opuestos para izquierda y derecha.

Sin embargo, **NO hardcodear definitivamente los signos sin verificar el montaje físico de las manos Inspire RH56DFTP**.

Las manos Inspire están montadas mediante una adaptación, por lo que el cero visual de la palma puede diferir del cero del joint del H1-2.

Idealmente definir:

```python
PALM_UP_L = ...
PALM_UP_R = ...
```

como constantes configurables al inicio del script.

Primero probar manualmente cada wrist roll, lentamente, desde cero, hasta identificar qué signo y qué ángulo colocan cada palma horizontal hacia arriba.

---

# 5. Movimiento principal del gesto

La articulación principal debe ser:

```text
shoulder_pitch
```

El movimiento debe ser sinusoidal y en contrafase entre brazo izquierdo y derecho.

## Parámetros nominales

Posición central:

```text
q0 = -0.75 rad
```

Amplitud:

```text
A = 0.15 rad
```

Frecuencia:

```text
f = 0.75 Hz
```

Frecuencia angular:

```text
omega = 2*pi*f
```

---

## Brazo izquierdo

```text
q_L(t) = -0.75 - 0.15*sin(2*pi*0.75*t)
```

---

## Brazo derecho

```text
q_R(t) = -0.75 + 0.15*sin(2*pi*0.75*t)
```

Esto genera exactamente el comportamiento:

```text
izquierdo arriba  -> derecho abajo
derecho arriba    -> izquierdo abajo
```

---

# 6. Rango angular

Cada shoulder pitch se moverá aproximadamente entre:

```text
-0.90 rad
-0.60 rad
```

equivalente aproximadamente a:

```text
-51.6°
-34.4°
```

Por tanto:

```text
L_shoulder_pitch: -0.90 <-> -0.60 rad
R_shoulder_pitch: -0.60 <-> -0.90 rad
```

siempre en contrafase.

---

# 7. Velocidad articular

La derivada de una sinusoidal es:

```text
dq/dt = A*omega*cos(omega*t)
```

Por tanto, la velocidad máxima del shoulder pitch será:

```text
dq_max = A * 2*pi*f
```

Con:

```text
A = 0.15 rad
f = 0.75 Hz
```

se obtiene aproximadamente:

```text
dq_max = 0.71 rad/s
```

Usar este valor como velocidad pico nominal del gesto.

---

# 8. Límites de velocidad recomendados

Para este script:

```text
transición inicial a postura: 0.25 - 0.35 rad/s
shoulder pitch durante gesto: hasta ~0.71 rad/s
elbow opcional: 0.20 - 0.30 rad/s
wrist roll al adoptar postura: 0.25 - 0.40 rad/s
```

No superar en este script:

```text
1.0 rad/s
```

aunque los motores tengan límites físicos superiores.

El paquete ya maneja aproximadamente ese valor como `max_ref_velocity`, así que reutilizar esa protección.

---

# 9. Trayectoria de velocidad

No enviar solamente `q(t)` dejando:

```python
dq = 0.0
```

durante una trayectoria dinámica.

Si el controlador usa término derivativo:

```text
Kd * (dq_ref - dq_actual)
```

poner `dq_ref = 0` mientras la articulación se está moviendo hace que el término derivativo se oponga al movimiento.

Por tanto, enviar simultáneamente:

## Izquierdo

```text
q_L(t)  = -0.75 - 0.15*sin(omega*t)
dq_L(t) = -0.15*omega*cos(omega*t)
```

## Derecho

```text
q_R(t)  = -0.75 + 0.15*sin(omega*t)
dq_R(t) = +0.15*omega*cos(omega*t)
```

donde:

```text
omega = 2*pi*0.75
```

---

# 10. Elbow opcional para una versión más natural

Primera versión:

```text
L_elbow = 0.45 rad
R_elbow = 0.45 rad
```

constantes.

Una vez validada la versión básica, opcionalmente añadir:

```text
L_elbow(t) = 0.45 - 0.05*sin(omega*t)
R_elbow(t) = 0.45 + 0.05*sin(omega*t)
```

Rango:

```text
0.40 a 0.50 rad
```

Velocidad máxima aproximada:

```text
0.24 rad/s
```

No incluir esta oscilación en la primera prueba salvo que sea trivial parametrizarla.

---

# 11. Duración del gesto

Crear un parámetro configurable:

```text
duration
```

Valor inicial recomendado:

```text
4.0 s
```

Con:

```text
f = 0.75 Hz
```

esto produce aproximadamente:

```text
3 ciclos completos
```

También puede implementarse por número de ciclos:

```text
cycles = 3
```

y calcular:

```text
duration = cycles / f
```

---

# 12. Secuencia completa deseada

La ejecución debería seguir esta secuencia:

## Etapa 1 - leer estado actual

Obtener posiciones actuales de las articulaciones.

No asumir que el robot empieza exactamente desde cero.

---

## Etapa 2 - llevar brazos a postura inicial

Mover suavemente hacia:

```text
L_shoulder_pitch = -0.75
R_shoulder_pitch = -0.75

L_shoulder_roll = +0.30
R_shoulder_roll = -0.30

L_shoulder_yaw = 0.0
R_shoulder_yaw = 0.0

L_elbow = 0.45
R_elbow = 0.45

L_wrist_roll = PALM_UP_L
R_wrist_roll = PALM_UP_R

L_wrist_pitch = 0.0
R_wrist_pitch = 0.0

L_wrist_yaw = 0.0
R_wrist_yaw = 0.0
```

Usar `ramp_to()` o equivalente existente.

La transición debe ser suave.

---

## Etapa 3 - ejecutar six-seven

Durante `duration` segundos:

- generar posición y velocidad sinusoidal;
- actualizar referencias a una frecuencia suficientemente alta;
- mantener ambos shoulder pitch exactamente en contrafase;
- mantener el resto de articulaciones en la postura base;
- respetar límites articulares;
- respetar `max_ref_velocity`;
- reutilizar protecciones de autocolisión existentes.

---

## Etapa 4 - finalizar en postura neutra del gesto

Al terminar:

- no cortar abruptamente la referencia;
- finalizar en un punto con velocidad cercana a cero;
- volver suavemente ambos shoulder pitch a:

```text
-0.75 rad
```

---

## Etapa 5 - opcionalmente volver a postura previa

Si la arquitectura del paquete lo permite, incluir una opción como:

```text
--return-home
```

para volver suavemente a una postura segura o a la posición original.

No debe ser obligatorio en la primera implementación.

---

# 13. Canal de control

Preferir el mecanismo recomendado actualmente por el paquete para controlar solamente los brazos:

```text
arm_sdk
```

Evitar comandar directamente `/lowcmd` salvo que se esté trabajando deliberadamente en modo de bajo nivel, robot colgado del arnés y condiciones de laboratorio apropiadas.

No crear una implementación paralela que ignore `H12Client` si la infraestructura existente ya soporta el movimiento requerido.

---

# 14. Seguridad

La implementación debe incluir o reutilizar:

- saturación de límites articulares;
- saturación de velocidad;
- protecciones de autocolisión;
- parada limpia con `Ctrl+C`;
- transición suave al iniciar;
- transición suave al terminar;
- validación de que se recibe estado válido antes de enviar comandos.

No enviar saltos instantáneos de posición.

No usar:

```text
q_des = valor_final
```

de forma abrupta desde una postura desconocida.

---

# 15. Parámetros configurables

Sería recomendable que el script exponga al menos:

```python
FREQUENCY = 0.75
AMPLITUDE = 0.15
CENTER_PITCH = -0.75

LEFT_SHOULDER_ROLL = +0.30
RIGHT_SHOULDER_ROLL = -0.30

ELBOW_CENTER = 0.45

PALM_UP_L = ...
PALM_UP_R = ...

DURATION = 4.0
MAX_REF_VELOCITY = 1.0
```

Preferiblemente como parámetros ROS 2 o argumentos CLI si el paquete ya sigue ese patrón.

---

# 16. Archivo sugerido

Crear:

```text
ros_h1_2_ws/src/h1_2_arm_control/h1_2_arm_control/six_seven.py
```

Si corresponde, añadir el entry point necesario en:

```text
setup.py
```

para poder ejecutar algo similar a:

```bash
ros2 run h1_2_arm_control six_seven
```

---

# 17. Variante con argumentos

Idealmente permitir:

```bash
ros2 run h1_2_arm_control six_seven \
  --ros-args \
  -p frequency:=0.75 \
  -p amplitude:=0.15 \
  -p duration:=4.0
```

Opcionalmente:

```text
palm_up_left
palm_up_right
center_pitch
elbow_center
```

---

# 18. Frecuencia de actualización

Usar la frecuencia de control ya recomendada por `H12Client` o el paquete.

No introducir una frecuencia arbitraria más baja si ya existe un loop de control apropiado.

Si se necesita definir una frecuencia explícita, usar una tasa suficientemente alta para que la sinusoidal sea suave, por ejemplo del orden de:

```text
100 - 200 Hz
```

pero respetando la arquitectura existente.

---

# 19. Primera versión mínima recomendada

La primera versión debería usar solamente:

```text
shoulder_pitch -> sinusoidal
shoulder_roll  -> constante
elbow          -> constante
wrist_roll     -> constante para palma arriba
```

y mantener:

```text
shoulder_yaw = 0
wrist_pitch  = 0
wrist_yaw    = 0
```

Esto permite validar rápidamente la coordinación sin introducir demasiadas articulaciones móviles.

---

# 20. Valores finales nominales

## Izquierdo

```text
L_shoulder_pitch:
    centro = -0.75 rad
    amplitud = 0.15 rad
    rango = [-0.90, -0.60] rad
    velocidad pico = ~0.71 rad/s

L_shoulder_roll:
    +0.30 rad

L_shoulder_yaw:
    0.0 rad

L_elbow:
    0.45 rad

L_wrist_roll:
    PALM_UP_L ≈ magnitud 1.2 a 1.5 rad

L_wrist_pitch:
    0.0 rad

L_wrist_yaw:
    0.0 rad
```

## Derecho

```text
R_shoulder_pitch:
    centro = -0.75 rad
    amplitud = 0.15 rad
    rango = [-0.90, -0.60] rad
    velocidad pico = ~0.71 rad/s
    fase opuesta al izquierdo

R_shoulder_roll:
    -0.30 rad

R_shoulder_yaw:
    0.0 rad

R_elbow:
    0.45 rad

R_wrist_roll:
    PALM_UP_R ≈ magnitud 1.2 a 1.5 rad
    signo probablemente opuesto al izquierdo

R_wrist_pitch:
    0.0 rad

R_wrist_yaw:
    0.0 rad
```

---

# 21. Ecuaciones finales

Definir:

```text
f = 0.75 Hz
A = 0.15 rad
q0 = -0.75 rad
omega = 2*pi*f
```

Brazo izquierdo:

```text
qL(t)  = q0 - A*sin(omega*t)
dqL(t) = -A*omega*cos(omega*t)
```

Brazo derecho:

```text
qR(t)  = q0 + A*sin(omega*t)
dqR(t) = +A*omega*cos(omega*t)
```

Estas ecuaciones deben ser la base del movimiento principal.

---

# 22. Criterios de aceptación

La implementación se considerará correcta si:

1. ambos brazos entran suavemente a la postura inicial;
2. las palmas quedan aproximadamente orientadas hacia arriba;
3. los brazos se mueven en contrafase;
4. el movimiento principal está concentrado en `shoulder_pitch`;
5. el rango nominal es aproximadamente `-0.90 a -0.60 rad`;
6. la velocidad pico se mantiene alrededor de `0.71 rad/s`;
7. no se supera `1.0 rad/s`;
8. no hay saltos bruscos al inicio o al final;
9. se envían tanto `q_ref` como `dq_ref`;
10. se reutilizan las protecciones existentes del paquete;
11. `Ctrl+C` termina de forma segura;
12. el script puede ejecutarse mediante ROS 2 con una interfaz simple.

---

# 23. Recomendación de implementación

No desarrollar un nodo independiente ignorando la arquitectura existente.

Prioridad:

```text
H12Client
ARM_INDICES
joint limits
ramp_to()
trajectory helpers
self-collision protection
existing gains
arm_sdk
```

La lógica nueva debe concentrarse principalmente en:

```text
1. postura inicial
2. generación q(t)
3. generación dq(t)
4. sincronización izquierda/derecha
5. duración
6. finalización suave
```

---

# 24. Prueba física recomendada

Antes de ejecutar el gesto completo:

1. probar solamente la postura inicial;
2. verificar shoulder roll izquierdo y derecho;
3. verificar signo de ambos wrist roll;
4. verificar que las palmas realmente estén orientadas hacia arriba;
5. ejecutar shoulder pitch con amplitud pequeña:

```text
A = 0.05 rad
```

6. después aumentar a:

```text
A = 0.10 rad
```

7. finalmente usar:

```text
A = 0.15 rad
```

Mantener inicialmente:

```text
f = 0.5 Hz
```

para validación lenta.

Una vez verificado el movimiento, subir a:

```text
f = 0.75 Hz
```

---

# 25. Resultado esperado

Visualmente, el movimiento debe parecer:

```text
       izquierda       derecha

t0     media           media
t1     arriba          abajo
t2     media           media
t3     abajo           arriba
t4     media           media
```

repetido periódicamente.

La intención es reproducir el movimiento de balanza de las manos característico del gesto “six-seven”, manteniendo el torso y las piernas sin movimiento adicional.
