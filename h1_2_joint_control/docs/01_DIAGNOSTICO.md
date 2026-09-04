# Diagnóstico: por qué el codo no se movía y por qué las muñecas vibraban

Fecha de las medidas: **2026-09-04**, portátil `mito` (192.168.123.51,
NIC `enp12s0`) contra el H1-2 (PC1 192.168.123.161, PC2 192.168.123.164).

Los dos síntomas —una articulación que no se mueve y otra que se mueve pero
tiembla— tienen la misma causa, y no son las ganancias.

---

## Resumen

**`/lowcmd` ya tiene dueño, y lo que manda es «par cero».** El controlador de
alto nivel (`ai`) publica en ese tópico **a 500 Hz de forma continua**, con
`kp = 0, kd = 0, tau = 0` en los 27 motores: los motores están habilitados pero
no aplican par. Un script que publique ahí a la vez no sustituye al servicio, se
alterna con él, y el bus de motores aplica el último mensaje que llega. El
resultado es que la articulación recibe, milisegundo a milisegundo,
alternativamente nuestras ganancias y **ganancias nulas**:

* A 250 Hz contra los 500 Hz del servicio, dos de cada tres mensajes anulan las
  ganancias. La articulación apenas ve un tercio del kp pedido → **no llega a su
  referencia**. Es el caso de `arm_joint_test.py`.
* A 500 Hz contra 500 Hz, la mitad. Suficiente para arrastrar la articulación,
  pero conmutando entre kp y 0 a centenares de hercios → **se mueve, pero
  vibrando**. Es el caso de `test_mandar_modificado.py`.

La solución no es tocar kp y kd, sino **quedarse solo en el canal**. Hay dos
formas, y solo una funciona en este robot:

| Vía | Resultado medido |
|---|---|
| `rt/arm_sdk` con peso 1 | **No hace nada.** Ver abajo |
| Soltar el controlador (`ReleaseMode`) y usar `rt/lowcmd` | **Funciona** |

### `arm_sdk` no sirve con el robot en reposo

`rt/arm_sdk` es el canal que Unitree diseñó para mover los brazos sin soltar la
locomoción, y el diagnóstico lo daba por bueno: 1 suscriptor, 0 publicadores,
canal libre. Nuestro publicador incluso empareja con él
(`get_subscription_count() == 1`). Pero **el servicio no aplica lo que recibe**.

Comprobado con cinco variantes del mensaje, todas con peso 1 y kp = 50 en
`L_wrist_yaw`, escalón de 0.08 rad:

| Variante | Recorrido | Par |
|---|---|---|
| Copia literal del ejemplo oficial (sin `mode`, sin `mode_machine`, sin CRC, 50 Hz) | 0.05 mrad | 0.06 Nm |
| Igual a 250 Hz | 0.06 mrad | 0.06 Nm |
| + `mode = 1` | 0.05 mrad | 0.06 Nm |
| + `mode_machine` | 0.05 mrad | 0.06 Nm |
| + CRC | 0.05 mrad | 0.06 Nm |

Ninguna mueve nada. La explicación más razonable es que el servicio solo mezcla
`arm_sdk` cuando el controlador de alto nivel está **realmente controlando**, y
aquí está en reposo con par cero. No se ha comprobado con el robot activo.

---

## Las medidas

### 1. `/lowcmd` está ocupado

```
$ python3 scripts/00_diagnose.py

  /lowstate    publicadores=1  suscriptores=4
  /lowcmd      publicadores=1  suscriptores=2
  /arm_sdk     publicadores=0  suscriptores=1

  /lowcmd lleva tráfico a 480 Hz y no lo ponemos nosotros
```

El publicador de `/lowcmd` no es ningún proceso nuestro: el diagnóstico solo
escucha. Son 500 Hz salidos del propio robot.

### 2. Quién es ese publicador

```
  CheckMode -> code=0  name='ai'  form='0'
```

El servicio `motion_switcher` responde que el controlador activo se llama
`ai`. Es el controlador de locomoción del H1-2 y es quien escribe en `/lowcmd`
mientras esté seleccionado.

### 3. `/arm_sdk` está libre y escuchando

```
  /arm_sdk    publicadores=0  suscriptores=1
```

Un suscriptor —el servicio `arm_sdk` del robot— y nadie hablándole.

---

## Por qué esto explica cada síntoma

### `arm_joint_test.py`: el codo no se movía

El script usa `H1_2_ArmController` de `xr_teleoperate` con
`motion_mode=False`, que en `teleop/robot_control/robot_arm.py` significa:

```python
kTopicLowCommand_Debug  = "rt/lowcmd"     # motion_mode = False   <-- este
kTopicLowCommand_Motion = "rt/arm_sdk"    # motion_mode = True
```

Publica en `rt/lowcmd` **a 250 Hz** (`control_dt = 1.0/250.0`) contra los
500 Hz del servicio `ai`. Por cada mensaje del script llegan dos del servicio.

El script llama a `Enter_Debug_Mode()` para soltar el servicio antes —que es lo
correcto, y lo que acaba funcionando—, pero él mismo admite que puede fallar y
sigue adelante de todos modos:

```python
print(f"  modo debug: {'OK' if status == 0 else 'FALLÓ (el servicio puede pelear el mando)'}")
```

Si esa llamada no prospera —y el `ai` seguía activo cuando se hicieron estas
medidas—, el codo se queda donde el servicio quiere. Exactamente lo observado.

> **Nota sobre este portátil.** `arm_joint_test.py` tampoco *podría* haberse
> ejecutado aquí: importa `unitree_sdk2py` y `teleop.robot_control.robot_arm`,
> y en `/home/mito` no está instalado ninguno de los dos. Además tiene la ruta
> cableada `sys.path.insert(0, "/home/utec/Documents/h1_2_teleoperation/…")`,
> del otro portátil. En esta máquina habría muerto en el `import`. La prueba
> que dio «el codo no se mueve» fue en la máquina `utec`, donde sí está el
> entorno conda `tv`.

### `test_mandar_modificado.py`: las muñecas se movían pero vibraban

Publica en `/lowcmd` a **500 Hz**, la misma frecuencia que el servicio. Dos
publicadores empatados: el motor recibe alternativamente el seno del script y
la postura del servicio. Eso es un temblor a decenas de Hz, que es justo lo
que se veía.

Que *sí* se moviera (a diferencia del codo) encaja: publicando a 500 Hz contra
500 Hz gana la mitad de las veces, suficiente para arrastrar la articulación,
pero no para llevarla limpiamente.

#### Dos agravantes independientes en ese mismo script

Aunque `/lowcmd` estuviera libre, ese script vibraría algo. Dos motivos:

**a) Salto de consigna en `t = 3 s`.** Al pasar de la etapa 1 a la 2 la
referencia salta de golpe:

```python
left_pitch_des = max_pitch * math.cos(2.0 * math.pi * t)   # t = 0  ->  0.25 rad
```

El tobillo venía de 0 rad y de repente se le pide 0.25 rad. Con `kp = 80` eso
son **20 Nm de golpe** en un motor cuyo límite son 60 Nm. Un escalón así excita
todos los modos mecánicos del robot.

**b) La velocidad de referencia va a cero mientras la consigna se mueve.**

```python
self._set_motor(H12JointIndex.LEFT_WRIST_ROLL, wrist_roll_des, 0.0, 50.0, 1.0, 0.0)
#                                                              ^^^ dq_des = 0
```

El motor calcula `tau = kp·(q_des − q) + kd·(0 − dq)`. Con un seno de 0.5 rad a
1 Hz la muñeca alcanza **3.14 rad/s**, y el término `kd·dq` frena con hasta
3.14 Nm en contra del movimiento que se le está pidiendo. Para vencer ese
freno el lazo necesita un error permanente de

```
err ≈ kd·dq / kp = 1.0 · 3.14 / 50 = 0.063 rad = 3.6°
```

es decir, un **12.6 % de la amplitud** de retraso puro, solo por no mandar la
velocidad de referencia. `xr_teleoperate` hace lo mismo
(`self.msg.motor_cmd[id].dq = 0`), así que conviene saber cuánto cuesta: por
eso `02_move.py` tiene `--zero-dq`, para medir la diferencia con y sin.

---

## Un tercer hallazgo, de camino

**No matar procesos `ros2` de la CLI.** El diagnóstico llamaba a
`ros2 topic hz` por `subprocess` con `timeout`, que al vencer mata el proceso.
Un participante DDS muerto a lo bruto no se despide, y el dominio local se
degrada mientras no vence su *lease*:

```
  antes de nada                  501.0 Hz
  tras 'topic info'              499.9 Hz
  tras 'topic hz' (matado)        62.6 Hz     <-- /lowstate se hunde
  10 s después                   355.7 Hz
```

Consecuencia práctica: si durante un ensayo alguien mata una herramienta `ros2`
en otra terminal, el lazo de control se queda sin estado fresco. El cliente lo
detecta (`lowstate_timeout = 0.5 s`) y suelta, pero mejor no provocarlo. Por eso
`00_diagnose.py` mide todo dentro del proceso, sin subprocesos.

**Corolario relacionado**: cerrar mal un nodo rclpy —destruir el nodo mientras
su ejecutor sigue girando— provoca `terminate called without an active
exception` y deja el participante DDS a medias. El síntoma no aparece en esa
ejecución sino en la **siguiente**, que no recibe `/lowstate`. `H12Client.shutdown()`
apaga en el orden correcto: parar el lazo → `executor.shutdown()` → `join` del
hilo → `destroy_node()` → `rclpy.shutdown()`.

---

## Qué hacer en su lugar

**Soltar el controlador de alto nivel y quedarse solo en `/lowcmd`.**

```bash
python3 scripts/06_debug_mode.py status   # ¿qué está mandando el servicio?
python3 scripts/06_debug_mode.py enter    # ⚠ robot COLGADO DEL ARNÉS
...ensayos...
python3 scripts/06_debug_mode.py exit     # devolver el mando a 'ai'
```

Lo que hace esto tolerable en este robot concreto es el resultado de `status`:
el servicio publica **kp = kd = 0 en los 27 motores**, así que ya no aplica par.
Soltarlo no cambia nada físicamente; lo único que cambia es que deja de
publicar y el canal queda libre. `06_debug_mode.py status` lo comprueba y lo
dice antes de dejarte entrar.

Con el robot **de pie sosteniéndose solo** la historia sería otra: ahí el
servicio sí aplicaría par y soltarlo lo tiraría. Por eso el aviso.

Verificado inmediatamente después de entrar: `/lowcmd` a 0 Hz, y el primer
escalón comandado (`L_wrist_yaw`, 0.10 rad, kp = 50) sale limpio —48 ms de
subida, 0 % de sobreimpulso, 0.16° de error final, sin temblor—, contra el
mismo ensayo por `arm_sdk` unos minutos antes, donde el error se quedaba en los
100 mrad comandados.

Ver [`02_ARQUITECTURA.md`](02_ARQUITECTURA.md) para el detalle de los canales y
[`06_RESULTADOS.md`](06_RESULTADOS.md) para las medidas.
