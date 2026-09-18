# `h1_2_arm_control`

Control articular de los brazos del Unitree H1-2 por `unitree_hg/LowCmd`.

Las ganancias están sintonizadas sobre el robot real y la compensación de
gravedad usa parámetros de masa **identificados**, no los del URDF. La campaña
de medida que produjo estos números vive en el repositorio, en
`h1_2_joint_control/docs/06_RESULTADOS.md`; este paquete solo lleva el
resultado.

## Estado

Probado **sobre el robot real** el 2026-09-14, colgado del arnés:

| | |
|---|---|
| `init_pose` | ✔ los catorce llegan a 0° |
| `rest_pose` | ✔ estira los brazos y los devuelve suavemente, sin rozar la pierna |
| `algorithm_template` | ✔ ciclo entero: colocar → algoritmo → devolver, en un proceso |

Con eso queda ejercitado casi todo lo que hay debajo: `engage`/`release` en
rampa, `ramp_to`, `set_trajectory`, la compensación de gravedad, el portero de
autocolisión y las dos secuencias de postura.

`goto`, `move_joint` y `hold` **no se han ejecutado como tales**. Son envoltorios
delgados sobre esas mismas llamadas, así que el riesgo es bajo, pero no es lo
mismo que haberlos corrido.

## Arrancar

```bash
cd ros_h1_2_ws
colcon build --symlink-install
source setup_env.sh
```

`setup_env.sh` hace falta, y por tres cosas: además del `install/setup.bash`,
fija `RMW_IMPLEMENTATION`, apunta `CYCLONEDDS_URI` a la NIC del robot, y aísla
el proceso de `~/.local`. Sin lo primero el nodo se crea pero no oye nada; sin
lo último **se muere con un segfault**.

### El segfault de NumPy, si te lo encuentras

```
AttributeError: _ARRAY_API not found
[ros2run]: Segmentation fault
```

En esta máquina conviven `numpy 1.21.5` (paquete Debian, en
`/usr/lib/python3/dist-packages`) y `numpy 2.2.6` (pip, en `~/.local`). Como
`~/.local` va antes en `sys.path`, gana el 2.2.6 — pero `ros-humble-pinocchio`,
y en general toda extensión C de ROS Humble, está compilada contra la ABI de
NumPy 1.x. Importarla entonces no lanza una excepción que se pueda atrapar:
mata el proceso.

**No lo arregles desinstalando numpy 2.2.6**: `opencv-python`, en ese mismo
`~/.local`, lo exige. `setup_env.sh` exporta `PYTHONNOUSERSITE=1`, con lo que
solo los procesos de este workspace ignoran `~/.local` y ven el numpy de
Debian. El resto del sistema se queda como está.

Si ya tienes la terminal abierta y no quieres volver a sourcear:

```bash
PYTHONNOUSERSITE=1 ros2 run h1_2_arm_control six_seven --ros-args -p duration:=8.0
```

`gravity.py` comprueba esa combinación antes de importar `pinocchio` y se niega
con un mensaje en vez de reventar.

## Lo primero, siempre

```bash
ros2 run h1_2_arm_control read_state          # ¿dónde está? (no publica nada)
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=status
```

**El canal `/lowcmd` tiene dueño.** El controlador `ai` del robot publica ahí a
500 Hz; si publicamos a la vez, los mensajes se alternan y el motor recibe
consignas contradictorias: la articulación ve una fracción de la ganancia y
vibra. Hay que soltarlo antes:

```bash
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=enter   # robot COLGADO
```

y al terminar la sesión, `-p action:=exit`.

## Nodos

| nodo | qué hace |
|---|---|
| `read_state` | lee y muestra el estado. Solo lectura |
| `debug_mode` | suelta o recupera el controlador de alto nivel |
| `hold` | sostiene los brazos donde estén. Comprueba que el control va |
| `init_pose` | los catorce a **0°**, despacio y en el orden que no choca |
| `goto` | lleva las articulaciones que se le digan a los ángulos que se le digan |
| `move_joint` | trayectoria de referencia sobre una articulación |
| `rest_pose` | vuelta a la postura de reposo **sin que la mano roce la pierna** |

| `algorithm_template` | plantilla: colocar → tu algoritmo → devolver, en UN proceso |
| `fk_right_arm_raise` | ensayo del brazo derecho, para comparar con el simulador |
| `plot_fk_raise` | gráficas de ese ensayo; superpone varios CSV |
| `fk_check` | comprueba que la cinemática es la misma que la del simulador |
| `six_seven` | el gesto de balanza con las dos manos, palmas arriba |

### `six_seven`

Balanza con las dos manos, palmas arriba, en contrafase. Se mueve **una sola
articulación** —el codo o el hombro-pitch— y el resto se queda clavado.

```bash
ros2 run h1_2_arm_control six_seven --ros-args \
    -p moving_joint:=elbow \
    -p shoulder_roll_deg:=5.0 \
    -p shoulder_pitch_deg:=0.0 \
    -p elbow_deg:=0.0 \
    -p amplitude_deg:=20.0 \
    -p speed:=0.5 \
    -p duration:=8.0
```

Ángulos en **grados**, velocidad en **rad/s**.

| parámetro | qué es |
|---|---|
| `moving_joint` | `elbow` o `shoulder_pitch`: la que oscila |
| `shoulder_roll_deg` | fijo, en valor absoluto (el signo lo pone el lado) |
| `shoulder_pitch_deg` | fijo si oscila el codo; referencia si oscila él |
| `elbow_deg` | fijo si oscila el hombro; referencia si oscila él |
| `amplitude_deg` | ± desde la referencia, lo mismo arriba que abajo |
| `speed` | velocidad angular de **pico**, rad/s |
| `duration` | segundos |
| `palm_up_left_deg`, `palm_up_right_deg` | orientación de las palmas |
| `approach_speed` | rad/s de las rampas de colocación |
| `return_home` | volver a reposo al terminar |

**La frecuencia no se pide: sale sola.** Para un seno de amplitud A la
velocidad de pico es A·ω, así que `f = speed / (2π · amplitude)`. El script la
calcula, la enseña y comprueba que la articulación puede seguirla.

#### Qué velocidad poner

El techo no es un número fijo: depende de la amplitud, porque lo que limita es
la **frecuencia** que sale de ella. Los anchos de banda están medidos (F5,
−3 dB, chirp logarítmico con coherencia ≥ 0.98):

```
shoulder_pitch   1.21 Hz          elbow   3.07 Hz
```

Manteniéndose en BW/3 el seguimiento es fiel. De ahí:

| amplitud | `shoulder_pitch` | `elbow` |
|---:|---:|---:|
| 5° | 0.22 | 0.56 |
| 10° | 0.44 | **1.00** |
| 20° | 0.88 | **1.00** |
| 30° | **1.00** | **1.00** |

En negrita, donde manda el tope de `max_ref_velocity` (1.0 rad/s) y no el
ancho de banda. Por debajo de **0.05 rad/s** el movimiento queda cerca del
ruido de velocidad medido (0.007–0.014 rad/s) y sale a tirones.

#### Antes de mover nada comprueba

Topes de la postura fija **y de los extremos de la oscilación**; autocolisión
**en todo el recorrido**, no solo en la referencia —con el codo oscilando, el
`|roll|` que la envolvente exige crece con la extensión—; velocidad de pico
contra `max_ref_velocity`; frecuencia contra el ancho de banda medido; y el
margen de par que deja la gravedad. Si algo no cuadra lo dice, dice cuánto
falta, y no ejecuta.

## ⚠ No encadenes comandos para meter tu algoritmo en medio

Lo natural sería esto, y **no funciona**:

```bash
ros2 run h1_2_arm_control init_pose
ros2 run mi_paquete mi_algoritmo        # <-- aquí el brazo YA se cayó
ros2 run h1_2_arm_control rest_pose
```

Cuando `init_pose` termina, baja las ganancias a cero y el proceso muere. Sin
nadie publicando, la gravedad se lleva los codos: **medido, de 1° a 79° y 85°
en unos segundos**. El siguiente comando tarda varios segundos en arrancar
—crear el nodo, esperar el primer `/lowstate`, enganchar en rampa— y para
entonces el brazo ya no está donde lo dejaron.

No es un fallo que se pueda tapar: mientras nadie mande, el brazo cae. La única
forma de que no haya hueco es que **el mismo proceso** coloque, ejecute y
devuelva. Para eso está `algorithm_template.py`: cópialo a tu paquete y cambia
`mi_algoritmo()` por lo tuyo.

```bash
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=enter
ros2 run h1_2_arm_control algorithm_template     # coloca, ejecuta y devuelve
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=exit
```

Si lo que quieres es solo dejar el brazo firme en 0° para mirarlo, `init_pose`
tiene `-p keep:=true`, que lo sostiene hasta Ctrl-C. Pero mientras esté
sosteniendo, ningún otro proceso puede publicar en `/lowcmd` sin pelearse por
el canal.

### Por qué `init_pose` va en tres tramos

El tope de autocolisión del hombro **depende del codo**: ±10° con el brazo
estirado, 0° con el codo flexionado. Así que hay que flexionar primero. En un
solo tramo, los hombros se quedan topando en ±10° sin llegar a cero.

### Por qué `rest_pose` no es «ir a la postura de reposo»

Soltar los brazos en 0° hace que **la mano golpee la pierna**. 0° de codo es la
posición *flexionada*, que no es el mínimo de gravedad: sin ganancia el
antebrazo cae solo hasta quedar colgando —medido, de 1° a 79° y 85° en
segundos— y en esa caída la mano recorre la pierna.

Por eso la secuencia es hombros fuera → codos estirados → reposo. Medido:
soltando en 0° la deriva es de **84°**; soltando así, de **1.7°**.

## Ganancias

`config/gains.yaml`. El conjunto activo se marca en `active` y se puede
sobrescribir con `-p gains:=...`.

| conjunto | cuándo |
|---|---|
| **`tuned_gff`** | el bueno. **Exige `gravity:=true`** (que es el valor por defecto) |
| `tuned` | sin compensación de gravedad. Más kp, más error permanente |
| `official_arm_sdk`, `xr_teleoperate`, `ros2_example` | los de referencia, para comparar |

> ⚠ **`tuned_gff` sin gravedad es peor que `tuned`.** Sus kp son bajos porque el
> par de sostenimiento lo da el feedforward; sin él, el error permanente
> `tau_g/kp` crece justo donde más pesa. Si no puedes usar `pinocchio`, cambia
> el conjunto, no apagues la gravedad.

La compensación necesita `pinocchio` y un URDF, que sale del paquete
**`h1_2_inspire_description` de este mismo workspace**
(`urdf/h1_2_with_RH56DFTP_hands.urdf`). Se resuelve por el índice de ament, así
que basta con `colcon build` y `source install/setup.bash`. Para probar otro
modelo sin tocar nada:

```bash
export H12_URDF=/ruta/a/otro.urdf
```

Si falta, el nodo avisa y sigue sin compensación en vez de caerse.

### Sobre el URDF

**Las masas del URDF no se usan para los brazos.** Al construir el modelo, los
parámetros identificados sobrescriben las inercias de los cuerpos del brazo, así
que lo único que el URDF aporta ahí es la CINEMÁTICA. Medido:

| masa de la mano | izquierda | derecha |
|---|---:|---:|
| `h1_2.urdf` (el que se usaba antes) | 0.316 kg | 0.316 kg |
| `h1_2_with_RH56DFTP_hands.urdf` (el de ahora) | 0.964 kg | 0.964 kg |
| **identificada sobre el robot** | **1.138 kg** | **1.074 kg** |

La identificada es la mayor de las tres, y es la que vale: el robot también
carga el conector y el cable, que ningún URDF modela.

El cambio de `h1_2.urdf` a `h1_2_with_RH56DFTP_hands.urdf` movió el par de
gravedad **0.47 Nm como mucho** —medido en tres posturas—; con kp = 111 eso son
0.24° de error permanente, dentro del ruido de lo que ya se mide. **No hubo que
resintonizar.** Lo que sí costaría caro es usar un URDF *sin* los parámetros
identificados: ahí la diferencia llega a **5.16 Nm**.

> ⚠ Los parámetros se cargan **por índice de cuerpo**, y ese índice vale para el
> URDF con el que se identificaron. Otro URDF puede aceptar los mismos índices
> y poner la masa en el eslabón equivocado sin que nada falle. Por eso
> `GravityModel` comprueba que las siete articulaciones del brazo estén donde
> deben antes de cargar nada, y si no cuadran deja ese brazo **sin compensar**
> en vez de compensar mal.

## Usarlo desde un algoritmo

Los nodos son para poner el robot en marcha. Para escribir algoritmos, lo que
importa es el cliente:

```python
from h1_2_arm_control.arm_client import H12Client
from h1_2_arm_control.joints import ARM_INDICES, BY_NAME
from h1_2_arm_control import trajectories as traj

with H12Client(controlled=ARM_INDICES, channel="lowcmd") as cli:
    cli.wait_for_state()
    cli.engage()                                   # toma el control en rampa
    cli.ramp_to({BY_NAME["L_elbow"].idx: 0.0}, speed=0.15)
    cli.set_trajectory(BY_NAME["L_elbow"].idx, traj.sine(0.12, 0.5))
    cli.sleep(6.0)
    cli.release()                                  # devuelve el control en rampa
```

El `with` importa: al salir —también por Ctrl-C o por una excepción— baja las
ganancias en rampa. Si el proceso muere dejando de publicar de golpe, los
motores se quedan con la última consigna.

## Lo que el cliente vigila solo

- **Par** sostenido por encima del 70 % del máximo durante 0.3 s → aborta.
- **Temperatura** por encima de 80 °C → aborta.
- **Estado rancio**: si `/lowstate` deja de llegar 0.5 s → aborta.
- **Autocolisión**: la consigna del hombro se recorta según dónde esté el codo.
  Se mira `q_des`, no `q`: cuando la articulación real ha llegado a una postura
  en colisión ya es tarde.
- **Piernas**: en `/lowcmd` nadie más manda, así que `legs:=hold` las sostiene.
  Con `free` se mueven solas por la reacción del brazo — medido.

## Validar el simulador contra el robot

`fk_right_arm_raise` es la contraparte de `h1_2_algoritms/fk_right_arm_raise.py`,
que hace lo mismo sobre MuJoCo. Lleva los dos brazos a 0°, recorre una
trayectoria quíntica hasta `q_goal` con los 7 motores del brazo derecho,
registra a 100 Hz y devuelve los brazos a reposo — **todo en un proceso**.

```bash
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=enter
ros2 run h1_2_arm_control fk_right_arm_raise
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=exit

ros2 run h1_2_arm_control plot_fk_raise <csv_real> \
    --compare <csv_sim> --labels "real,simulación"
```

El CSV tiene **las mismas 50 columnas** que el del simulador, en el mismo
orden, y la trayectoria y sus valores por defecto también son los mismos. Los
dos se superponen sin tocar nada.

### Por qué la comparación es válida

La cinemática de `fk.py` es **la misma tabla DH** que usa el simulador, copiada
de `h1_2_algoritms.fk_functions`. Está copiada y no importada porque los dos
paquetes viven en workspaces distintos; si un día cambia allí, hay que traer el
cambio aquí — y `fk_check` lo detecta.

```bash
ros2 run h1_2_arm_control fk_check
```

| comprobación | resultado |
|---|---|
| contra la DH de `h1_2_algoritms`, ambos brazos | **0.00e+00** (idéntica) |
| contra el URDF de `h1_2_inspire_description`, con pinocchio | 0.0003 mm, **0.0000°** |
| contra las columnas `ee_*` de los CSV del simulador (1200 filas) | **0.0000 mm**, 0.0229° † |

† esos 0.0229° son la precisión con la que el CSV guarda los decimales.

Esto no es una formalidad. Al portar la tabla se intercambió el signo del
offset de hombro entre brazos, y costaba **30° exactos** de orientación y
253 mm de posición. Lo encontró la comprobación, no una lectura del código.

El cuaternión se calcula con la **misma** implementación que el simulador, a
propósito: su signo es ambiguo —q y −q son la misma rotación— y dos versiones
distintas darían columnas con el signo cambiado que parecerían un salto.
