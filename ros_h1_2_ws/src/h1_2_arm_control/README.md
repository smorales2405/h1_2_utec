# `h1_2_arm_control`

Control articular de los brazos del Unitree H1-2 por `unitree_hg/LowCmd`.

Las ganancias están sintonizadas sobre el robot real y la compensación de
gravedad usa parámetros de masa **identificados**, no los del URDF. La campaña
de medida que produjo estos números vive en el repositorio, en
`h1_2_joint_control/docs/06_RESULTADOS.md`; este paquete solo lleva el
resultado.

## Arrancar

```bash
cd ros_h1_2_ws
colcon build --symlink-install
source setup_env.sh
```

`setup_env.sh` hace falta: además del `install/setup.bash`, fija
`RMW_IMPLEMENTATION` y apunta `CYCLONEDDS_URI` a la NIC del robot. Sin eso el
nodo se crea pero no oye nada, o falla con un error que no dice por qué.

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

Una sesión típica:

```bash
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=enter
ros2 run h1_2_arm_control init_pose
#   ... aquí va tu algoritmo ...
ros2 run h1_2_arm_control rest_pose
ros2 run h1_2_arm_control debug_mode --ros-args -p action:=exit
```

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

La compensación necesita `pinocchio` y el URDF con manos, que **no está en este
repositorio**: viene de `github.com/oscar-ramos/h1_2_utec`, paquete
`h1_2_description`. Si no está en una ruta habitual:

```bash
export H12_URDF=/ruta/a/h1_2_description/urdf/h1_2.urdf
```

Si falta, el nodo avisa y sigue sin compensación en vez de caerse.

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
