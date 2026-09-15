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

| `algorithm_template` | plantilla: colocar → tu algoritmo → devolver, en UN proceso |
| `six_seven` | el gesto de balanza con las dos manos, palmas arriba |

### `six_seven`

```bash
ros2 run h1_2_arm_control six_seven --ros-args -p duration:=8.0
```

Hace el ciclo entero: coloca en 0°, adopta la postura del gesto, oscila los dos
hombros en contrafase y vuelve a reposo. La amplitud entra y sale con una
envolvente de coseno alzado, así que **empieza y acaba en el centro con
velocidad exactamente nula** en vez de cortarse a media carrera.

Parámetros: `duration`, `frequency`, `amplitude`, `center_pitch`,
`shoulder_roll`, `elbow_center`, `palm_up_left`, `palm_up_right`,
`elbow_swing`, `fade`, `return_home`.

Antes de mover nada comprueba los topes, la envolvente de autocolisión, la
velocidad de pico contra `max_ref_velocity` y el margen de par que deja la
gravedad. Si algo no cuadra, lo dice y no ejecuta.

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
