# h1_2_joint_control

Control y **sintonización de las ganancias** de las articulaciones del Unitree
H1-2, sin teleoperación de por medio. Sirve para contestar una pregunta que hay
que contestar **antes** de lanzar `xr_teleoperate`:

> ¿cada articulación sigue su referencia, y con qué calidad?

Corre sobre `unitree_ros2` (mensajes `unitree_hg`), que es lo que ya está
instalado y funcionando en este portátil. Las ganancias que salen de aquí valen
igual para `xr_teleoperate`: los dos caminos escriben el mismo `kp`/`kd` en el
mismo mensaje DDS.

---

## Lo primero: por qué el codo no se movía

**`/lowcmd` ya tiene dueño, y lo que manda es «par cero».** El controlador de
alto nivel (`ai`) publica ahí a 500 Hz sin parar, con `kp = kd = 0` en los 27
motores. Un script que publique en el mismo tópico no lo sustituye: se alterna
con él, y el motor recibe alternativamente nuestras ganancias y ganancias nulas.

* `arm_joint_test.py` publica a 250 Hz contra 500 → dos de cada tres mensajes
  anulan el kp → **el codo no llega a su referencia**.
* `test_mandar_modificado.py` publica a 500 Hz contra 500 → la mitad, suficiente
  para arrastrar la articulación pero conmutando kp/0 a centenares de hercios →
  **se mueve, pero vibra**.

No es un problema de ganancias. Y `arm_sdk`, que parecía la salida limpia,
**no hace nada con el robot en reposo**: probadas cinco variantes del mensaje,
incluida una copia literal del ejemplo oficial de Unitree, ninguna mueve la
articulación. Lo que funciona es soltar el controlador de alto nivel y quedarse
solo en `/lowcmd`:

```bash
python3 scripts/06_debug_mode.py status   # qué está mandando el servicio
python3 scripts/06_debug_mode.py enter    # ⚠ robot COLGADO DEL ARNÉS
...
python3 scripts/06_debug_mode.py exit
```

Medidas y detalle en [`docs/01_DIAGNOSTICO.md`](docs/01_DIAGNOSTICO.md).

## Y la respuesta a «¿sigue cada articulación su referencia?»

**Sí, las 14.** Con las ganancias de `xr_teleoperate`, escalón suave de 0.12 rad:
ninguna oscila, ninguna tiembla por encima del ruido de los motores libres, y
ninguna pasa del 25 % de su par. La tabla completa y el resto de resultados
—barridos de kp/kd del codo, compensación de gravedad, el coste de mandar
`dq = 0`— en [`docs/06_RESULTADOS.md`](docs/06_RESULTADOS.md).

---

## Puesta en marcha

No hace falta compilar nada ni crear entornos: usa el `unitree_ros2` que ya
está en `~/humanoid_ws/src/unitree_ros2`.

```bash
cd ~/Documents/h1_2_utec/h1_2_joint_control
source scripts/env.sh          # ROS 2 + CycloneDDS + PYTHONPATH
python3 scripts/99_selftest.py # comprueba el software, sin robot
python3 scripts/00_diagnose.py # solo lectura: no toca el robot
```

`env.sh` detecta sola la NIC con IP en `192.168.123.0/24`. Para forzarla:
`H12_NIC=enp12s0 source scripts/env.sh`.

Dependencias, todas ya presentes: ROS 2 Humble, `unitree_ros2` compilado
(`unitree_hg`, `unitree_api`), `numpy`, `pyyaml`, `matplotlib`.

---

## Los scripts

Todos aceptan `--help`, `--dry-run` y `--weight`.

| Script | Toca el robot | Para qué |
|---|:--:|---|
| `00_diagnose.py` | **no** | Quién manda en `/lowcmd`, qué controlador está activo, estado de los 27 motores, ganancias configuradas |
| `01_hold.py` | sí | Toma el control **sin comandar movimiento**. La prueba que hay que pasar antes de mover nada |
| `02_move.py` | sí | Mueve UNA articulación con escalón / seno / chirp, mide y guarda CSV |
| `03_tune.py` | sí | Barrido de kp/kd sobre una articulación, con criterio explícito. Escribe el ganador en `gains.yaml` |
| `04_sweep_arms.py` | sí | Recorre las 14 articulaciones de los brazos y saca la tabla de veredictos |
| `05_plot.py` | **no** | Gráficas desde los CSV: consigna vs. medida, error, par y espectro del temblor |
| `06_debug_mode.py` | sí | Entra y sale del modo debug. Es la única puerta a `/lowcmd`, y antes de dejarte entrar comprueba qué está mandando el servicio |
| `07_gravity_ff.py` | sí | Mide el par de gravedad de una articulación y comprueba que compensarlo con `tau_ff` elimina el error permanente |
| `99_selftest.py` | **no** | 32 comprobaciones sin robot: tabla contra el URDF, CRC contra la implementación literal, métricas contra señales de respuesta conocida, carga de `gains.yaml` |

### Recorrido típico

```bash
source scripts/env.sh

# 0. ¿está el terreno despejado?
python3 scripts/00_diagnose.py

# 1. prueba en seco: se publica en un tópico que nadie escucha
python3 scripts/02_move.py --joint L_wrist_yaw --traj step --amp 0.10 --dry-run

# 2. canal bueno, autoridad nula
python3 scripts/01_hold.py --seconds 10 --weight 0

# 3. autoridad real, sin comandar movimiento
python3 scripts/01_hold.py --seconds 10

# 4. primer movimiento de verdad, en la articulación menos comprometida
python3 scripts/02_move.py --joint L_wrist_yaw --traj step --amp 0.10
python3 scripts/05_plot.py --last

# 5. sintonizar
python3 scripts/03_tune.py --joint L_elbow --kp-list 50,80,110,140 --kd-list 2
python3 scripts/03_tune.py --joint L_elbow --kp-list 110 --kd-list 0.5,1,2,3,5 --write tuned

# 6. recorrer los dos brazos con lo sintonizado
python3 scripts/04_sweep_arms.py --gains tuned
```

---

## Ganancias

`config/gains.yaml` guarda varios conjuntos y `active` dice cuál se usa:

| conjunto | origen | codo | muñecas |
|---|---|---|---|
| `official_arm_sdk` | `h1_2_arm_sdk_dds_example.cpp` de Unitree | kp 50 / kd 1 | kp 50 / kd 1 |
| `xr_teleoperate` | `H1_2_ArmController` — **lo que usará la teleoperación** | kp 140 / kd 3 | kp 50 / kd 2 |
| `ros2_example` | `low_level_ctrl_hg.cpp`, kp 50 / kd 1 planos | kp 50 / kd 1 | kp 50 / kd 1 |
| `tuned` | lo medido en **este** robot | lo que escriba `03_tune.py` | — |

La diferencia entre los dos primeros no es menor: **`xr_teleoperate` casi
triplica el kp del codo**. Y el codo del H1-2 tiene solo 18 Nm, así que con
kp = 140 basta con 0.13 rad (7.4°) de error para saturarlo. Validarlo es una de
las razones de ser de este paquete.

---

## Estructura

```
h1_2_joint_control/
├── README.md                   este fichero
├── config/gains.yaml           conjuntos de ganancias + topes de seguridad
├── docs/
│   ├── 01_DIAGNOSTICO.md       por qué no se movía y por qué vibraba, con medidas
│   ├── 02_ARQUITECTURA.md      canales, protocolo arm_sdk, CRC, índices y topes
│   ├── 03_SEGURIDAD.md         qué vigila el cliente y qué hacer si algo va mal
│   ├── 04_METODOLOGIA.md       cómo se sintoniza y qué significa cada métrica
│   └── 05_BITACORA.md          registro de ensayos y decisiones
├── h1_2_joint_control/
│   ├── joints.py               los 27 motores: índices, topes, par, velocidad
│   ├── crc.py                  CRC de Unitree, tabulado (8× más rápido)
│   ├── config.py               carga de gains.yaml
│   ├── client.py               cliente de bajo nivel con vigilancia y rampas
│   ├── motion_switcher.py      CheckMode / ReleaseMode del robot
│   ├── trajectories.py         escalón, escalón suave, seno, chirp
│   ├── metrics.py              seguimiento, escalón e índice de temblor
│   └── recorder.py             CSV de series temporales + índice de ensayos
├── scripts/                    las herramientas de línea de comandos
└── logs/                       CSV, PNG e `index.csv` con todas las métricas
```

---

## Seguridad, en corto

El H1-2 pesa unos 70 kg. **Robot colgado del arnés o sujeto, nadie al alcance
de los brazos, mando a mano. Paro de emergencia: `L2 + B`.**

El cliente aborta solo si el par pasa del 70 % del límite del URDF durante más
de 0.3 s, si un motor pasa de 80 °C, o si `/lowstate` deja de llegar 0.5 s. Al
salir —también por Ctrl-C o por excepción— devuelve el brazo a la postura de
partida y baja el peso de `arm_sdk` a 0 en rampa.

Lo completo, en [`docs/03_SEGURIDAD.md`](docs/03_SEGURIDAD.md).
