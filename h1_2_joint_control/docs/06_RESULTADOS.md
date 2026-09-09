# Resultados de los ensayos

## Temporización del lazo (F0) — 2026-09-08

Cabecera obligatoria de todo lo que sigue: sin estos números, los kd no son
reproducibles en otra máquina. Portátil `mito`, 8 núcleos, gobernador de CPU en
**`powersave`**, sin privilegios de tiempo real (`ulimit -r = 0`).

Cuatro bloques de 120 s, publicando en el tópico inerte con el robot conectado
(carga real de `/lowstate` a 500 Hz):

| bloque | `dt` med | `dt` p99 | `dt` máx | p99/T | tarde | `/lowstate` |
|---|---:|---:|---:|---:|---:|---:|
| 250 Hz | 4.000 ms | 5.797 | 16.962 | **1.45** | 0.78 % | 499 Hz |
| 500 Hz | 1.999 ms | 2.901 | 10.709 | **1.45** | 0.74 % | 498 Hz |
| 250 Hz + registro | 4.000 ms | 5.175 | 13.859 | 1.29 | 0.35 % | 500 Hz |
| 250 Hz + `g(q)` | 4.000 ms | 6.454 | 23.799 | 1.61 | 1.05 % | 499 Hz |

**El criterio literal del protocolo (p99 ≤ 1.2·T, cero muestras perdidas) NO se
cumple.** Y sin embargo la conclusión no es que haya que arreglar la máquina.

### El criterio estaba midiendo lo que no era

Dos razones, las dos estructurales de cómo está escrito este paquete:

**La trayectoria se evalúa contra reloj, no contra contador de ciclos**
(`func(now - t0)` con `time.monotonic()`, `client.py:652`). Un ciclo que llega
tarde publica la consigna **correcta para ese instante**. El jitter añade ruido
de muestreo, no error de fase en el comando — que es el mecanismo por el que el
protocolo temía que se confundiera con falta de amortiguamiento.

**El lazo lee la última posición conocida en cada ciclo**, así que perderse
mensajes intermedios de `/lowstate` es inocuo. Lo que sesgaría las métricas es
que el mensaje más reciente fuera **viejo** al usarlo. Eso es la magnitud que
había que medir, y no se estaba midiendo.

### La métrica correcta: edad del estado al usarlo

60 s a 250 Hz, 30 176 mensajes recibidos (498 Hz):

| | edad | error aparente a 0.5 rad/s |
|---|---:|---:|
| mediana | 0.37 ms | 0.19 mrad |
| p95 | 1.09 ms | 0.55 mrad |
| p99 | 1.23 ms | 0.62 mrad |
| p99.9 | 2.01 ms | 1.01 mrad |
| **máxima** | **3.17 ms** | **1.59 mrad** |

Contra errores rms medidos de **5 a 32 mrad**. En el peor caso la antigüedad del
estado aporta 1.59 mrad: un 5 % del error del hombro y un 28 % del de la muñeca
más limpia. Es el término que hay que tener presente al comparar muñecas entre
sí, y despreciable para los hombros.

**Criterio adoptado en sustitución**: edad del estado p99 ≤ 2 ms y error
aparente por antigüedad ≤ 2 mrad. Se cumple.

### Sobre el p99 de `dt`, y por qué no se persigue

El mismo lazo, sin cambiar una línea, dio **p99/T = 1.11 en una corrida y 1.45
en otra**. La varianza entre corridas es del tamaño del efecto, así que p99/T no
es una propiedad del código sino del estado de la máquina en ese momento.

Se probaron cuatro estrategias de temporización (sueño completo, sueño + 1.0 ms
girando, + 0.3 ms, y sin recolector de basura). La mejor fue sueño + 1.0 ms
girando, con p99/T = 1.03 — **pero con una sola corrida por estrategia, y esa
diferencia no supera la varianza observada**. Adoptarla sería cometer el error
que la regla 5 del protocolo prohíbe. Queda como candidata a medir con N
adecuado, no como mejora aplicada.

Mitigaciones disponibles si alguna vez hiciera falta: gobernador a
`performance`, `SCHED_FIFO` (hoy imposible, `ulimit -r = 0`), C-states. Ninguna
se aplica porque el criterio que importa ya se cumple.

---

# Resultados de los ensayos — 2026-09-04

Robot colgado del arnés. Canal `/lowcmd` con el controlador de alto nivel
soltado (`06_debug_mode.py enter`). Lazo a 250 Hz. Todos los datos crudos en
`logs/index.csv` y en los CSV por ensayo.

---

> **Nota de la segunda sesión (17:30–18:10).** El barrido de la sección 1 se
> repitió con la **postura de ensayo** validada (hombros a ±18° reales, ver
> [`07_POSTURA.md`](07_POSTURA.md)). Los resultados están en la sección 1 bis y
> matizan a los de abajo: doce articulaciones siguen igual de bien, y las dos
> `shoulder_roll` pasan a tener un error de ~4°, **más grande que antes**, por
> una razón física clara. Se conservan las dos tablas porque la comparación
> entre ellas es justamente el resultado.

## 1. ¿Sigue cada articulación su referencia? Sí, las 14

Escalón suavizado de 0.12 rad (6.9°) en 0.3 s, una articulación cada vez, con
las ganancias de `xr_teleoperate` —las que usará la teleoperación—.

| articulación | kp | kd | err final | sobreimp. | subida | temblor | par máx |
|---|---:|---:|---:|---:|---:|---:|---|
| L_shoulder_pitch | 140 | 3.0 | −1.24° | 9.1 % | 181 ms | 0.0122 | 8.09 / 40 Nm |
| L_shoulder_roll | 140 | 3.0 | +0.96° | 2.9 % | 216 ms | 0.0217 | 9.05 / 40 Nm |
| L_shoulder_yaw | 140 | 3.0 | −0.41° | 0.0 % | 170 ms | 0.0200 | 1.79 / 18 Nm |
| L_elbow | 140 | 3.0 | −0.72° | 0.0 % | 187 ms | 0.0150 | 2.64 / 18 Nm |
| L_wrist_roll | 50 | 2.0 | +0.00° | 0.1 % | 184 ms | 0.0069 | 0.50 / 19 Nm |
| L_wrist_pitch | 50 | 2.0 | −0.59° | 0.0 % | 188 ms | 0.0057 | 1.06 / 19 Nm |
| L_wrist_yaw | 50 | 2.0 | +0.42° | 0.0 % | 208 ms | 0.0071 | 0.69 / 19 Nm |
| R_shoulder_pitch | 140 | 3.0 | −1.04° | 6.4 % | 180 ms | 0.0140 | 6.59 / 40 Nm |
| R_shoulder_roll | 140 | 3.0 | −1.43° | 5.9 % | 187 ms | 0.0186 | 9.23 / 40 Nm |
| R_shoulder_yaw | 140 | 3.0 | −0.38° | 0.0 % | 168 ms | 0.0116 | 1.01 / 18 Nm |
| R_elbow | 140 | 3.0 | −0.75° | 0.0 % | 157 ms | 0.0155 | 3.25 / 18 Nm |
| R_wrist_roll | 50 | 2.0 | +0.58° | 0.0 % | 231 ms | 0.0068 | 0.62 / 19 Nm |
| R_wrist_pitch | 50 | 2.0 | −0.51° | 0.0 % | 205 ms | 0.0093 | 0.69 / 19 Nm |
| R_wrist_yaw | 50 | 2.0 | +0.45° | 0.0 % | 202 ms | 0.0074 | 1.00 / 19 Nm |

**14 de 14 dentro de tolerancia.** Ninguna oscila, ninguna tiembla (el índice de
temblor se queda entre 0.006 y 0.022 rad/s, contra 0.005–0.016 medidos con los
motores libres: no añadimos vibración), y ninguna pasa del 25 % de su par.

Con esto, la teleoperación tiene una base sana. Las ganancias de
`xr_teleoperate` son utilizables tal cual.

### Lo que no se ve en la tabla

Los hombros son los únicos con sobreimpulso apreciable (2.9 %–9.1 %) y los
únicos que piden par de verdad. Es lo esperable: cargan con el brazo entero.

---

## 1 bis. El mismo barrido desde la postura de ensayo

Con los hombros llevados a ±18° reales antes de medir, mismo escalón suave de
0.12 rad:

| articulación | err final | sobreimp. | subida | temblor | par máx |
|---|---:|---:|---:|---:|---|
| L_shoulder_pitch | −0.95° | 10.3 % | 195 ms | 0.0146 | 6.77 / 40 Nm |
| **L_shoulder_roll** | **+4.17°** | 0.0 % | — | 0.0131 | 15.12 / 40 Nm |
| L_shoulder_yaw | +0.39° | 0.0 % | 163 ms | 0.0191 | 2.07 / 18 Nm |
| L_elbow | +0.03° | 3.0 % | 212 ms | 0.0146 | 1.58 / 18 Nm |
| L_wrist_roll | +0.05° | 0.7 % | 168 ms | 0.0058 | 0.50 / 19 Nm |
| L_wrist_pitch | −0.04° | 0.0 % | 169 ms | 0.0083 | 0.62 / 19 Nm |
| L_wrist_yaw | +0.01° | 0.1 % | 161 ms | 0.0090 | 0.31 / 19 Nm |
| R_shoulder_pitch | −0.14° | 3.7 % | 198 ms | 0.0152 | 6.50 / 40 Nm |
| **R_shoulder_roll** | **−4.15°** | 0.0 % | — | 0.0151 | 15.73 / 40 Nm |
| R_shoulder_yaw | −0.27° | 0.0 % | 169 ms | 0.0157 | 1.40 / 18 Nm |
| R_elbow | +0.13° | 4.0 % | 159 ms | 0.0122 | 1.76 / 18 Nm |
| R_wrist_roll | −0.05° | 0.0 % | 175 ms | 0.0117 | 0.69 / 19 Nm |
| R_wrist_pitch | +0.22° | 0.0 % | 199 ms | 0.0087 | 0.62 / 19 Nm |
| R_wrist_yaw | −0.01° | 0.3 % | 144 ms | 0.0105 | 0.56 / 19 Nm |

**Doce articulaciones clavan la referencia**: los codos y las seis muñecas se
quedan por debajo de **0.25°** de error final, mejor que en el barrido anterior.
El temblor sigue en el nivel del ruido de fondo en todas.

Las dos `shoulder_roll` empeoran, y no por casualidad. Esta vez **no hay
choque**: la postura las pone a ±18° y el ensayo las aleja aún más, hasta ±27°,
donde el brazo está más horizontal y **el par de gravedad es mayor**. De 9 Nm
antes a 15 Nm ahora. Confirmado midiendo:

| articulación | ángulo | `tau_g` | err sin `tau_ff` | `tau_g/kp` previsto | err con `tau_ff` | mejora |
|---|---:|---:|---:|---:|---:|---:|
| L_shoulder_roll | +27.3° | 9.63 Nm | +4.13° | **+3.94°** | +1.10° | 73 % |
| R_shoulder_roll | −27.0° | −10.36 Nm | −4.17° | **−4.24°** | −0.36° | 91 % |

El modelo acierta a la décima de grado. No es un fallo de sintonización: es lo
que un PD sin integral puede hacer, y **ningún kp razonable lo arregla** (haría
falta kp ≈ 600 para bajar de 1°, y con 40 Nm de motor eso satura con 0.07 rad
de error).

### Qué significa esto para la teleoperación

**`shoulder_roll` va a ir retrasado, y tanto más cuanto más levantado esté el
brazo**: ~1° con el brazo casi caído, ~4° con el brazo a 27°. Es la articulación
donde la compensación de gravedad más se nota, y `xr_teleoperate` hoy le pasa
`tauff = 0`.

Las otras doce no necesitan nada: con las ganancias que ya trae, siguen la
referencia con menos de un cuarto de grado.

---

## 2. El sentido del ensayo importa, y mucho

**Primera pasada del mismo barrido, moviendo siempre en `+amp`:** 12/14, con
`L_shoulder_roll` a +3.06° y `R_shoulder_roll` a **+6.25° y 15.03 Nm**.

No era un problema de ganancias: **los brazos del H1-2 son especulares**, así
que un `+0.12 rad` que separa el brazo izquierdo del cuerpo mete el derecho
**contra el torso**. El error no se cerraba porque el brazo estaba chocando.

Llevado al extremo con compensación de gravedad, `R_shoulder_roll` llegó a
**29.6 Nm** y saltó la protección de par (que hizo su trabajo y soltó el brazo).
En el sentido correcto, el mismo ensayo da 1.67° de error y 4.3 Nm.

Corregido con `pick_amplitude()` en `scripts/_common.py`: por defecto se mueve
hacia donde queda **más recorrido articular**, que en la práctica separa los dos
casos. Se puede forzar con `--direction positive|negative`.

> Esto vale también para la teleoperación: cualquier consigna que empuje un
> brazo contra el torso produce un error permanente grande y un par alto sin
> que nada esté «mal» en el controlador.

---

## 3. El error permanente es gravedad, y se puede quitar

El PD del motor no tiene término integral. Para sostener una postura contra la
gravedad necesita un error permanente `e = tau_gravedad / kp`, que no se va
nunca. Medido con `07_gravity_ff.py`:

| articulación | kp | par de gravedad | err sin `tau_ff` | err con `tau_ff` | `tau_g/kp` previsto | mejora |
|---|---:|---:|---:|---:|---:|---:|
| L_shoulder_roll | 140 | +4.22 Nm | +1.89° | +0.75° | +1.73° | 60 % |
| R_shoulder_roll | 140 | −4.34 Nm | −1.67° | **−0.13°** | −1.78° | **92 %** |

Dos cosas:

1. **El modelo cuadra.** El error previsto `tau_g/kp` (1.73° y 1.78°) coincide
   con el medido (1.89° y 1.67°). No hay misterio: es gravedad dividida por kp.
2. **La compensación funciona.** Metiendo el par medido como `tau_ff`, el error
   cae hasta un 92 %.

`xr_teleoperate` ya tiene el hueco preparado —`H1_2_ArmController.ctrl_dual_arm(q,
tauff)` acepta un par por articulación—, pero hoy la teleoperación le pasa
ceros. **Ahí hay una mejora real disponible**, y `pinocchio` ya está en su
entorno para calcular el par de gravedad a partir de la postura.

---

## 4. Barrido de ganancias del codo

### kp, con escalón suave de 0.15 rad y kd = 2

| kp | err rms | temblor | par máx | err final |
|---:|---:|---:|---:|---:|
| 50 | 88.0 mrad | 0.0070 | 4.48 Nm | +91.1 mrad |
| 80 | 56.4 mrad | 0.0132 | 4.31 Nm | +56.7 mrad |
| 110 | 41.1 mrad | 0.0120 | 4.57 Nm | +41.0 mrad |
| 140 | 33.5 mrad | 0.0152 | 5.10 Nm | +29.3 mrad |
| 180 | 31.4 mrad | 0.0095 | 5.27 Nm | — |
| 220 | 24.1 mrad | 0.0090 | 5.27 Nm | — |
| 260 | 25.1 mrad | 0.0091 | 5.71 Nm | +22.6 mrad |

El error baja como `1/kp` —otra vez la gravedad— y se aplana hacia kp ≈ 220. No
aparece ni sobreimpulso ni temblor en todo el rango: **con escalón suave, kp no
está limitado por la estabilidad sino por el par disponible**.

Con escalón DURO la historia cambia: kp = 140 con un salto de 0.15 rad pide
15.82 Nm de los 18 del codo, **12 % de margen**. kp = 50 se queda en 6.33 Nm
(65 % de margen) a cambio de 3.53° de error en vez de 1.57°.

### kd, con seguimiento sinusoidal de 0.15 rad a 0.5 Hz y kp = 140

| kd | err rms | temblor | par máx |
|---:|---:|---:|---:|
| 1 | 40.6 mrad | 0.1390 | 6.68 Nm |
| 2 | 31.4 mrad | 0.1069 | 6.86 Nm |
| 3 | 22.0 mrad | 0.0967 | 5.01 Nm |
| 4 | 23.3 mrad | 0.0858 | 2.64 Nm |
| 5 | 22.8 mrad | 0.0789 | 2.55 Nm |
| 6 | 21.2 mrad | 0.0670 | 3.16 Nm |
| **8** | **21.2 mrad** | **0.0554** | 2.90 Nm |
| 11 | 31.6 mrad | 0.0478 | 2.99 Nm |

Óptimo interior en **kd ≈ 6–8**: el error se estanca a partir de 6 y el temblor
sigue bajando hasta 8, pero en 11 el error se dispara. El kd = 3 de
`xr_teleoperate` funciona, pero deja un 75 % más de temblor que kd = 8 con el
mismo error.

**Cautela**: todo esto está medido en UNA postura (codo a 85°, brazo colgando).
La inercia y el par de gravedad cambian con la configuración del brazo. Antes de
cambiar el `gains.yaml` de producción conviene repetirlo en dos o tres posturas
representativas.

---

## 5. La velocidad de referencia: lo que cuesta mandar `dq = 0`

`xr_teleoperate` fija `msg.motor_cmd[id].dq = 0` siempre. Medido en el codo,
seno de 0.15 rad a 0.5 Hz, kp = 140, kd = 6, dos pasadas seguidas:

| | err rms |
|---|---:|
| con velocidad de referencia | 35.8 mrad |
| con `dq = 0` (como `xr_teleoperate`) | 48.5 mrad |

**35 % más de error** por no mandar la derivada de la consigna. La razón está
en la fórmula: el término `kd·(0 − dq)` frena activamente el movimiento que se
está pidiendo, y hace falta error de posición extra para vencerlo.

Es la segunda mejora disponible en la teleoperación, y más barata que la
primera: la derivada de la consigna ya se conoce en el lado del operador.

---

## 6. Dispersión entre pasadas

El mismo ensayo (codo, seno 0.5 Hz, kp = 140, kd = 6) dio 21.2 mrad dentro de un
barrido y 35.8 mrad ejecutado por separado. La diferencia es grande y hay que
tenerla en cuenta antes de sacar conclusiones de diferencias pequeñas.

La explicación más probable es **fricción estática**: dentro del barrido la
articulación llevaba minutos moviéndose, y arrancada en frío se pega más.
Conviene mover la articulación un par de veces antes de medir, y usar
`--repeats` para promediar.

---

---

## 7. Sintonización completa de hombros, codos y muñecas (2026-09-05)

Método: kp primero (kd en su referencia), luego kd con el kp ganador. Medida
por seguimiento sinusoidal de 0.12 rad a 0.5 Hz, tres ciclos, con la postura de
ensayo aplicada y `dq_des` **activa**. Canal `lowcmd`, robot colgado.

### Las 14, y la mejora sobre la referencia

| articulación | referencia | sintonizado | err rms | qué mandó la decisión |
|---|---|---|---:|---|
| L_shoulder_pitch | 140 / 3.0 | 280 / 13.5 | 10.08 mrad | kp en el tope de saturación (40 Nm) |
| L_shoulder_roll | 140 / 3.0 | 280 / 13.5 | 31.91 mrad | ídem; es la peor con diferencia |
| L_shoulder_yaw | 140 / 3.0 | 126 / 13.5 | 10.48 mrad | kp acotado: 18 Nm, no 40 |
| L_elbow | 140 / 3.0 | 126 / 13.5 | 9.49 mrad | ídem |
| L_wrist_roll | 50 / 2.0 | 100 / 4.0 | 5.62 mrad | kd con óptimo **interior** |
| L_wrist_pitch | 50 / 2.0 | 100 / 4.0 | 7.39 mrad | ídem, mismo valor |
| L_wrist_yaw | 50 / 2.0 | 100 / 9.0 | 9.55 mrad | |
| R_shoulder_pitch | 140 / 3.0 | 280 / 13.5 | 9.45 mrad | 16.86 → 9.45, **44 %** mejor |
| R_shoulder_roll | 140 / 3.0 | 280 / 9.0 | 30.90 mrad | 56.71 → 30.90, **46 %** |
| R_shoulder_yaw | 140 / 3.0 | 126 / 13.5 | 8.65 mrad | kp acotado |
| R_elbow | 140 / 3.0 | 126 / 13.5 | 13.76 mrad | ídem |
| R_wrist_roll | 50 / 2.0 | 100 / 9.0 | 6.74 mrad | 9.73 → 6.74, **31 %** |
| R_wrist_pitch | 50 / 2.0 | 100 / 6.0 | 7.47 mrad | 12.25 → 7.47, **39 %** |
| R_wrist_yaw | 50 / 2.0 | 100 / 9.0 | 9.56 mrad | 16.59 → 9.56, **42 %** |

Mejora del **31 al 46 %** en error de seguimiento, sin un solo aborto en 126
ensayos.

Que `L_wrist_roll` y `L_wrist_pitch` —articulaciones independientes— den
exactamente el mismo par de ganancias es buena señal de que el método no ajusta
ruido. Los kd de las muñecas del brazo derecho salen algo más altos (9, 6, 9
contra 4, 4, 9): es dispersión entre pasadas, del orden de la ya vista en la
sección 6.

### Barrido de comprobación con las ganancias nuevas

Escalón suave de 0.12 rad, las 14 de una en una, postura de ensayo aplicada:

| articulación | kp | kd | err final | sobreimp | subida | temblor | par máx |
|---|---:|---:|---:|---:|---:|---:|---|
| L_shoulder_pitch | 280 | 13.5 | −0.25° | 4.5 % | 204 ms | 0.0102 | 8.44 / 40 Nm |
| L_shoulder_roll | 280 | 13.5 | +0.95° | 20.3 % | 132 ms | 0.0100 | 13.01 / 40 Nm |
| L_shoulder_yaw | 126 | 13.5 | +0.52° | 0.1 % | 188 ms | 0.0102 | 2.40 / 18 Nm |
| L_elbow | 126 | 13.5 | −0.18° | 0.0 % | 204 ms | 0.0108 | 3.52 / 18 Nm |
| L_wrist_roll | 100 | 4.0 | +0.06° | 0.0 % | 176 ms | 0.0049 | 0.44 / 19 Nm |
| L_wrist_pitch | 100 | 4.0 | −0.16° | 0.0 % | 177 ms | 0.0068 | 0.69 / 19 Nm |
| L_wrist_yaw | 100 | 9.0 | +0.05° | 0.0 % | 177 ms | 0.0048 | 0.31 / 19 Nm |
| R_shoulder_pitch | 280 | 13.5 | −0.14° | 3.2 % | 192 ms | 0.0089 | 6.68 / 40 Nm |
| R_shoulder_roll | 280 | 9.0 | −1.09° | 21.0 % | 128 ms | 0.0117 | 14.06 / 40 Nm |
| R_shoulder_yaw | 126 | 13.5 | +0.29° | 0.1 % | 180 ms | 0.0139 | 1.40 / 18 Nm |
| R_elbow | 126 | 13.5 | −0.79° | 0.4 % | — | 0.0086 | 4.04 / 18 Nm |
| R_wrist_roll | 100 | 9.0 | +0.20° | 0.1 % | 184 ms | 0.0082 | 0.56 / 19 Nm |
| R_wrist_pitch | 100 | 6.0 | +0.19° | 0.0 % | 184 ms | 0.0078 | 0.69 / 19 Nm |
| R_wrist_yaw | 100 | 9.0 | −0.02° | 0.1 % | 176 ms | 0.0050 | 0.94 / 19 Nm |

**14 de 14 dentro de tolerancia.** Frente al barrido con las ganancias de
`xr_teleoperate` (sección 1 bis), el error permanente de los `shoulder_roll`
baja de ±4.2° a **±1.0°**: cuatro veces mejor en la articulación que peor
estaba. El resto se mantiene por debajo de 0.8°.

El precio es sobreimpulso en los dos `shoulder_roll`, 20 %, que antes no había.
Es real y es la contrapartida de kp = 280: suben en 132 ms en vez de 200 y se
pasan 1.4° antes de asentarse. Con umbral de 25 % pasa, pero si en
teleoperación molesta, la salida es bajar kd… no: es bajar kp y aceptar más
error permanente, o —mejor— compensar gravedad y volver a kp moderado.

### Un fallo de la métrica de sobreimpulso, encontrado aquí

La primera lectura de ese barrido dio **42 % y 46 %** de sobreimpulso en los
`shoulder_roll`. Al intentar corregirlo bajando kp, salió lo contrario de lo
que dice la teoría:

| kp | «sobreimpulso» |
|---:|---:|
| 280 | 42.1 % |
| 220 | 48.0 % |
| 180 | 56.9 % |
| 140 | 72.3 % |

Bajar kp con el mismo kd **aumenta** el amortiguamiento relativo, así que el
sobreimpulso tenía que bajar. Que subiera delataba un fallo de medida, no del
robot.

La causa: el sobreimpulso se medía contra la CONSIGNA, y con gravedad la
articulación no parte de la consigna anterior sino de ella menos la caída
`tau_g/kp`. Ese desfase entraba en el denominador. Y como la caída **crece** al
bajar kp, el artefacto crecía al bajar kp — de ahí la tendencia invertida.

Corregido midiendo contra el **valor final**, que es la pregunta correcta: ¿se
pasa de donde acaba reposando, y cuánto? Recalculado sobre los mismos CSV, sin
volver a tocar el robot:

| | antes | corregido |
|---|---:|---:|
| L_shoulder_roll | 42.0 % | **20.3 %** |
| R_shoulder_roll | 45.9 % | **21.0 %** |
| L_shoulder_pitch | 0.7 % | 4.5 % |

Casi la mitad era artefacto; el 20 % que queda es real.

### Por qué kp acaba pegado a su techo, y qué significa

En hombros y codo el error lo domina la gravedad, `tau_g/kp`, así que baja
monótonamente con kp: el barrido sube hasta donde la saturación se lo permite.
**No es «cuanto más kp, mejor»**; es que en esas articulaciones el problema no
es la ganancia, es que falta compensación de gravedad. Ver la sección 3.

El tope viene de exigir que un salto de consigna de 0.10 rad no pida más par que
el umbral de aborto: `kp_max = 0.7 · tau_max / 0.10`. Son 280 en los hombros de
40 Nm y 126 en el codo y el hombro-yaw de 18 Nm.

Ese mismo criterio marca como excesivas **cuatro ganancias del propio
`xr_teleoperate`**: `L/R_shoulder_yaw` y `L/R_elbow`, todas a kp = 140 sobre
motores de 18 Nm, cuyo techo es 126. Un salto de consigna de 7.4° las satura.
`99_selftest.py` lo comprueba ahora automáticamente.

### kd: lo que el barrido premia y lo que hay que mirar antes de creérselo

En las muñecas kd tiene un mínimo claro. Ejemplo real, `L_wrist_roll` a kp = 100:

| kd | err rms | temblor | par máx |
|---:|---:|---:|---:|
| 1.0 | 5.14 mrad | 0.0563 | 0.50 Nm |
| 2.0 | 5.24 | 0.0334 | 0.44 |
| **4.0** | **5.62** | **0.0206** | **0.44** |
| 6.0 | 6.08 | 0.0165 | 1.00 |
| 9.0 | 6.50 | 0.0143 | 1.94 |

Subiendo kd el temblor baja 4× y el error empeora un 26 %, pero lo que decide es
la última columna: **el par de pico se cuadruplica** de kd = 4 a kd = 9. Eso es
el término de amortiguación frenando el movimiento que se está pidiendo.

En hombros y codo esa penalización **no aparece a 0.5 Hz** —tienen 40 Nm de
margen y la misma velocidad—, y por eso allí el barrido empuja kd al máximo del
rango (13.5). No significa que 13.5 sea buena idea: significa que a esa
frecuencia no se paga.

### El aviso que acompaña a todos estos kd

Están medidos **con `dq_des` activa**. `xr_teleoperate` manda `dq_des = 0`, y
entonces el término `kd·(0 − dq)` frena con un par que solo depende de la
velocidad:

| | `tau_max` | kd | 1 rad/s | 2 rad/s | 3 rad/s |
|---|---:|---:|---:|---:|---:|
| hombro | 40 Nm | 13.5 | 13.5 Nm | 27.0 Nm | **satura** |
| codo | 18 Nm | 13.5 | 13.5 Nm | **satura** | **satura** |
| codo | 18 Nm | 3.0 | 3.0 Nm | 6.0 Nm | 9.0 Nm |

Con kd = 13.5 y sin velocidad de referencia, **el codo se satura solo con
frenar, a 2 rad/s**, antes de que kp haya pedido nada.

De ahí que los valores de esta sección vengan con una condición: **son válidos
si la teleoperación manda la derivada de la consigna**. Si se queda con
`dq_des = 0`, kd tiene que quedarse cerca de la referencia.

---

---

## 8. Gravedad y fricción separadas (F2.1) — 2026-09-08

19 puntos del brazo izquierdo, cada uno alcanzado **desde los dos sentidos**
para separar `τ_g` de la fricción estática. Canal `lowcmd`, ganancias `tuned`,
configuración completa registrada en cada medida (`logs/gravity_id_*.json`).

### La fricción, medida por primera vez

| articulación | \|f\| medio | \|f\| máx | como fracción de `τ_max` |
|---|---:|---:|---:|
| L_shoulder_pitch | 0.14 Nm | 0.29 | 0.7 % |
| L_shoulder_roll | 0.83 Nm | 1.46 | 3.7 % |
| **L_shoulder_yaw** | **0.95 Nm** | 1.36 | **5.3 %** |
| **L_elbow** | **0.81 Nm** | 0.86 | **4.5 %** |
| L_wrist_roll | 0.16 Nm | 0.27 | 1.4 % |
| L_wrist_pitch | 0.28 Nm | 0.28 | 1.5 % |
| L_wrist_yaw | 0.23 Nm | 0.30 | 1.6 % |

Las dos peores son `shoulder_yaw` y `elbow`, los motores de 18 Nm: ahí la
fricción vale un 5 % del par disponible. Eso explica dos cosas que estaban
sueltas: la dispersión entre pasadas de §6 (21.2 contra 35.8 mrad en el mismo
ensayo) y la banda de pegado del codo.

### Mi hipótesis era falsa

En [`08_PLAN.md`](08_PLAN.md) §1.3 escribí que «una parte apreciable de los
3.74 Nm de discrepancia es fricción, no masa», y lo dejé escrito antes de medir
precisamente para poder contrastarlo. **No se sostiene.** Separando la fricción,
el error del modelo contra la gravedad limpia sigue siendo de **0.96 Nm de media
y 4.63 Nm máximo**, contra un criterio de 1.0 Nm.

La fricción es real pero pequeña; no explica el hueco.

### Y la masa de la mano tampoco lo explica

Ajuste de un solo parámetro sobre la gravedad limpia:

| factor sobre la masa del URDF | masa de mano | error medio | error máx |
|---:|---:|---:|---:|
| ×1.0 | 316 g | 0.96 Nm | 4.63 |
| ×2.5 | 789 g (≈ la real) | 0.72 Nm | 3.16 |
| ×4.0 | 1263 g | 0.50 Nm | 1.85 |
| ×6.0 | **1895 g** | 0.41 Nm | 1.43 |

El óptimo pide **1.9 kg de mano**, más del doble de la real. Un parámetro que
para ajustar los datos tiene que tomar un valor físicamente imposible está
absorbiendo otra cosa.

Y el residuo con ese mejor factor tiene **estructura**, no ruido:

| articulación | residuos | medio |
|---|---|---:|
| L_shoulder_pitch | −1.13, −1.43, −0.96 | **1.17 Nm** |
| L_shoulder_yaw | −0.45, −0.96, −0.30 | 0.57 |
| L_shoulder_roll | −0.66, +0.27 | 0.46 |
| L_wrist_yaw | +0.38, +0.30, +0.37 | 0.35 |
| L_elbow | +0.13, −0.08, +0.00 | **0.07** |
| L_wrist_roll / pitch | ≈ 0 | 0.08 |

Los residuos de `shoulder_pitch` y `wrist_yaw` son **casi constantes con el
ángulo**. Un desplazamiento constante en un par de gravedad no es un error de
masa: es una masa que el modelo no tiene, o un sesgo del propio `tau_est`.

### Consecuencia para el protocolo: F2.1 no puede cerrarse como está escrito

El protocolo plantea F2.1 como «verificar la masa de la mano, corregirla, y
seguir», con una rama de contingencia para desmontar una mano. Los datos dicen
que **ninguna de las dos ramas aplica**: no es la mano, y el residuo tiene
estructura en más de una articulación a la vez.

Dos caminos, y el segundo es el que recomiendo:

**(a) Identificación inercial completa.** Ajustar los parámetros base del brazo,
no un escalar. Es el procedimiento estándar, pero necesita una trayectoria de
excitación diseñada para que el problema esté bien condicionado. Los 19 puntos
de aquí están agrupados alrededor de una postura y no lo están.

**(b) Regresión empírica de `τ_g(q)`, sin modelo físico.** Para lo que hace
falta —un `tau_ff` que funcione— no se necesita un modelo correcto, se necesita
una buena predicción. Y acabamos de demostrar que `τ_g` se puede medir limpia.
Muestrear el espacio de trabajo y ajustar una regresión da el `tau_ff` sin
pelearse con parámetros inerciales que el URDF tiene mal.

La (b) es más barata, no depende de que el URDF sea correcto, y da exactamente
el número que la teleoperación necesita. La (a) solo compensa si además se
quiere el modelo para otra cosa.

---

## 9. Identificación de los parámetros de masa (F2, camino b) — 2026-09-08

Tras descartar que el hueco fuera fricción (§8) o masa de la mano, se cambió de
enfoque: en vez de corregir el URDF a mano, **identificar los parámetros de masa
a partir de medidas**.

### El método, y por qué no es una regresión ciega

El par de gravedad es **lineal en los parámetros de masa**:

```
tau_g(q) = R(q) · pi        pi = [m, m·cx, m·cy, m·cz] por cuerpo
```

`R(q)` sale de `pinocchio` con velocidad y aceleración nulas y depende solo de
la **cinemática** del URDF — longitudes y ejes de los eslabones, que es lo que
un URDF suele tener bien. Lo que estaba mal eran las masas, y es exactamente lo
que se ajusta. Verificado que el regresor es exacto: `R·pi` reproduce
`computeGeneralizedGravity` con error de 3.5·10⁻¹⁵.

Frente a una regresión genérica de `tau_g(q)`, esto extrapola a posturas no
medidas, necesita muchos menos puntos, y de regalo dice cuánto pesa cada
eslabón.

**Regularización de cresta hacia el URDF.** De las 76 direcciones del espacio de
parámetros, muchas no son observables con estos datos — el centro de masa de una
falange no afecta a nada medible. Sin regularizar, el ajuste les asignaría
valores arbitrarios que encajan el ruido. Con cresta hacia el prior del URDF, lo
que los datos no ven se queda donde estaba.

### Los datos

`12_gravity_map.py`: **30 configuraciones sorteadas** por el espacio de trabajo
respetando topes y autocolisión, cada una alcanzada desde los dos sentidos, y en
cada una se leen **las siete articulaciones a la vez** — el par que sostiene cada
una está ahí sin coste extra. **210 medidas** con pares de hasta 19.45 Nm, contra
los 19 puntos agrupados de §8.

### El resultado

| | URDF de partida | identificado |
|---|---:|---:|
| rms sobre las 210 medidas | 2.574 Nm | **0.337 Nm** |
| error máximo | 8.167 Nm | **1.126 Nm** |
| masa del brazo | 6.399 kg | 6.866 kg |
| masa de la mano | 0.316 kg | **1.138 kg** |

**Validación cruzada**: rms de entrenamiento 0.330 Nm, de prueba 0.354 Nm. La
diferencia es despreciable, así que con 76 parámetros y 210 ecuaciones **no hay
sobreajuste** — la cresta hizo su trabajo. Todas las masas salen positivas.

Por articulación:

| articulación | URDF | identificado | mejora |
|---|---:|---:|---:|
| L_shoulder_pitch | 4.508 Nm | 0.439 | **90 %** |
| L_shoulder_roll | 4.189 | 0.508 | 88 % |
| L_elbow | 2.346 | 0.309 | 87 % |
| L_wrist_pitch | 0.735 | 0.112 | 85 % |
| L_wrist_yaw | 0.556 | 0.122 | 78 % |
| L_wrist_roll | 0.299 | 0.087 | 71 % |
| L_shoulder_yaw | 1.442 | 0.463 | 68 % |

`shoulder_yaw` es la que menos mejora, y tiene sentido: es la de mayor fricción
(0.93 Nm de media, §8), así que buena parte de su residuo no es gravedad.

### La masa de la mano, ahora sí plausible

El ajuste da **1.138 kg** donde el URDF pone 316 g. La mano Inspire sola pesa
~800 g, y F4.1 del protocolo pide pesar «el conjunto mano + conector + tramo de
cable que carga la muñeca». 1.14 kg encaja con eso.

Contrástese con el ajuste de un solo parámetro de §8, que pedía **1.9 kg** —
físicamente imposible. La diferencia es que aquí también se ajustan los
eslabones del brazo (+0.47 kg repartidos), así que la mano no tiene que absorber
todo el error.

### Los dos brazos, identificados por separado

Repetido el mapeo con el brazo derecho (30 configuraciones, semilla distinta,
otro conjunto de cuerpos del modelo):

| | izquierdo | derecho |
|---|---:|---:|
| rms URDF → identificado | 2.574 → **0.337** Nm | 2.252 → **0.360** Nm |
| error máximo | 8.167 → 1.126 Nm | 7.494 → 1.438 Nm |
| **masa de mano identificada** | **1.138 kg** | **1.074 kg** |
| masa de brazo | 6.866 kg | 7.287 kg |

Los dos ajustes son **independientes** —datos distintos, cuerpos distintos del
modelo— y coinciden en la masa de la mano al **6 %**. Es la mejor validación
que hay del método: dos identificaciones separadas convergen al mismo valor
físico, y ese valor es compatible con una mano Inspire de ~800 g más conector y
cable.

### Validación en movimiento: escalón con y sin compensación

Escalón suave de 0.12 rad, ganancias `tuned`, canal `lowcmd`:

| articulación | | err final | sobreimp. | establec. | pico τ |
|---|---|---:|---:|---:|---:|
| **L_shoulder_roll** | sin compensar | +17.85 mrad | 19.4 % | 506 ms | 13.36 Nm |
| | con `τ_ff` | **+2.72 mrad** | 18.3 % | 495 ms | 13.80 Nm |
| | `τ_ff` + kp a la mitad | +13.70 mrad | **5.6 %** | 482 ms | 10.46 Nm |
| **L_shoulder_pitch** | sin compensar | −14.19 mrad | 8.3 % | 792 ms | 10.11 Nm |
| | con `τ_ff` | **+0.32 mrad** | 8.0 % | 533 ms | 9.93 Nm |
| | `τ_ff` + kp a la mitad | −1.01 mrad | **2.3 %** | **464 ms** | **8.79 Nm** |

Con compensación el error permanente cae **un 85 % en el roll y un 98 % en el
pitch**. Y la última fila de `shoulder_pitch` es el resultado que buscaba el
protocolo en F2.3: **con la gravedad compensada y la mitad de kp**, el error es
14 veces menor que sin compensar a kp completo, el sobreimpulso baja del 8.3 %
al 2.3 %, asienta más rápido y pide menos par. Todo a la vez.

En `shoulder_roll` la rebaja de kp sale peor (13.70 contra 2.72 mrad) porque el
residuo del modelo en esa articulación es mayor: al dividir kp por dos, el error
que ese residuo produce se duplica. Aun así el sobreimpulso baja del 19.4 % al
5.6 %.

**Prueba de humo del protocolo**: `01_hold` con `τ_ff` y kp a la mitad — el
brazo se sostiene, deriva de 1.63 mrad, cero ciclos recortados por el tope del
50 % del par. El signo del modelo es correcto.

### Límites de este resultado
- **Sin objeto en la mano.** Es la condición L0 de F4.
- **kp sigue sin re-sintonizarse.** La tabla de arriba prueba que con `τ_ff` se
  puede bajar kp, pero el barrido completo con la gravedad activa (F2.3) está
  pendiente. Los `tuned` actuales se eligieron sin compensación.
- El λ elegido (0.003) es el menor probado, pero la curva de error de prueba es
  plana entre 0.03 y 0.003 (0.359 contra 0.354 Nm), así que no es un óptimo en
  el borde: es una meseta.

---

## 10. Re-sintonización con la gravedad compensada (F2.3) — 2026-09-08

### Un fallo del propio barrido, encontrado por el camino

La primera pasada de F2.3 dio sobreimpulsos de **52 a 84 %** en
`L_shoulder_roll`, contra el 19 % medido sin compensar. Como no tenía sentido
—compensar gravedad no puede empeorar el sobreimpulso tanto—, se repitieron los
mismos kp en corridas **aisladas**:

| kp | en el barrido | aislado |
|---:|---:|---:|
| 70 | 51.8 % | **0.8 %** |
| 111 | 83.6 % | **1.1 %** |
| 176 | 69.2 % | 13.5 % |
| 280 | 67.1 % | 17.3 % |

Los números del barrido eran falsos. La causa es la **regla 1 del protocolo**,
que `09_tune_all.py` no estaba respetando: *nunca cambiar kp con `q_des`
desactualizado*. Si la articulación sostiene con un error `e`, pasar de `kp` a
`kp'` produce un salto de par `(kp'−kp)·e` en un solo ciclo, y el ensayo
siguiente empezaba con la articulación aún asentándose de ese golpe.

Corregido igualando la consigna a la posición medida antes de tocar las
ganancias. Verificado: el barrido reproduce ahora los valores aislados (1.4 %
contra 0.8 %, 13.3 % contra 13.5 %, 17.0 % contra 17.3 %).

> **Alcance del fallo.** Afectaba a `09_tune_all.py` desde que se escribió, y
> por tanto a la campaña del 09-05. Aquellos barridos usaban seguimiento
> sinusoidal con `skip=0.5`, y entre el cambio de ganancias y el inicio de la
> medida hay más de 1 s (rampa + pausa + descarte), así que **probablemente** no
> están contaminados. No se ha verificado, y hasta que se haga la afirmación es
> «probablemente», no «seguro».

### El resultado, con el barrido arreglado

`L_shoulder_roll`, escalón suave de 0.12 rad, kd = 13.5, gravedad compensada:

| kp | err rms | err final | sobreimp. | pico τ |
|---:|---:|---:|---:|---:|
| **70** | 11.99 mrad | **+7.01 mrad (0.40°)** | **1.4 %** | **8.79 Nm** |
| 111 | 14.91 | +13.96 (0.80°) | 0.4 % | 8.88 |
| 176 | 14.09 | +4.65 (0.27°) | 13.3 % | 10.37 |
| 280 | 8.52 | +2.77 (0.16°) | 17.0 % | 13.45 |

Para comparar, **sin** compensar a kp = 280: err final +16.94 mrad (0.97°),
sobreimpulso 19.9 %, pico 13.10 Nm.

**Criterios de aceptación de F2.3:**

| criterio | resultado |
|---|---|
| Error permanente < 0.5° con kp ≤ 140 | ✔ **0.40° con kp = 70** |
| Sobreimpulso < 10 % (hoy 20 % con kp = 280) | ✔ **1.4 % con kp = 70** |
| Ganador de kp no en el borde en 2 de 3 | ✗ sigue en el borde, pero ahora en el **inferior** |

Los dos primeros se cumplen, y con **kp = 70: una cuarta parte del kp actual**.
El tercero falla en la letra, pero el ganador se ha movido del borde superior al
inferior, que es lo que predecía la hipótesis: kp estaba alto **para tapar la
falta de compensación**.

Con la gravedad compensada, `shoulder_roll` a kp = 70 da un sobreimpulso 14
veces menor, la mitad de error permanente y 4.7 Nm menos de pico que a kp = 280
sin compensar.

### Codo y muñeca: ya no hay nada que sintonizar

Con gravedad compensada, `L_elbow` da **0.0–0.3 % de sobreimpulso y 0.35–1.4
mrad de error final en todo el rango de kp de 31.5 a 126**, y `L_wrist_pitch`
otro tanto. El error rms mejora solo un 30 % al cuadruplicar kp. Para esas
articulaciones la elección de kp ha dejado de importar: cualquier valor
razonable va bien.

### Dónde evaluar `g()`: no importa

El protocolo pide `τ_ff = g(q_des)`. Se sospechó que evaluar en la consigna
adelanta durante el movimiento y añade empuje de más. Medido en
`L_shoulder_roll` a kp = 280: sobreimpulso 17.9 % con `q_des` y 17.3 % con la
posición medida, error final +2.48 contra +2.70 mrad. **No hay diferencia
apreciable.** Se deja `meas` por defecto, por principio —cancelar la gravedad
que actúa, no la que actuará—, pero la opción existe y ninguna de las dos es
mejor con estos datos.

---

## 11. El barrido no es reproducible — F1 deja de ser opcional

Al extender F2.3 a las siete articulaciones apareció algo que invalida el
procedimiento, no un resultado concreto.

`L_shoulder_roll`, escalón de 0.12 rad, kd = 13.5, gravedad compensada, **las
mismas ganancias en las tres corridas**:

| corrida | kp = 70 | kp = 111 | kp = 176 | kp = 280 |
|---|---:|---:|---:|---:|
| aislada, `02_move.py` | 0.8 % | 1.1 % | 13.5 % | 17.3 % |
| `09_tune_all.py`, una articulación | 3.0 % | 0.3 % | 13.4 % | 17.6 % |
| `09_tune_all.py`, las siete | **29.5 %** | **47.9 %** | **68.2 %** | **63.3 %** |

Las dos primeras coinciden. La tercera no se parece en nada.

El error permanente cuenta la misma historia: a kp = 70 da +8.00 mrad en la
corrida enfocada y **+31.03 mrad** en la completa. Como el error permanente es
`residuo/kp`, eso significa que el residuo del modelo de gravedad es **cuatro
veces mayor** en la corrida completa. La hipótesis natural es que la
compensación funciona peor en las configuraciones por las que pasa el brazo
cuando se barren siete articulaciones seguidas, pero **no está aislada**.

### Dos artefactos ya corregidos, y aun así

Por el camino se encontraron y arreglaron dos fallos reales de medida:

1. **Cambio de ganancias con `q_des` desactualizado** (regla 1 del protocolo).
   Pasar de kp a kp' con un error `e` mete un salto de par `(kp'−kp)·e`.
   Corregido igualando la consigna a la posición medida antes de tocar nada.
2. **Repeticiones sin asentar.** La segunda repetición empezaba mientras la
   articulación volvía de la primera, y `step_response` tomaba como punto de
   partida una posición en movimiento. Corregido con `wait_settled()`, que
   espera a velocidad baja de verdad en vez de a una pausa fija.

Los dos eran reales y los dos están verificados. **Y aun así queda esta
dispersión**, así que hay un tercer factor sin identificar.

### La conclusión que toca

Esto es exactamente la hipótesis **H1** del protocolo: «N = 1 en casi todo… la
dispersión es del orden de las mejoras que se reportan». Y **F1 — la fase que
cuantifica `δ_min` y que el protocolo declara bloqueante — es la que se saltó**.

Con una dispersión de 0.8 % a 29.5 % en el sobreimpulso, **ningún barrido puede
elegir entre candidatos**. Las mejoras del 0 al 20 % que reporta el resumen del
brazo izquierdo están por debajo del ruido y no significan nada.

Por eso el conjunto `tuned_gff` queda marcado como **provisional** en
`gains.yaml`, y no se ha tocado `tuned`. Lo que sí se sostiene de F2 son los
resultados medidos con corridas aisladas y repetidas:

- la identificación de masas (§9), validada por dos brazos independientes;
- que `τ_ff` reduce el error permanente un 85–98 % (§9);
- que con gravedad compensada `shoulder_roll` a kp = 70 da ~1–3 % de
  sobreimpulso contra ~17 % a kp = 280 (medido tres veces en corridas aisladas
  y enfocadas, que sí concuerdan entre sí).

**Antes de seguir sintonizando hay que cerrar F1.** No es una recomendación de
método: sin `δ_min` no se puede afirmar que un candidato sea mejor que otro.

---

## 12. F1: la dispersión no era ruido, era un fallo. `δ_min` medido

La §11 concluía que el barrido «no es reproducible» y que hacía falta F1 para
acotar el ruido. **La conclusión era equivocada, y F1 lo demostró.**

### La medida es muy repetible

Ensayo canónico repetido, `L_shoulder_roll`, escalón de 0.12 rad, kp = 280,
kd = 13.5, gravedad compensada:

| bloque | n | err rms mediana | IQR | sobreimp. mediana | IQR | err final mediana | IQR |
|---|--:|--:|--:|--:|--:|--:|--:|
| base | 6 | 9.69 mrad | 0.20 | 16.4 % | 0.5 | 2.57 mrad | 0.02 |
| vecinas | 6 | 10.08 | 0.16 | 16.7 % | 0.2 | 2.79 | 0.12 |

**`δ_min` = 1.5 × IQR del bloque base:**

| magnitud | `δ_min` |
|---|---:|
| error rms | **0.30 mrad** |
| sobreimpulso | **0.7 puntos porcentuales** |
| error permanente | **0.03 mrad** |

Por debajo de eso, ninguna diferencia entre candidatos es defendible. Y son
valores **pequeños**: el instrumento es bueno.

El bloque «vecinas» —mover las otras seis articulaciones entre repetición y
repetición— desplaza el sobreimpulso 0.4 pp, dentro del ruido. **No hay efecto
de contexto.**

### El fallo que producía la dispersión

`wrist_motion` se deducía de la lista de articulaciones **controladas**, no de
las que se mueven. En un barrido las siete están controladas pero solo una se
mueve, así que el margen extra por muñeca se aplicaba **siempre** y el tope de
`shoulder_roll` subía de 10° a 15°.

Un ensayo que va de 18° a 11.1° choca con ese tope. El portero de autocolisión
lo recortaba —**45.6 % de los ciclos** en la corrida que lo destapó— y el
escalón se quedaba a medias: `q_final` cerca de `q_start`, y un «sobreimpulso»
del 61 % que no existía.

Corregido usando las articulaciones con trayectoria activa en vez de las
controladas. Verificado:

| | antes | después |
|---|---:|---:|
| 7 controladas, 1 en movimiento | 61.2 % | **16.4 %** |
| 1 controlada (aislado) | 16.9 % | 16.9 % |

Coinciden. La dispersión de §11 era **determinista**, no estadística: el mismo
ensayo daba resultados distintos según cuántas articulaciones hubiera en la
lista de controladas, y eso no es ruido, es un fallo.

### Qué significa para lo anterior

- **§11 queda corregida.** El barrido sí es reproducible; lo que no era válido
  era el barrido con el fallo dentro.
- **El resultado de F2.3 se sostiene y ahora es significativo**: la diferencia
  entre 1–3 % de sobreimpulso a kp = 70 y 17 % a kp = 280 es **veinte veces
  `δ_min`**.
- Los tres artefactos encontrados en dos días —métrica de sobreimpulso contra la
  consigna, salto de par al cambiar ganancias, y este— tienen la misma forma:
  **algo que parece un resultado físico y es un fallo de medida**. La regla 7
  del protocolo («un hallazgo que contradiga la teoría es primero un fallo de
  medida») acertó tres de tres.

### Un aviso que salió de paso

En la corrida con el fallo, el lazo se quedó en **218.8 Hz de 250, con un 28.4 %
de ciclos tarde** — con el portero disparándose en el 45 % de los ciclos, siete
articulaciones registrándose y el modelo de gravedad activo. Sin el fallo el
portero no se dispara, pero conviene vigilar la frecuencia efectiva cuando se
acumulan las tres cosas.

---

## 13. `tuned_gff`: el conjunto definitivo con gravedad compensada

Barrido de aceptación con las ganancias sintonizadas y `--gravity-ff`, escalón
suave de 0.12 rad, las 14 de una en una. **14/14 dentro de tolerancia.**

| articulación | kp | kd | err final | sobreimp. | subida | par máx |
|---|---:|---:|---:|---:|---:|---|
| L_shoulder_pitch | 111 | 20.25 | −0.02° | 0.4 % | 227 ms | 7.82 / 40 Nm |
| L_shoulder_roll | **70** | 13.50 | +0.61° | **0.1 %** | 179 ms | 9.58 / 40 |
| L_shoulder_yaw | 126 | 8.10 | +0.41° | 0.2 % | 176 ms | 2.85 / 18 |
| L_elbow | 79 | 20.25 | −0.33° | 0.1 % | 180 ms | 3.78 / 18 |
| L_wrist_roll | 100 | 2.40 | −0.03° | 0.0 % | 176 ms | 0.44 / 19 |
| L_wrist_pitch | 100 | 6.00 | −0.05° | 0.0 % | 181 ms | 0.94 / 19 |
| L_wrist_yaw | 40 | 13.50 | −0.34° | 0.1 % | 198 ms | 0.69 / 19 |
| R_shoulder_pitch | 280 | 20.25 | +0.12° | 6.5 % | 167 ms | 6.42 / 40 |
| R_shoulder_roll | **70** | 13.50 | +0.18° | **0.2 %** | 227 ms | 9.05 / 40 |
| R_shoulder_yaw | 126 | 13.50 | −0.45° | 0.0 % | 199 ms | 1.57 / 18 |
| R_elbow | 79 | 20.25 | −0.69° | 1.8 % | 198 ms | 3.52 / 18 |
| R_wrist_roll | 100 | 5.40 | +0.16° | 0.0 % | 193 ms | 0.56 / 19 |
| R_wrist_pitch | 100 | 1.80 | −0.11° | 0.0 % | 163 ms | 0.81 / 19 |
| R_wrist_yaw | 100 | 2.70 | +0.19° | 0.0 % | 177 ms | 0.56 / 19 |

### Frente a `tuned` sin compensación (§7)

| | `tuned` | `tuned_gff` |
|---|---:|---:|
| sobreimpulso máximo | **21.0 %** | **6.5 %** |
| sobreimpulso de los `shoulder_roll` | 20.3 % y 21.0 % | **0.1 % y 0.2 %** |
| error permanente máximo | 1.09° | 0.69° |
| par máximo usado (hombro) | 14.06 / 40 Nm (35 %) | 9.58 / 40 Nm (24 %) |
| kp de `shoulder_roll` | 280 | **70** |

Los dos `shoulder_roll` —las articulaciones que peor estaban desde el primer
día— pasan de **20 % de sobreimpulso a 0.1 %**, con una cuarta parte del kp y
4 Nm menos de pico. Todas las diferencias están muy por encima de `δ_min`.

### Coherencia entre brazos

Sintonizados por separado, con datos y ajustes independientes:

| articulación | izquierdo | derecho |
|---|---|---|
| `shoulder_roll` | **70 / 13.50** | **70 / 13.50** |
| `elbow` | **79 / 20.25** | **79 / 20.25** |
| `shoulder_yaw` | 126 / 8.10 | 126 / 13.50 |
| `wrist_roll` / `wrist_pitch` | 100 | 100 |
| `shoulder_pitch` | 111 | 280 |
| `wrist_yaw` | 40 | 100 |

Coinciden exactamente en `shoulder_roll` y `elbow`, y en kp en cuatro más. Las
dos que discrepan —`shoulder_pitch` y `wrist_yaw`— son precisamente aquellas en
las que el criterio es plano: cualquier kp del rango da un resultado dentro de
tolerancia, así que el ganador lo decide el ruido. Ahí conviene elegir el kp
**bajo** de los dos por margen de par, no dejar que lo elija el barrido.

### Salud del lazo durante los barridos

197–208 Hz de 250, con 28–44 % de ciclos tarde. El perfil por ciclo dice que el
trabajo son **952 µs de un presupuesto de 4000** (CRC 508 µs, publicación
373 µs, gravedad 35 µs, el resto marginal), o sea un techo teórico de 1051 Hz.
La diferencia es competencia por el GIL entre el hilo de control, el ejecutor
que procesa `/lowstate` a 500 Hz y el hilo principal.

No invalida las medidas —la trayectoria se evalúa contra reloj (§F0) y F1 midió
la repetibilidad **en estas mismas condiciones**, con IQR de 0.5 pp— pero es lo
primero que hay que atacar si alguna vez hace falta más frecuencia.

---

## 14. F7.2 y F7.3: el puente a `xr_teleoperate` — 2026-09-09

`patches/xr_teleoperate_h1_2_tuning.patch`. Lleva las tres cosas a la vez —
`dq_des`, gravedad y ganancias por articulación— porque son inseparables: los kd
de `tuned_gff` se sintonizaron con velocidad de referencia, y sus kp bajos
funcionan porque el par de sostenimiento lo da el feedforward.

### Lo que se comprobó sin mover el robot

`H1_2_ArmController.__init__` lleva los brazos a 0° a 30 rad/s en cuanto se
construye, así que no se instancia para probar. Se montó un objeto mínimo y se
llamaron sueltos los dos métodos nuevos:

| comprobación | resultado |
|---|---|
| las 14 ganancias entran en `msg.motor_cmd` | coinciden con `tuned_gff` |
| el modelo carga con los parámetros de los dos brazos | sí |
| topes de `tau_ff` = 0.5·`tau_max` | 9 a 20 Nm |
| la rampa sube de 0 a 1 | 250 ciclos = 1 s a 250 Hz |
| `tau_ff` en la postura de reposo | −1.93 a +1.60 Nm, sin recortes |

### El mapeo de índices, que era el riesgo real

Dentro de `_gravity_tauff` la traducción es `t.get(int(jid), 0.0)`: las claves
de `t` son índices de `h1_2_joint_control` y `jid` es un `H1_2_JointArmIndex`.
Si esa correspondencia estuviera desplazada, los pares saldrían plausibles pero
**en la articulación equivocada**, que es peor que no compensar.

Comprobado de forma exacta, no estadística: los 14 `int(jid)` coinciden uno a
uno con `ARM_INDICES`. Ojo con `kLeftElbowRoll = 17`, que es el
`left_wrist_roll` —el nombre de `xr_teleoperate` es erróneo, el índice no—.

Y una prueba de perturbación: con un brazo estirado y el hombro a 90°, el par
mayor cae donde debe, `L_shoulder_pitch` −23.00 Nm y `R_shoulder_pitch` −22.66
Nm según el brazo movido.

### La comprobación contra el par medido NO vale, y por qué

Se intentó validar el modelo contra `tau_est` del robot real, con la idea de
que en equilibrio estático el par del motor iguala al de la gravedad. Salió rms
0.739 Nm, máximo 2.107 Nm, y el número es **inservible**: los pares medidos son
todos de unos 0.2 Nm, que no sostienen un brazo estirado.

En modo `ai` los brazos van con ganancia cero. Están flácidos: han caído hasta
apoyarse y siguen desplazándose a 0.031 rad/s. No hay equilibrio que comparar,
sino un transitorio. El propio script lo detectó por la velocidad y lo avisó.

Dato de seguridad que sale de paso: con el robot colgado y en `ai`, los brazos
**se van solos hacia dentro** —los `shoulder_roll` estaban en +6.0° y −1.9°,
por debajo del tope de ±10° que corresponde al codo estirado—. La postura de
partida no es segura por sí sola; hay que llevarlos a la postura de prueba
antes de nada.

Validar la magnitud sobre el robot exige sostener una postura, o sea mandar, y
eso ya es un ensayo con el robot en marcha.

### Lo que queda sin probar

La teleoperación completa, con visor y robot colgado. Requiere supervisión
presencial.

---

## Qué queda por hacer

- [x] ~~Repetir el barrido con una postura de partida reproducible~~ — hecho,
      sección 1 bis.
- [ ] Repetir el barrido del codo en dos o tres posturas más, para ver cuánto
      cambian las ganancias óptimas con la configuración del brazo.
- [ ] Sintonizar hombros y muñecas con el mismo método (aquí solo se validaron
      con las ganancias de `xr_teleoperate`, que salen bien).
- [x] Compensación de gravedad con `pinocchio` en vez del par medido punto a
      punto, y llevarla a `tauff_target` de `H1_2_ArmController`. (§14)
- [x] Mandar la velocidad de referencia en la teleoperación. (§14)
- [ ] Probar el parche combinado con el visor y el robot colgado.
- [ ] Comprobar si `arm_sdk` funciona con el robot activo (de pie). Si
      funcionara, evitaría tener que soltar la locomoción.
