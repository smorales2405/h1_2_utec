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

## Qué queda por hacer

- [x] ~~Repetir el barrido con una postura de partida reproducible~~ — hecho,
      sección 1 bis.
- [ ] Repetir el barrido del codo en dos o tres posturas más, para ver cuánto
      cambian las ganancias óptimas con la configuración del brazo.
- [ ] Sintonizar hombros y muñecas con el mismo método (aquí solo se validaron
      con las ganancias de `xr_teleoperate`, que salen bien).
- [ ] Compensación de gravedad con `pinocchio` en vez del par medido punto a
      punto, y llevarla a `tauff_target` de `H1_2_ArmController`.
- [ ] Mandar la velocidad de referencia en la teleoperación.
- [ ] Comprobar si `arm_sdk` funciona con el robot activo (de pie). Si
      funcionara, evitaría tener que soltar la locomoción.
