# Postura de ensayo y qué hace cada articulación durante una prueba

## Qué pasa con las demás articulaciones mientras se prueba una

**Se sostienen, no se dejan libres ni se mandan a un ángulo absoluto fijo.**

`H12Client.engage()` lee `/lowstate` una vez, guarda esa postura en `q0` y fija
`q_des = q0` para **todas** las articulaciones que comanda, cada una con su
`kp`/`kd` del conjunto activo. Es decir: las otras trece quedan clavadas donde
estuvieran, con la misma rigidez que tendrían durante la teleoperación. No es
una elección menor: si se dejaran flojas, el brazo se movería por reacción al
movimiento de la articulación bajo prueba y la medida saldría contaminada.

Con detalle, según el canal:

| | `arm_sdk` | `lowcmd` (modo debug) |
|---|---|---|
| Articulaciones comandadas | 15: los 14 brazos + cintura | las 27 |
| Las no probadas | clavadas en `q0` con sus ganancias | ídem |
| Piernas | **las lleva el robot**, no las tocamos | según `--legs`: `free` (kp=kd=0, por defecto), `damp` (kd=2) o `hold` (kp=300) |

En `03_tune.py` y `04_sweep_arms.py` el `engage()` se hace **una sola vez** al
principio, y entre candidato y candidato la articulación bajo prueba vuelve a su
punto de partida con `ramp_to()`. Así todos los candidatos compiten desde el
mismo sitio: si no, cada uno arrancaría donde lo dejó el anterior.

Al terminar —también por Ctrl-C, por SIGTERM o por un aborto— se devuelven
**todas** las articulaciones comandadas a `q0`, la postura que tenía el robot
antes de que lo tocáramos.

## La postura de ensayo

Hasta la sesión del 2026-09-04, la postura de partida era «donde estuviera el
robot al arrancar». Eso tiene dos problemas: no es reproducible entre sesiones,
y puede ser peligrosa. Con los hombros cerca de cero, un ensayo de
`shoulder_roll` mete el brazo **contra el torso**: medido, 6.3° de error que no
se cerraba nunca y 29.6 Nm de par, hasta que saltó la protección.

Ahora hay una postura explícita en `config/gains.yaml`:

```yaml
test_posture_deg:
  joints:
    L_shoulder_roll:  18.0
    R_shoulder_roll: -18.0
```

Los scripts de movimiento la aplican **después** de `engage()` y antes de medir,
despacio (0.25 rad/s) y con verificación de llegada. Se desactiva con
`--no-posture`.

`q0` sigue siendo donde estaba el robot al empezar —ahí se le devuelve al
final—, y aparece un segundo vector, `q_base`, que es desde dónde parten los
ensayos. Para las articulaciones que la postura no toca, los dos coinciden.

## La consigna no es la posición: hay que cerrar el lazo

La postura de ensayo es una posición **física** —existe para que el brazo no
toque el torso—, así que lo que importa es dónde acaba el brazo, no qué se le
pidió. Y con un PD sin integral esas dos cosas **no coinciden**.

Medido en `R_shoulder_roll` con kp = 140: se le pide −18.0° y se queda en
**−15.5°**, con 6.4 Nm de par de sostenimiento. Es el error permanente de
siempre, `tau_g/kp` = 6.4/140 = 45.7 mrad = 2.6°, y coincide con el medido. En
una postura pensada para no tocarse, quedarse 2.5° **más cerca del cuerpo** de
lo previsto no vale.

Por eso `go_to_test_posture()` corrige: manda, mide, desplaza la consigna por lo
que falte, y repite hasta que el ángulo REAL entra en 0.5° (o se agotan seis
iteraciones). Es acción integral, aplicada una vez.

Validado el 2026-09-04 en tres pasadas independientes, canal `lowcmd`, ganancias
de `xr_teleoperate`:

| pasada | `L_shoulder_roll` real | `R_shoulder_roll` real | consigna extra |
|---|---:|---:|---|
| sin corregir | — | −15.5° | — |
| 1 | +18.15° | −18.38° | +2.83° / −2.94° |
| 2 | +17.71° | −17.65° | +2.44° / −2.17° |

Con corrección, los dos hombros caen dentro de ±0.4° del objetivo. La consigna
extra que hace falta (2.2°–2.9°) es justo la caída por gravedad.

Sosteniendo ahí: temblor 0.009–0.013 rad/s, indistinguible del ruido de fondo, y
6.3–7.6 Nm de los 40 del motor (81–84 % de margen).

**Consecuencia para los ensayos**: `q_base` no es el ángulo objetivo sino la
consigna corregida, que es la que mantiene el brazo donde se quiere. Las
amplitudes de los ensayos se cuentan desde ahí.

## Los topes blandos

El URDF describe el final de carrera **mecánico** de cada articulación por
separado. No sabe nada de que el brazo pueda chocar con el torso. Los topes de
autocolisión, medidos en el robot montado, van aparte:

```yaml
soft_limits_deg:
  joints:
    L_shoulder_roll: {min: 10.0}    # el roll positivo separa el brazo IZQUIERDO
    R_shoulder_roll: {max: -10.0}   # el negativo separa el DERECHO
```

`Gains.limits(idx)` devuelve la intersección de los dos, y **todo** recorte de
consigna pasa por ahí: `set_target`, `ramp_to` y la evaluación de trayectorias
en el lazo de control. No hay forma de comandar un ángulo que los viole.

Los dos tipos de tope se tratan distinto a propósito:

* **URDF**: se estrechan con `safety.joint_limit_margin` (0.05 rad), porque
  llegar al final de carrera es un golpe mecánico.
* **Blandos**: se aplican **tal cual**, sin margen extra. Ya llevan dentro el
  margen de seguridad que se ha decidido; sumarles otros 2.9° dejaría el
  hombro sin recorrido útil.

Resultado para los hombros:

| | URDF | efectivo |
|---|---|---|
| `L_shoulder_roll` | −21.8° … +194.8° | **+10.0°** … +191.9° |
| `R_shoulder_roll` | −194.8° … +21.8° | −191.9° … **−10.0°** |

Desde la postura de ensayo (±18°) quedan **8° de recorrido hacia el cuerpo**,
suficiente para una amplitud de 0.12 rad (6.9°) sin llegar al tope.

## Elegir el sentido del ensayo

`--direction auto` (por defecto) va hacia donde queda **más recorrido
articular**. Desde ±18°, eso es *alejarse* del cuerpo: es el sentido seguro y
el que se usa salvo que se pida otra cosa.

Para ejercitar el recorrido *hacia* el cuerpo —que es donde está el límite de
autocolisión— hay que pedirlo, y el signo es distinto en cada brazo porque son
especulares:

```bash
# hombro derecho: de −18° hacia −11.1°
python3 scripts/02_move.py --joint R_shoulder_roll --direction positive --amp 0.12

# hombro izquierdo: de +18° hacia +11.1°
python3 scripts/02_move.py --joint L_shoulder_roll --direction negative --amp 0.12
```

Si se pide una amplitud que no cabe, el tope blando manda: la amplitud se
recorta, o el sentido se invierte si al otro lado sí cabe. El script lo dice.
