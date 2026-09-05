# Metodología de sintonización

## Qué se busca

Para cada articulación, el par (kp, kd) que:

1. **sigue la referencia** con error pequeño,
2. **no tiembla**,
3. **no satura** el motor,

en ese orden. Con un PD sin integral no se puede tener todo: subir kp reduce
el error permanente pero acerca la saturación y despierta la vibración; subir
kd amortigua pero mete retardo y ruido derivado.

## Las métricas, y por qué esas

Todas salen de `metrics.py` y se guardan en `logs/index.csv`.

### Del escalón (`--traj step`)

| Métrica | Qué dice |
|---|---|
| `rise_time` | del 10 % al 90 %. Rapidez bruta. Baja con kp alto |
| `overshoot` | cuánto se pasa. Sube con kp alto y kd bajo |
| `settling_time` | hasta quedarse dentro del ±2 % |
| `steady_error` | **lo que no se corrige nunca**: `tau_gravedad/kp` |
| `oscillations` | cruces por el valor final tras el primer pico. > 4 es mala señal |
| `peak_tau` | contra el `tau_max` del URDF |

### Del seguimiento (`--traj sine` / `chirp` / al mantener)

| Métrica | Qué dice |
|---|---|
| `rms_error` | error de seguimiento. Lo que verá la teleoperación |
| `mean_error` | el sesgo: la caída por gravedad, separada del ruido |
| `chatter_dq` | **el índice de temblor** (ver abajo) |
| `peak_freq` | a qué frecuencia tiembla. Ayuda a distinguir la causa |
| `tau_headroom` | margen de par que queda |

### El índice de temblor

`chatter_dq` es el valor eficaz de la velocidad articular **por encima de
8 Hz**, obtenido de la FFT (identidad de Parseval, con ventana de Hann y
corrección de su pérdida de potencia). Se mide sobre `dq` y no sobre `q`
porque la derivada realza justo la banda que interesa.

Por qué 8 Hz: un movimiento de teleoperación tiene contenido útil por debajo
de 2–3 Hz. Todo lo que aparezca por encima de 8 Hz es temblor, no seguimiento.

Lecturas orientativas, medidas en reposo con `01_hold.py`:

| `chatter_dq` | interpretación |
|---|---|
| < 0.02 rad/s | ruido del encoder. Limpio |
| 0.02 – 0.05 | perceptible al tacto, tolerable |
| > 0.05 | vibración audible o visible. Investigar |
| > 0.15 | inestable. Bajar kp o revisar quién más publica |

`peak_freq` distingue causas: un pico entre **10 y 30 Hz** suele ser la
mecánica excitada por kd; un pico cerca de la **mitad de la frecuencia de
publicación** (~125 Hz a 250 Hz de lazo) apunta a que la consigna llega a
saltos o a que hay dos publicadores alternándose.

## Procedimiento

### Paso 0 — Diagnóstico y sujeción

```bash
source scripts/env.sh
python3 scripts/00_diagnose.py
```

Nada de lo demás vale si `/lowcmd` está en disputa o si un motor está caliente.

### Paso 1 — Sostener sin mover

```bash
python3 scripts/01_hold.py --seconds 10 --weight 0    # sin autoridad
python3 scripts/01_hold.py --seconds 10               # autoridad real
```

Verifica tres cosas a la vez: que el canal llega, que kp basta para sostener
el brazo contra la gravedad, y cuál es el nivel de ruido de partida. Ese
`chatter_dq` en reposo es la línea base contra la que se compara todo lo demás.

### Paso 2 — Una articulación, un escalón

Empezar por la que menos puede estropear: una muñeca, no un hombro.

```bash
python3 scripts/02_move.py --joint L_wrist_yaw --traj step --amp 0.10
python3 scripts/05_plot.py --last
```

### El techo de kp no lo pone el seguimiento, lo pone la saturación

Antes de barrer nada conviene entender por qué el barrido no puede ser abierto
por arriba.

En las articulaciones con carga estática apreciable —los hombros, el codo— el
error de seguimiento está dominado por la caída de gravedad, `tau_g/kp`. Es
decir, **baja monótonamente con kp**. Un criterio que solo mire el error elegirá
siempre el kp más alto que se le ofrezca, y ampliar el rango solo mueve la
respuesta más arriba. Medido en el hombro izquierdo: kp = 280 (el doble de la
referencia) ganaba los tres barridos, con el aviso de «ganador en el borde» en
los tres.

Lo que sí pone un límite es la **saturación**. Un kp alto es inofensivo mientras
la consigna se mueva despacio, y peligroso en cuanto da un salto — que en
teleoperación ocurre cada vez que parpadea el seguimiento del mando. De ahí el
tope que aplica `09_tune_all.py`:

```
kp_max = tau_abort_fraction · tau_max / salto        (salto = 0.10 rad por defecto)
```

| grupo | `tau_max` | `kp_max` |
|---|---:|---:|
| hombro pitch/roll | 40 Nm | **280** |
| hombro yaw, codo | 18 Nm | **126** |
| muñecas | 19 Nm | **133** |

Esto descartó un resultado que el barrido sin tope había dado por bueno:
`L_shoulder_yaw` salía con kp = 280, y es un motor de 18 Nm — un salto de
consigna de 3.7° lo habría saturado. También deja el codo sintonizado en la
primera sesión (kp = 140) **por encima de su tope** de 126.

La rejilla de kp es geométrica, no lineal: kp actúa como `1/error`, así que
interesa muestrear en proporción.

### Paso 3 — Barrer kp y kd

```bash
# primero kp, con kd en el valor de referencia
python3 scripts/03_tune.py --joint L_elbow --kp-list 50,80,110,140,170 --kd-list 2

# después kd, con el kp ganador
python3 scripts/03_tune.py --joint L_elbow --kp-list 110 --kd-list 0.5,1,2,3,5
```

Barrer los dos a la vez multiplica los ensayos sin aportar mucho: kp domina el
error y kd el temblor, y están poco acoplados en este rango. Si el ganador cae
en el borde de la lista, el script avisa para ampliar el rango.

El criterio es explícito y se puede cambiar:

```
J = w_err·err_rms[mrad] + w_chatter·temblor[10⁻² rad/s] + w_overshoot·sobreimpulso[%]
```

Si lo que molesta es la vibración, `--w-chatter 3`. Si lo que importa es la
fidelidad del seguimiento para teleoperación, `--w-err 2`.

### kd óptimo a 0.5 Hz no es kd óptimo a 3 Hz

El barrido de kd elige el que menos error y menos temblor da **a la frecuencia
con la que se ha medido**. Y a 0.5 Hz, subir kd casi siempre gana: el temblor
baja y el error apenas se mueve. Medido en el hombro izquierdo, kd salía en el
borde superior del barrido una y otra vez (13.5, que es 4.5 veces la
referencia).

Eso no significa que kd deba ser 13.5. Un kd alto es un filtro paso bajo sobre
la respuesta: quita ruido, y **añade retardo a los movimientos rápidos**. A
0.5 Hz no se nota; a 2 o 3 Hz —que es donde vive un gesto brusco del
operador— sí.

Por eso el kd ganador **hay que confirmarlo con un chirp** antes de darlo por
bueno:

```bash
python3 scripts/02_move.py --channel lowcmd --joint L_shoulder_pitch \
    --traj chirp --amp 0.08 --f0 0.2 --f1 3.0 --duration 20 --kp 280 --kd 13.5
```

y compararlo con el kd de referencia. Si a 3 Hz el candidato sigue peor que el
de partida, el barrido a 0.5 Hz ha sobreajustado.

### Paso 4 — Confirmar con seguimiento, no solo con escalón

Un escalón premia ganancias agresivas que luego tiemblan siguiendo una
trayectoria. Confirmar siempre:

```bash
python3 scripts/02_move.py --joint L_elbow --traj sine --amp 0.15 --freq 0.5 --kp 110 --kd 2
python3 scripts/02_move.py --joint L_elbow --traj chirp --amp 0.08 --f0 0.2 --f1 3.0
```

El chirp enseña de un vistazo hasta qué frecuencia sigue y dónde resuena.

### Paso 5 — Guardar y recorrer

```bash
python3 scripts/03_tune.py --joint L_elbow --kp-list 110 --kd-list 2 --write tuned
python3 scripts/04_sweep_arms.py --gains tuned
```

## Comparaciones que merece la pena hacer

| Comparación | Comando | Qué se aprende |
|---|---|---|
| Oficial vs. teleoperación | `--gains official_arm_sdk` vs `--gains xr_teleoperate` | El codo pasa de kp=50 a kp=140. ¿Mejora o solo endurece? |
| Con y sin velocidad de referencia | `--zero-dq` | Cuánto retardo mete la decisión de `xr_teleoperate` de mandar `dq = 0` |
| Escalón duro vs. suave | `--traj step` vs `--traj smooth_step` | Cuánto del sobreimpulso es del escalón y cuánto de las ganancias |
| 250 vs. 500 Hz | `--rate` | Si la frecuencia de publicación influye en el temblor |

## Punto de partida sugerido por articulación

Del ejemplo oficial `arm_sdk`, que es lo más contrastado que hay:

| grupo | kp | kd | comentario |
|---|---:|---:|---|
| hombro pitch/roll | 120 | 2.0 | 40 Nm de motor, cargan el brazo entero |
| hombro yaw | 80 | 1.5 | 18 Nm |
| codo | 50 | 1.0 | **18 Nm**, y carga con antebrazo + mano |
| muñecas | 50 | 1.0 | 19 Nm, poca inercia |
| cintura | 200 | 2.0 | 200 Nm |

`xr_teleoperate` sube hombros y codo a kp=140 / kd=3 y deja las muñecas en
kp=50 / kd=2. Con 18 Nm en el codo, kp=140 significa que **0.13 rad (7.4°) de
error ya saturan el motor**: es una elección agresiva que hay que validar, no
dar por buena.
