# Parche: sintonización completa del H1-2 en `xr_teleoperate`

`xr_teleoperate_h1_2_tuning.patch` — se aplica a
`xr_teleoperate/teleop/robot_control/robot_arm.py`.

```bash
cd xr_teleoperate
git apply ../patches/xr_teleoperate_h1_2_tuning.patch
```

Toca **solo** `H1_2_ArmController`: 140 inserciones y 2 borrados, que son
exactamente las dos líneas que deben cambiar. Las clases del G1, H1, H2 y R1
quedan intactas.

> ⚠ **Este parche INCLUYE `xr_teleoperate_h1_2_dq_ref.patch`.** No se aplican
> los dos: chocan, porque tocan el mismo bucle. Aplica uno **o** el otro.
> El de `dq_ref` está para quien solo quiera esa mejora, que es independiente.

## Las tres cosas que hace, y por qué van juntas

| | qué | qué vale |
|---|---|---|
| 1 | manda `dq_des` en vez de 0 | 35–45 % menos error rms |
| 2 | mete el par de gravedad en `tau_ff` | 85–98 % menos error permanente |
| 3 | ganancias **por articulación** del conjunto `tuned_gff` | sobreimpulso del hombro de 20 % a 0.1 % |

**Son inseparables.** `tuned_gff` tiene kd de hasta 20.25, sintonizados **con**
velocidad de referencia: sin (1), el orden de preferencia entre candidatos se
invierte. Y tiene kp de hasta cuatro veces más bajos que `tuned`, que funcionan
porque el par de sostenimiento lo da (2): sin gravedad, el error permanente
vuelve a ser `tau_g/kp` y con kp = 70 en el hombro es peor que antes.

## De dónde salen los números

Del paquete `h1_2_joint_control` de este mismo repositorio, que el parche
importa. La ruta se deduce de la del propio fichero; se puede forzar con
`H12_JOINT_CONTROL`.

- **Ganancias**: `config/gains.yaml`, conjunto `tuned_gff`. Barrido de kp y kd
  por articulación con la gravedad compensada, `δ_min` conocido.
- **Gravedad**: `logs/gravity_params_*.json`. Los parámetros de masa **no** son
  los del URDF, que da 2.57 Nm de error rms; se identificaron sobre el robot y
  dan **0.34 Nm**. Los dos brazos, ajustados por separado con datos distintos,
  coinciden en la masa de la mano al 6 %.

Detalle en [`h1_2_joint_control/docs/06_RESULTADOS.md`](../../h1_2_joint_control/docs/06_RESULTADOS.md),
secciones 9 a 13.

## Si algo falta, la teleoperación sigue

El parche degrada en vez de caerse. Si no encuentra `h1_2_joint_control`, si
falta `pinocchio` o si no hay parámetros identificados, **avisa por el log y
sigue** con las ganancias por defecto y sin compensación. Es peor, pero
funciona. Los tres avisos empiezan por `[H1_2]`.

## Entrar y salir: los dos movimientos que hace solo

`H1_2_ArmController` se construye en la línea 174 de `teleop_hand_and_arm.py` y
el `Press [r] to start` está en la 265. Al construirse lleva los brazos a **0°**,
91 líneas antes de preguntarte nada. Y al parar llama a
`ctrl_dual_arm_go_home()`, que los deja en **0°** y muere sin bajar ganancias.

Los dos extremos están mal, y el parche cambia los dos.

### Al entrar: rampa, no salto

El parche hace que el controlador **sostenga la postura donde esté** al arrancar
el hilo publicador, y luego rampe a 0° con un coseno alzado en
`H12_STARTUP_RAMP_S` segundos (4 por defecto).

> ⚠ **No uses `H12_ARM_VELOCITY_LIMIT` para ir despacio.** Parece lo natural y
> es una trampa: `clip_arm_q_target` recorta contra la posición **medida**, así
> que la consigna nunca va más de `límite × control_dt` por delante del brazo.
> Eso limita el error de posición y con él el par del PD, a lo sumo
> `kp × límite × control_dt`. Medido en el codo izquierdo (kp 79.4, dt 1/250),
> recorrido en 3 s pidiendo 17°:
>
> | límite | par máx | recorrido |
> |---:|---:|---:|
> | 0.3 rad/s | 0.10 Nm | **0.0°** |
> | 0.6 | 0.19 Nm | 0.1° |
> | 2.0 | 0.64 Nm | 1.2° |
> | 5.0 | 1.59 Nm | 12.0° |
> | 15.0 | 4.76 Nm | 18.0° |
>
> Bajarlo no frena el brazo: **lo desactiva**. Los 30 rad/s de fábrica no son
> generosos, son lo justo para que el codo tenga par para sostenerse.

### Al salir: a la postura de reposo, no a 0°

Soltar en 0° es lo peor posible: **0° de codo es la posición flexionada y no es
el mínimo de gravedad**. Sin ganancia el antebrazo cae solo hasta quedar
colgando y, con el hombro cerca de 0°, ese recorrido lleva **la mano contra la
pierna**. Medido: los codos pasan de 1° a 79° y 85° en segundos.

Tiene que resolverse **dentro** del controlador, mientras aún manda. Comprobado
que un script posterior no llega: entre que un proceso suelta y el siguiente
toma el control pasan ~6 s, y para entonces los codos ya han caído 73° y 79°.

El parche reemplaza `go_home` por tres tramos, y el orden es lo único que
importa:

1. los `shoulder_roll` salen a **±10°**, apartando la mano de la pierna antes
   de que el brazo recorra nada a lo largo del cuerpo;
2. los codos se estiran a **80°**, ya lejos de la pierna;
3. todo va a la postura de reposo, con el brazo ya estirado, que **es** el
   mínimo de gravedad; y las ganancias bajan a 0 en 1 s antes de dejar de
   publicar.

Los ángulos salen de `rest_posture_deg` en `config/gains.yaml`.

### Ciclo completo verificado en el robot

Arranque con rampa de 6 s → los 14 a menos de 1.6° de cero → `go_home` en 9.5 s
→ **deriva máxima al soltar: 0.23°**. Sin el parche, esa deriva era de 84°.

## Parámetros

| atributo | valor | para qué |
|---|---:|---|
| `send_dq` | `True` | `False` restaura `dq_des = 0` |
| `dq_filter_hz` | 10.0 | corte del derivador; bajar si el brazo vibra |
| `dq_limit` | 2.0 rad/s | saturación de `dq_des` |
| `use_gravity_ff` | `True` | `False` desactiva la compensación |
| `gains_set` | `"tuned_gff"` | otro conjunto de `gains.yaml` |
| `_grav_ramp_s` | 1.0 s | rampa de entrada del par, sin escalón |
| `_grav_frac` | 0.5 | tope de `tau_ff` como fracción del par máximo |

Y una variable de entorno:

| variable | para qué |
|---|---|
| `H12_STARTUP_RAMP_S` | segundos de la rampa inicial a 0° (4 por defecto) |
| `H12_ARM_VELOCITY_LIMIT` | rad/s; **no lo bajes**, ver arriba |
| `H12_JOINT_CONTROL` | ruta del paquete, si no se deduce sola |

## Verificado

Con el parche aplicado y **sin construir el controlador** —su `__init__` lleva
los brazos a 0° a 30 rad/s, así que no se instancia a la ligera— se comprobó
que los métodos nuevos cargan lo que deben:

- las 14 ganancias entran en `msg.motor_cmd` con los valores de `tuned_gff`;
- el modelo de gravedad se construye con los parámetros de los dos brazos;
- la rampa sube de 0 a 1 en 250 ciclos (1 s a 250 Hz);
- en la postura de reposo real, `tau_ff` va de −1.93 a +1.60 Nm, muy por debajo
  de los topes (9 a 20 Nm), sin recortes.

**Lo que NO se ha probado**: la teleoperación completa con el visor y el robot
colgado. Eso requiere supervisión presencial.

## No igualar las ganancias entre brazos

`R_shoulder_pitch` sale kp 280 y `L_shoulder_pitch` 111; `wrist_yaw`, 100 contra
40. **Parece una asimetría a corregir y no lo es.** Bajar el kp del derecho al
del izquierdo cuesta 26 y 10 veces `δ_min` y casi duplica su error: el brazo
derecho es peor en las siete articulaciones, de 1.06× a 2.18×, y necesita más
kp. El barrido detectó una diferencia física real entre los dos brazos.

Se queda como está. Detalle en
[`06_RESULTADOS.md` §13](../../h1_2_joint_control/docs/06_RESULTADOS.md).
