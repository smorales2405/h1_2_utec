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

## El movimiento inicial, que es lo peligroso

`H1_2_ArmController` se construye en la línea 174 de `teleop_hand_and_arm.py` y
el `Press [r] to start` está en la 265. Al construirse arranca ya el hilo
publicador con `q_target = zeros(14)` y `arm_velocity_limit = 30 rad/s`: **el
robot se mueve 91 líneas antes de que te pregunte nada**, y la `[r]` solo decide
cuándo empieza a seguirte a ti. Con los codos estirados son 80° de recorrido.

El parche añade `H12_ARM_VELOCITY_LIMIT` (rad/s) para bajar ese límite, y va lo
primero del método, antes de cualquier import que pueda fallar, porque es lo
único ahí dentro que protege al robot.

[`scripts/arranca_teleop.sh`](../scripts/arranca_teleop.sh) lo pone y además
coloca los brazos en 0° a 0.15 rad/s antes de lanzar la teleoperación.

Pero **colocarlos antes no sustituye al límite de velocidad**, y esto está
medido: al soltar, los codos vuelven solos a 79° y 85° en segundos, porque 0°
es *flexionado* y no es el mínimo de gravedad. Las otras doce se quedan a menos
de 6°. Para el codo —el del recorrido de 80°— lo único que protege es bajar el
límite.

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
| `H12_ARM_VELOCITY_LIMIT` | rad/s del movimiento inicial (de fábrica 30) |
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
