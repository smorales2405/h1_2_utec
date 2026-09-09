# Parche: velocidad de referencia para el H1-2

`xr_teleoperate_h1_2_dq_ref.patch` — se aplica a
`xr_teleoperate/teleop/robot_control/robot_arm.py`.

```bash
cd xr_teleoperate && git apply ../patches/xr_teleoperate_h1_2_dq_ref.patch
```

> ⚠ **Este parche es la mitad de `xr_teleoperate_h1_2_tuning.patch`**, que hace
> esto y además compensa la gravedad y pone las ganancias por articulación. Los
> dos tocan el mismo bucle y **chocan**: se aplica uno **o** el otro, nunca los
> dos. Este está aquí para quien solo quiera esta mejora, que es la única de las
> tres que no depende de las otras. Ver [`README_tuning.md`](README_tuning.md).

## Qué hace

`H1_2_ArmController` manda hoy `msg.motor_cmd[id].dq = 0` siempre. El motor
calcula

```
tau = kp·(q_des − q) + kd·(dq_des − dq) + tau_ff
```

así que con `dq_des = 0` el término `kd` **frena el movimiento que se está
pidiendo**, y hace falta error de posición extra para vencerlo. El parche
deriva `dq_des` de la consigna y la manda.

Toca **solo** `H1_2_ArmController`. Las clases del G1, H1, H2 y R1 quedan
intactas.

## Cuánto vale, medido en el robot

Ensayos en `h1_2_joint_control` (ver `docs/06_RESULTADOS.md`):

| ensayo | con `dq_des` | con `dq_des = 0` | diferencia |
|---|---:|---:|---:|
| seno 0.5 Hz, codo, kp 140 / kd 6 | 35.8 mrad rms | 48.5 mrad | **+35 %** |
| chirp 0.2→2 Hz, codo, `tuned_gff` | 20.85 mrad rms | 30.18 mrad | **+45 %** |

**No es opcional con el conjunto `tuned_gff`.** Esos kd llegan a 20.25 y se
sintonizaron **con** velocidad de referencia; sin ella el orden de preferencia
entre candidatos se invierte (medido: con `dq_des`, kd 13.5 gana a kd 3 por un
31 %; sin ella, pierde por un 32 %).

## Por qué el filtro, y por qué a 10 Hz

`dq_des` se deriva de la consigna **ya recortada** por `clip_arm_q_target` —la
que ve el motor— con un filtro de primer orden. Sin filtrar, el ruido del IK
entra en el par multiplicado por kd, que con 20.25 es mucho.

Compromiso, simulado con 0.5 mrad de ruido por muestra a 250 Hz:

| corte | fidelidad a 1 Hz | a 3 Hz | par de ruido | % de `tau_max` del codo |
|---:|---:|---:|---:|---:|
| 3 Hz | 95 % | 69 % | 0.5 Nm | 3 % |
| 5 Hz | 98 % | 84 % | 0.9 Nm | 5 % |
| **10 Hz** | **99 %** | **95 %** | **1.9 Nm** | **10 %** |
| 20 Hz | 100 % | 98 % | 3.2 Nm | 18 % |

10 Hz conserva el 95 % de la derivada a 3 Hz —el extremo de la banda de la
teleoperación— por un 10 % del par del codo en ruido. Es el valor del parche.

> El nivel de ruido de 0.5 mrad es una **suposición**: no se ha medido el ruido
> real de la salida del IK de `xr_teleoperate`. Si resultara ser peor, hay que
> bajar el corte, y la tabla dice cuánto cuesta. `self.dq_filter_hz` es un
> atributo, así que se cambia sin tocar el parche.

## Parámetros

| atributo | valor | para qué |
|---|---:|---|
| `send_dq` | `True` | ponerlo a `False` restaura el comportamiento original |
| `dq_filter_hz` | 10.0 | corte del filtro |
| `dq_limit` | 2.0 rad/s | saturación, como pide el protocolo |

## Lo que este parche NO hace

No conecta la **compensación de gravedad**. `ctrl_dual_arm(q, tauff)` ya acepta
el par por articulación y la teleoperación le pasa ceros; el modelo
identificado está en `h1_2_joint_control/gravity.py` y falta el puente. Es la
otra mitad de F7.
