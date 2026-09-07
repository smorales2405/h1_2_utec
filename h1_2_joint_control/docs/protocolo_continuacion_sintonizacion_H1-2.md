# Protocolo de Continuación — Sintonización de las Articulaciones de Brazo del H1-2

**Alcance:** cerrar la sintonización de los 14 GDL de brazo del Unitree H1-2 hasta un
conjunto de ganancias defendible para (a) teleoperación y (b) manipulación en contacto
con las manos Inspire RH56DFTP-2 montadas.

**Punto de partida:** `smorales2405/h1_2_utec` → `h1_2_joint_control`, sesiones del
2026-09-04 y 2026-09-05. Este protocolo continúa `docs/04_METODOLOGIA.md` y se apoya en
la infraestructura ya existente (`client.py`, `metrics.py`, `trajectories.py`,
`recorder.py`, postura de ensayo y topes blandos de `gains.yaml`).

> Todas las precondiciones de seguridad de `docs/03_SEGURIDAD.md` siguen vigentes sin
> cambios: robot colgado del arnés, nadie al alcance, mando a mano, `L2 + B`. Este
> documento no las repite, las asume.

---

## 0. Estado de partida — qué está cerrado y qué no

### Cerrado (no repetir)

| Resultado | Referencia |
|---|---|
| `/lowcmd` en disputa con el controlador `ai`; solución vía `06_debug_mode.py` | `01_DIAGNOSTICO.md` |
| Postura de ensayo reproducible (±18° reales, con corrección integral de una pasada) | `07_POSTURA.md` |
| Topes blandos de autocolisión brazo-torso (±10° en `shoulder_roll`) | `gains.yaml` |
| 14/14 siguen la referencia con las ganancias de `xr_teleoperate` | `06_RESULTADOS.md` §1, §1bis |
| Techo de kp por saturación: `kp_max = 0.7·τ_max/0.10` | `04_METODOLOGIA.md` |
| Métrica de sobreimpulso corregida (contra valor final, no contra consigna) | `06_RESULTADOS.md` §7 |
| Error permanente = `τ_g/kp`, verificado a la décima de grado | `06_RESULTADOS.md` §3, §1bis |
| Coste de `dq_des = 0`: +35 % de error rms | `06_RESULTADOS.md` §5 |
| kd óptimo depende de si se manda `dq_des` (el orden se invierte) | `05_BITACORA.md` |
| Conjunto `tuned` completo para las 14 | `gains.yaml` |

### Abierto — y por qué importa cada uno

| # | Hueco | Consecuencia si no se cierra |
|---|---|---|
| H1 | **N = 1 en casi todo.** La dispersión entre pasadas medida (21.2 vs 35.8 mrad, §6) es del orden de las mejoras que se reportan (31–46 %) | Ninguna diferencia por debajo de ~15 mrad es defendible. Compromete todo §7 |
| H2 | **Jitter del lazo no medido.** Se publica a 250 Hz pero no hay histograma de `dt` | Un jitter alto se confunde con falta de amortiguamiento y contamina todo kd |
| H3 | **Una sola postura.** Todo lo de §4 y §7 está medido con el codo a 85° y el brazo colgando | Las ganancias pueden no valer en la postura de trabajo real |
| H4 | **Sin compensación de gravedad por modelo.** `τ_ff` se mide punto a punto | kp está pegado al techo de saturación *porque* falta `τ_ff`. Es la causa raíz, no un extra |
| H5 | **Sin carga sujetada.** Los ensayos llevan las manos montadas (configuración de operación), pero ninguno con objeto en la mano | La carga desplaza τ_g y la inercia justo en el extremo. Es la condición de la tesis |
| H6 | **Sin caracterización en frecuencia formal.** El chirp se usa comparativamente, no para extraer ancho de banda | No hay un número citable de BW para la tesis |
| H7 | **Todo medido articulación por articulación.** Nunca las 7 a la vez | El acoplamiento inercial no está probado. Es lo que hará la teleoperación |
| H8 | **`tuned` es inseguro si `xr_teleoperate` manda `dq = 0`.** Con kd = 13.5 el codo satura solo con frenar a 2 rad/s | Riesgo real de aborto o de par excesivo en la primera sesión de teleoperación |
| H9 | **Sobreimpulso del 20 % en los dos `shoulder_roll`** con `tuned` | Real, no artefacto. Se resuelve con F2, no bajando kd |
| H10 | **Criterio de selección optimizado para seguimiento, no para contacto** | Para la tesis, el brazo rígido convierte error de posición en fuerza de impacto sobre el objeto |

---

## Orden de ejecución y dependencias

```
F0 ──> F1 ──> F2 ──> F3 ──> F5 ──┐
                │                 ├──> F7 ──> F8
                └──> F4 ──> F6 ──┘
```

F0 y F1 son bloqueantes: sin ellas, los resultados de las demás no son interpretables.
F2 es la de mayor retorno y probablemente disuelve H4, H9 y buena parte de H10 a la vez.

---

## F0 — Temporización del lazo (bloqueante, sin mover el robot)

**Por qué.** Todos los kd sintonizados dependen de la fase real del lazo. Un jitter p99
de 4 ms sobre un periodo de 4 ms es un 100 % de variación y se manifiesta exactamente
igual que un amortiguamiento insuficiente.

**Script nuevo:** `10_loop_timing.py` (solo lectura, no comanda).

**Procedimiento**

1. Publicar en un tópico inerte (`--dry-run`) a 250 Hz durante 5 min. Registrar `dt`
   de cada iteración con `time.monotonic()`.
2. En paralelo, registrar el intervalo de llegada de `/lowstate` durante los mismos 5 min.
3. Repetir a 500 Hz.
4. Repetir con `matplotlib` importado y con el `recorder` activo, para ver si el logging
   introduce jitter.

**Reportar**

| Métrica | 250 Hz | 500 Hz |
|---|---|---|
| `dt` mediana / p95 / p99 / máx | | |
| Deriva acumulada en 5 min | | |
| Intervalo de `/lowstate`: mediana / p99 | | |
| Muestras perdidas (`dt > 2·T`) | | |

**Criterio de aceptación:** `dt` p99 ≤ 1.2 · T y cero muestras perdidas. Si no se cumple:
prioridad `SCHED_FIFO`, aislamiento de CPU (`isolcpus`), desactivar C-states, y valorar
`PREEMPT_RT`. Documentar el resultado en `05_BITACORA.md` antes de seguir.

**Salida:** una fila fija en la cabecera de todo informe posterior. Sin este número, los
kd de §7 no son reproducibles en otra máquina.

---

## F1 — Consolidación estadística (bloqueante)

**Por qué.** §6 documenta 21.2 vs 35.8 mrad para el mismo ensayo. Esa dispersión es del
mismo orden que las mejoras reportadas en §7. Hasta cuantificarla, no se puede afirmar
que `tuned` sea mejor que `xr_teleoperate`.

### F1.1 — Efecto de fricción estática (hipótesis de §6)

Ensayo canónico: `L_elbow`, seno 0.12 rad a 0.5 Hz, kp = 126, kd = 13.5, `dq_des` activa.

| Condición | Descripción | N |
|---|---|---|
| A — frío | primer ensayo tras ≥ 10 min de reposo | 5 |
| B — precalentado | tras 30 s de seno continuo a la misma amplitud | 5 |
| C — dentro de barrido | tras 5 candidatos previos | 5 |

**Reportar** mediana e IQR de `rms_error` por condición. Si A ≫ B, la hipótesis de §6
queda confirmada y **se adopta un precalentamiento obligatorio de 30 s** antes de toda
medida, que pasa a `_common.py`.

### F1.2 — Resolución mínima detectable

```bash
python3 scripts/02_move.py --joint L_elbow --traj sine --amp 0.12 --freq 0.5 \
    --kp 126 --kd 13.5 --repeats 10 --tag repro_bloque1
# repetir como bloque2 y bloque3, separados ≥ 20 min, con el robot en reposo entre bloques
```

Tres bloques × 10 repeticiones, con precalentamiento, en tres articulaciones
representativas (`L_shoulder_roll`, `L_elbow`, `L_wrist_pitch`).

**Producto:** `δ_min = 1.5 × IQR` global por articulación.

**Regla que se adopta:** ninguna diferencia menor que `δ_min` se reporta como efecto.
Escribirla en `04_METODOLOGIA.md`.

### F1.3 — Revalidación de §7 con N adecuado

Repetir la comparación `xr_teleoperate` vs `tuned` con `--repeats 5` en las 14
articulaciones, orden aleatorizado, registrando temperatura al inicio y fin de cada
bloque.

**Reportar:** mediana ± IQR por articulación y por conjunto, y marcar explícitamente
cuáles de las mejoras del 31–46 % sobreviven a `δ_min`. Es previsible que las de las
muñecas (31–42 % sobre 6–16 mrad) queden por debajo del umbral y las de los hombros
(44–46 % sobre 17–57 mrad) sobrevivan. Ese resultado, si sale así, es honesto y hay que
publicarlo tal cual.

---

## F2 — Compensación de gravedad por modelo

**Por qué es la fase de mayor retorno.** §7 concluye que kp acaba pegado al techo porque
el error lo domina `τ_g/kp`, y §3 demuestra que `τ_ff` elimina hasta el 92 % de ese
error. Con `τ_ff` activo, el barrido de kp deja de ser monótono y aparece un óptimo
interior. Eso resuelve H4, H9 y baja el kp necesario, que es lo que quiere H10.

### F2.1 — Modelo y validación

**Script nuevo:** `11_gravity_model.py`, con `pinocchio` sobre el URDF de
`smorales2405/h1_2_inspire_description`.

1. Cargar el URDF **con las manos Inspire montadas**, que es la configuración de todos
   los ensayos existentes. **Verificar primero la masa que declara el URDF**: la mano
   real pesa ~800 g, y las descripciones publicadas suelen arrastrar el valor del
   RH56DFX sin sensores táctiles, que es apreciablemente menor. Si no coincide,
   corregirla antes de calcular nada, junto con el centro de masa y una inercia
   estimada como cilindro equivalente. Documentar la estimación.
2. Calcular `g(q)` para las 14 articulaciones.
3. **Validar contra lo ya medido**: los cuatro puntos de §3 y §1bis
   (`L/R_shoulder_roll` a ±18° y ±27°, con 4.22, −4.34, 9.63 y −10.36 N·m) son datos de
   validación gratuitos que ya están en el repositorio.
4. Añadir 12 puntos nuevos: cada articulación de un brazo medida con `07_gravity_ff.py`
   en tres posturas distintas.

**Criterio de aceptación:** `|τ_modelo − τ_medido| < max(1.0 N·m, 15 %)` en los 16 puntos.

**Si falla, identificar la masa de la mano sin desmontar nada.** La discrepancia es lineal
en `m_mano`:

```
τ_medido(q) − τ_modelo_sin_mano(q) = m_mano · g · r_mano(q)
```

Midiendo τ_g en 6–8 posturas con brazos de palanca distintos (variando `elbow` y
`shoulder_pitch`, que es lo que más cambia `r_mano`) y ajustando `m_mano` por mínimos
cuadrados:

| Residuo tras el ajuste | Diagnóstico | Acción |
|---|---|---|
| Plano, sin estructura | el error era la masa/CoM de la mano | corregir el URDF y seguir |
| Con estructura respecto a `q` | el error está en los eslabones del brazo | desmontar **una** mano y repetir 6 medidas estáticas de τ_g para separar las dos contribuciones |

Ese desmontaje es un diagnóstico contingente de media hora, no una fase planificada: la
configuración de referencia de todo el protocolo es **con las manos montadas**.

### F2.2 — Integración en el lazo

- `τ_ff = g(q_des)` evaluado a la tasa del lazo (no a la de la trayectoria).
- Rampa de activación de 1 s al enganchar, para no meter un escalón de par.
- Saturación de `τ_ff` a `0.5 · τ_max` como red de seguridad independiente del abort.
- Bandera `--gravity-ff on|off` en `02_move.py`, `03_tune.py` y `09_tune_all.py`.

**Prueba de humo obligatoria antes de sintonizar nada:** `01_hold.py --seconds 30` con
`τ_ff` activo y kp reducido a la mitad. El brazo debe sostenerse. Si se cae o si el par
crece, el signo del modelo está invertido: abortar y revisar convención de ejes.

### F2.3 — Re-sintonización con `τ_ff` activo

Repetir el barrido de kp de `09_tune_all.py` en `L_shoulder_roll`, `L_elbow` y
`L_wrist_pitch`, con `τ_ff` activo, `--repeats 3`.

**Hipótesis a contrastar:** con `τ_ff` activo, el ganador de kp deja de estar en el
borde superior y aparece un mínimo interior.

**Criterio de aceptación:**
- Error permanente en `shoulder_roll` a ±27° por debajo de **0.5°** con `kp ≤ 140`.
- Sobreimpulso de `shoulder_roll` por debajo de **10 %** (hoy: 20 % con kp = 280).
- Ganador de kp **no** en el borde de la rejilla en al menos 2 de las 3 articulaciones.

Si se cumple, se genera el conjunto `tuned_gff` y **queda deprecado `tuned`**, cuyos
kp = 280 existen solo para tapar la falta de compensación.

---

## F3 — Dependencia de la postura

**Por qué.** `06_RESULTADOS.md` §4 lo marca explícitamente como cautela: todo medido con
el codo a 85° y el brazo colgando.

### Rejilla

| Postura | `shoulder_pitch` | `shoulder_roll` | `elbow` | Representa |
|---|---:|---:|---:|---|
| P1 — reposo | 0° | ±18° | 85° | la ya medida (referencia) |
| P2 — trabajo | −40° | ±25° | 60° | alcance frontal a mesa |
| P3 — extendida | −70° | ±35° | 20° | peor caso de inercia y gravedad |

P3 es el peor caso en los dos ejes a la vez: máximo brazo de palanca y máxima inercia
vista por el hombro. Verificar antes que P3 respeta los topes blandos y que hay espacio
físico bajo el arnés.

### Ejecución

Para cada postura y cada una de 4 articulaciones (`shoulder_pitch`, `shoulder_roll`,
`elbow`, `wrist_pitch`) del brazo izquierdo:

```bash
python3 scripts/03_tune.py --joint L_elbow --posture P2 \
    --kp-list 60,80,110,140,190,260 --kd-list 6 --gravity-ff on --repeats 3
```

**Reportar** por articulación: `kp*`, `kd*`, `rms_error` y `τ_pico` en las tres posturas,
más la razón `kp*_max / kp*_min`.

### Decisión (escribirla antes de mirar los datos)

| Condición | Decisión |
|---|---|
| `kp*_max / kp*_min < 1.5` en todas | Un solo conjunto, elegido en **P3** (peor caso). Documentar la degradación en P1 y P2 |
| Razón ≥ 1.5 en alguna | *Gain scheduling* por postura, con `kp(q) ∝ M_ii(q)` calculado del mismo modelo de `pinocchio`. Se implementa como tabla de 3 puntos con interpolación lineal |

> Nota de coherencia con la tesis: si sale *gain scheduling*, es el mismo argumento
> estructural que usas para el supervisor adaptativo de la mano (la ganancia de lazo
> escala con la planta y una sintonización única no preserva el margen). Vale la pena
> señalarlo en el documento; refuerza los dos capítulos.

---

## F4 — Carga sujetada

**Por qué.** Las manos ya están montadas en todos los ensayos existentes, así que la
configuración base está cubierta y **no hay que ensayar sin ellas**. Lo que falta es la
carga: un objeto en la mano añade masa por delante del punto donde ya hay 800 g, y es la
condición en la que operará el sistema durante la manipulación.

### F4.1 — Verificación mecánica previa

- [ ] Masa de cada mano en balanza (referencia ~800 g) y masa del conjunto
      mano + conector + tramo de cable que carga la muñeca. Anotar en `05_BITACORA.md`
      y contrastar con el valor del URDF (ver F2.1).
- [ ] Ruteo del cable Modbus/USB con `wrist_yaw` en sus dos extremos (±1.012 rad).
      Si el cable tensa, **reducir el tope blando de `wrist_yaw` en `gains.yaml`** antes
      de cualquier ensayo.
- [ ] Recalcular `τ_g` máximo en P3 con carga y verificar que el aborto al `0.7·τ_max`
      sigue teniendo margen. Con el brazo extendido, mano y objeto juntos aportan varios
      N·m en hombro y codo.

### F4.2 — Condiciones

| Condición | Descripción |
|---|---|
| **L0** | mano montada, sin objeto — **es la línea base ya medida** (sesiones 09-04 y 09-05) |
| L1 | mano sujetando 150 g |
| L2 | mano sujetando 300 g (límite superior del alcance de la tesis) |

L0 no se vuelve a medir salvo para el `--repeats` de F1.3. El objeto debe ir sujeto por
los dedos, no atado a la muñeca: la posición del centro de masa respecto a `wrist_pitch`
importa tanto como la masa.

Ejecutar en L1 y L2: barrido de comprobación (`04_sweep_arms.py`) de las 7 articulaciones
del brazo izquierdo, en P1 y P3, `--repeats 3`.

**Reportar:** `Δτ_g`, `Δrms_error` y `Δτ_pico` de L1 y L2 respecto a L0, por articulación.

**Criterio de aceptación:** las ganancias de F2/F3 siguen dando 7/7 dentro de tolerancia
en L2. Si `wrist_pitch` o `elbow` se degradan por encima de `δ_min`, re-sintonizar esas
dos en L2.

**Decisión sobre la condición de referencia:** si la degradación de L0 a L2 supera
`δ_min` en más de dos articulaciones, adoptar **L1 como condición de sintonización por
defecto** (carga intermedia, error acotado en los dos extremos) en vez de L0. Si no,
mantener L0, que es donde ya está toda la caracterización.

---

## F5 — Caracterización en frecuencia

**Por qué.** El chirp ya se usa (bitácora del 2026-09-05) pero de forma comparativa. Para
la tesis hace falta un número: ancho de banda a −3 dB por articulación.

**Script nuevo:** `12_bode.py`.

**Señal:** chirp logarítmico 0.2 → 5 Hz, con **amplitud escalada como
`A(f) = v_max / (2πf)`** para mantener velocidad de pico constante y no saturar en alta
frecuencia. Duración 90 s. `v_max` = 0.3 rad/s.

**Configuraciones:** solo 3 articulaciones (`shoulder_pitch`, `elbow`, `wrist_pitch`) ×
3 conjuntos (`xr_teleoperate`, `tuned_gff`, y `tuned_gff` con `dq_des = 0`). Nueve
corridas, no más.

**Análisis:** estimar `|Q(f)/Q_d(f)|` y la fase por promediado espectral (Welch,
segmentos con solape del 50 %).

**Reportar por configuración**

| Métrica | Valor |
|---|---|
| BW a −3 dB [Hz] | |
| Frecuencia con fase −90° [Hz] | |
| Pico de resonancia [dB] y su frecuencia | |
| Coherencia media en la banda 0.2–3 Hz | |

**Criterio de aceptación:** coherencia > 0.9 en 0.2–3 Hz. Por debajo de eso, la
estimación no es fiable (probablemente fricción o zona muerta) y hay que subir la
amplitud.

**Uso en la tesis:** esta tabla es la que justifica cuantitativamente que el brazo no
introduce dinámica relevante en la banda de la teleoperación (< 2–3 Hz) y que su tiempo
de respuesta es despreciable frente a los 64 ms de tiempo muerto del lazo de fuerza de
la mano.

---

## F6 — Validación multiarticulación

**Por qué.** Todo está medido con una articulación en movimiento y trece clavadas. La
teleoperación mueve las siete a la vez y el acoplamiento inercial no se ha probado nunca.

**Script nuevo:** `13_multi_joint.py`.

### Trayectorias

| ID | Descripción | Duración |
|---|---|---|
| T1 | min-jerk punto a punto, P1 → P2 → P3 → P1, las 7 a la vez | 12 s |
| T2 | senos desfasados: cada articulación a 0.5 Hz con fase `2πk/7` | 20 s |
| T3 | trayectoria de teleoperación grabada (reproducir un episodio real de `xr_teleoperate`) | 30 s |

`--repeats 5`, condición L2 (con carga de 300 g), postura y ganancias de F4.

**Reportar por articulación:** `rms_error`, `τ_pico`, `chatter_dq`, y la razón
`rms_multi / rms_single` respecto al valor de F1.3.

**Criterio de aceptación**
- `rms_multi / rms_single ≤ 1.5` en las 7.
- `chatter_dq` de las muñecas no sube por encima de 0.05 rad/s cuando arranca el hombro
  (esa es la firma del acoplamiento: la muñeca tiembla porque el hombro la sacude).
- Ningún par por encima del 60 % de `τ_max` en T3.

Si falla, el sospechoso principal es kd alto en los hombros excitando las muñecas.
Ensayo diagnóstico: repetir T1 con kd de hombro reducido a la mitad y ver si el
`chatter_dq` de muñeca cae.

---

## F7 — Cierre del lazo con la teleoperación

### F7.1 — Parche de velocidad de referencia (`dq_des`)

§5 mide un 35 % de error extra por mandar `dq = 0`, y la bitácora demuestra que **los kd
de `tuned` solo son válidos con `dq_des` activa**. Este parche no es opcional: es la
condición de validez del conjunto sintonizado.

- Derivar `dq_des` de la salida del IK con diferenciación filtrada (filtro de primer
  orden a ~10 Hz; sin filtro, el ruido del IK entra multiplicado por kd).
- Saturar `|dq_des| ≤ 2 rad/s`.
- Verificar en el ensayo canónico que `rms_error` reproduce el valor de F1.3.

**Plan B documentado:** si el parche no se puede aplicar, generar y activar un conjunto
`tuned_zerodq` con kd cercano a la referencia (3.0 en hombros y codo, 2.0 en muñecas),
sintonizado explícitamente con `--zero-dq`. Nunca desplegar `tuned` con `dq = 0`: con
kd = 13.5 el codo satura solo por frenado a 2 rad/s.

### F7.2 — Parche de compensación de gravedad

`H1_2_ArmController.ctrl_dual_arm(q, tauff)` ya acepta el par por articulación. Conectar
la salida de `11_gravity_model.py` a `tauff_target`.

**Verificación:** repetir T3 con y sin `τ_ff` y comparar `rms_error` de los dos
`shoulder_roll`. Esperado: reducción > 60 %.

### F7.3 — `arm_sdk` con el robot de pie

Es el único punto del protocolo que se ensaya con el robot no colgado, y por eso va al
final y con su propia secuencia.

> Las dos manos suman ~1.6 kg en los extremos de los brazos. Con el robot de pie, un
> movimiento de brazo desplaza el centro de masa más de lo que el controlador de
> locomoción esperaría de un H1-2 sin efectores. Es una razón adicional para escalar la
> amplitud despacio en el paso 5.

1. Con el robot **colgado**, verificar que `arm_sdk` responde tras el cambio de modo
   (`R1 + X` o equivalente en la versión de firmware instalada). §README documenta que
   `arm_sdk` no hacía nada con el robot en reposo; comprobar si eso cambia con el
   controlador de locomoción activo.
2. Rampa de la junta de peso 0 → 1 en 3 s, sin comandar movimiento. Observar 30 s.
3. Amplitud 0.05 rad en `L_wrist_yaw` únicamente.
4. Solo si 1–3 pasan: robot de pie, **arnés puesto pero con holgura**, repetir 2–3.
5. Escalar amplitud en pasos de 0.05 rad hasta 0.12, vigilando la reacción del
   controlador de locomoción (desplazamiento de pies, par en tobillos).

**Criterio de parada:** cualquier paso de tobillo no comandado, o par de tobillo por
encima del 40 % de su límite → abortar, bajar peso en rampa, volver a colgar el robot.

### F7.4 — Aceptación final

`04_sweep_arms.py --gains tuned_gff` en el modo de operación real (canal, postura y
carga definitivos), `--repeats 3`. 14/14 dentro de tolerancia. Congelar `gains.yaml` y
etiquetar el commit.

---

## F8 — Criterio de impedancia para manipulación en contacto

**Por qué esta fase existe.** Las fases anteriores optimizan seguimiento, y el criterio
`J = w_err·err + w_chatter·temblor + w_overshoot·sobreimpulso` premia rigidez. Para la
tesis eso es el criterio equivocado en la fase de contacto: un brazo rígido convierte
cualquier error de posición en fuerza de impacto sobre el objeto, y el lazo de fuerza de
la mano tiene 64 ms de tiempo muerto, así que no puede reaccionar al transitorio. La
regulación fina de fuerza debe quedar en los dedos; el brazo debe ser la etapa de baja
impedancia.

### F8.1 — Experimento: fuerza de impacto vs rigidez del brazo

Éste es un ensayo nuevo y, probablemente, el más publicable del conjunto.

**Montaje:** objeto rígido de referencia fijado a un soporte rígido en el espacio de
trabajo. La mano cerrada en una configuración fija (sin control de dedos activo), de modo
que el contacto lo produzca íntegramente el movimiento del brazo.

**Variables**

| Factor | Niveles |
|---|---|
| kp del brazo (hombro + codo, escalado conjunto) | 50, 100, 140, 280 |
| Velocidad cartesiana de aproximación | 0.05, 0.10, 0.20 m/s |

**Medida:** `F_pico` y la integral de fuerza en los primeros 200 ms, tomadas de
`FORCE_ACT` de la mano (~98 Hz) y del canal táctil como verificación de la zona de
contacto. `N ≥ 10` por celda, orden aleatorizado.

**Salida:** `F_pico` vs kp, agrupado por velocidad. Es el análogo directo, a nivel de
brazo, de la figura `ΔF` vs velocidad que ya tienes para la mano en la caracterización
dinámica del RH56DFTP.

**Hipótesis:** `F_pico` crece monótonamente con kp del brazo a velocidad constante, y el
efecto es comparable en magnitud al de la velocidad de cierre de los dedos.

**Seguridad:** techo de fuerza de la mano activo, `timeout` duro por ensayo, y aborto por
par de brazo al 50 % (no al 70 %) durante este experimento por tratarse de contacto
contra un soporte rígido.

### F8.2 — Conjunto dual de ganancias

Del resultado anterior se derivan dos conjuntos, y se documenta cuándo aplica cada uno:

| Conjunto | Uso | Criterio |
|---|---|---|
| `tuned_tracking` | movimiento en espacio libre, aproximación, retirada | el de F2–F4: mínimo error de seguimiento |
| `tuned_contact` | desde la detección de proximidad hasta la liberación | **el kp más bajo que mantenga el error de seguimiento por debajo de la tolerancia de la tarea**, no el más alto estable |

Definir la tolerancia de la tarea explícitamente y por adelantado (propuesta: 2° por
articulación, que a la distancia del efector son unos pocos milímetros). Conmutar entre
conjuntos con rampa de kp de 0.5 s: un cambio brusco de kp con error no nulo produce un
salto de par instantáneo.

---

## Reglas transversales (aplican a todas las fases)

1. **Nunca cambiar kp o kd con `q_des` desactualizado.** Antes de aplicar ganancias
   nuevas, escribir `q_des = q_medido`. Un cambio de ganancia con error de posición no
   nulo produce un salto de par proporcional a `Δkp · e`.
2. **Precalentamiento de 30 s** antes de toda medida (pendiente de confirmación en F1.1).
3. **Orden de celdas aleatorizado** y temperatura registrada al inicio y fin de cada
   bloque. Sin aleatorización, el calentamiento progresivo se confunde con el efecto de
   la ganancia.
4. **`--repeats ≥ 3`** en todo ensayo cuyo resultado vaya a informar una decisión.
   Reportar mediana e IQR, no media y desviación: la distribución no es simétrica cuando
   hay fricción estática.
5. **Ninguna diferencia menor que `δ_min` (F1.2) se reporta como efecto.**
6. **Metadatos por corrida** en `index.csv`: kp, kd, `τ_ff` on/off, `dq_des` on/off,
   postura, carga (L0/L1/L2), temperatura inicial, jitter p99 de la sesión, hash del
   commit, versión de firmware.
7. **Un hallazgo que contradiga la teoría es primero un fallo de medida.** Es lo que pasó
   con la métrica de sobreimpulso el 2026-09-05, y la regla vale la pena escribirla.

---

## Entregables

| Fase | Producto | Destino |
|---|---|---|
| F0 | Tabla de jitter y tasa efectiva | cabecera de `06_RESULTADOS.md` |
| F1 | `δ_min` por articulación; §7 revalidado con IQR | `04_METODOLOGIA.md` + `06_RESULTADOS.md` §8 |
| F2 | `11_gravity_model.py`; conjunto `tuned_gff`; tabla modelo vs medido | `gains.yaml`, `06_RESULTADOS.md` §9 |
| F3 | Tabla `kp*` × 3 posturas; decisión sobre *gain scheduling* | `06_RESULTADOS.md` §10 |
| F4 | Δ de L1 y L2 sobre L0; condición de referencia decidida | `06_RESULTADOS.md` §11 |
| F5 | `12_bode.py`; tabla de BW y fase | `06_RESULTADOS.md` §12 + capítulo de tesis |
| F6 | `13_multi_joint.py`; razón `rms_multi/rms_single` | `06_RESULTADOS.md` §13 |
| F7 | Parches de `dq_des` y `τ_ff` en `xr_teleoperate`; `arm_sdk` de pie | `05_BITACORA.md` + PR |
| F8 | `F_pico` vs kp del brazo; conjuntos `tuned_tracking` / `tuned_contact` | capítulo de tesis |

---

## Checklist de ejecución

**F0 — temporización**
- [ ] `10_loop_timing.py` escrito y corrido a 250 y 500 Hz
- [ ] `dt` p99 ≤ 1.2·T confirmado, o mitigado y re-medido
- [ ] Resultado en la cabecera de los informes

**F1 — estadística**
- [ ] Efecto frío/caliente cuantificado; precalentamiento adoptado si procede
- [ ] `δ_min` calculado en 3 articulaciones
- [ ] §7 revalidado con `--repeats 5`; mejoras marcadas como significativas o no

**F2 — gravedad**
- [ ] URDF con manos verificado (masa e inercia)
- [ ] Modelo validado contra los 4 puntos existentes + 12 nuevos
- [ ] Prueba de humo de `01_hold.py` con `τ_ff` y kp reducido
- [ ] Barrido de kp con `τ_ff`: ganador **no** en el borde
- [ ] Sobreimpulso de `shoulder_roll` por debajo del 10 %
- [ ] `tuned_gff` escrito; `tuned` marcado como deprecado

**F3 — postura**
- [ ] P2 y P3 verificadas contra topes blandos y espacio físico
- [ ] 4 articulaciones × 3 posturas sintonizadas
- [ ] Decisión conjunto único vs *gain scheduling* tomada y justificada

**F4 — carga sujetada**
- [ ] Masa contrastada con el URDF; ruteo de cable verificado en ambos extremos de `wrist_yaw`
- [ ] Margen de par recalculado en P3 con carga
- [ ] L1 y L2 caracterizadas; 7/7 en L2

**F5 — frecuencia**
- [ ] Chirp de amplitud escalada implementado
- [ ] Coherencia > 0.9 en 0.2–3 Hz
- [ ] Tabla de BW por configuración

**F6 — multiarticulación**
- [ ] T1, T2, T3 corridas en L2
- [ ] `rms_multi/rms_single ≤ 1.5` en las 7
- [ ] Sin acoplamiento hombro → muñeca

**F7 — teleoperación**
- [ ] `dq_des` parcheado y verificado, o `tuned_zerodq` generado
- [ ] `τ_ff` conectado a `tauff_target`
- [ ] `arm_sdk` de pie ensayado por la secuencia escalonada
- [ ] Aceptación final 14/14; `gains.yaml` congelado y etiquetado

**F8 — contacto**
- [ ] Montaje de objeto rígido y protocolo de aborto al 50 %
- [ ] `F_pico` vs kp × velocidad, N ≥ 10 por celda
- [ ] Tolerancia de tarea definida por adelantado
- [ ] `tuned_tracking` y `tuned_contact` documentados con su regla de conmutación
