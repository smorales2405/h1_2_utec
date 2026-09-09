# Traspaso a la máquina con `xr_teleoperate`

Qué está hecho, qué falta, y qué hace falta tener instalado para continuar.

---

## 1. Estado: todo está en `main`

| artefacto | dónde | ¿versionado? |
|---|---|:--:|
| Los 15 scripts y la librería | `h1_2_joint_control/` | sí |
| Ganancias, postura y topes | `config/gains.yaml` | sí |
| **Parámetros de masa identificados** | `logs/gravity_params_*.json` | **sí** |
| Datos crudos de la identificación | `logs/gravity_map_*.json`, `gravity_id_*.json` | sí |
| Métricas de los 300+ ensayos | `logs/index.csv` | sí |
| Parche de `dq_des` | `h1_2_teleoperation/patches/` | sí |
| Documentación completa | `docs/01..09` | sí |
| Trazas temporales de cada ensayo (138 CSV) | `logs/*.csv` | **no**, por convención del repo |

Los CSV crudos se quedan en la máquina `mito`. Todo lo demás —incluidos los
parámetros de masa, que son el artefacto que no se puede regenerar sin el
robot— está en el repositorio.

---

## 2. Qué hace falta en la otra máquina

### Para aplicar los parches y teleoperar

Solo `xr_teleoperate` y su entorno, que ya están.

### Para usar la compensación de gravedad (`gravity.py`)

Tres cosas, y ninguna es ROS:

1. **`pinocchio`** (probado con 3.8.0) y `numpy`.
2. **El URDF con las manos Inspire**, que **NO está en este repositorio**: viene
   de `github.com/oscar-ramos/h1_2_utec`, paquete `h1_2_description`, fichero
   `urdf/h1_2.urdf`. Clonarlo, y si no queda en una de las rutas habituales:
   ```bash
   export H12_URDF=/ruta/a/h1_2_description/urdf/h1_2.urdf
   ```
3. Los `logs/gravity_params_*.json` de este repositorio, que ya están.

`gravity.py` no importa nada de ROS, así que funciona en el entorno de
`xr_teleoperate` tal cual.

### Para volver a medir con `h1_2_joint_control`

Esto **no** funcionará en la otra máquina salvo que tenga `unitree_ros2`
compilado con los mensajes `unitree_hg`. Los dos caminos son distintos:

| | `h1_2_joint_control` | `xr_teleoperate` |
|---|---|---|
| capa | ROS 2 + `unitree_ros2` | `unitree_sdk2py` (DDS directo) |
| mensajes | `unitree_hg/LowCmd` | `unitree_hg::msg::dds_::LowCmd_` |

Es el mismo tráfico DDS y las ganancias son intercambiables, pero las
**herramientas de medida** requieren el camino ROS. Si esa máquina no lo tiene,
la sintonización nueva se sigue haciendo en `mito` y allí solo se despliega.

---

## 3. Lo que hay que hacer allí

### F7.1 — aplicar el parche de `dq_des` (listo, sin probar en el lazo)

```bash
cd xr_teleoperate
git apply ../h1_2_teleoperation/patches/xr_teleoperate_h1_2_dq_ref.patch
```

Detalle y justificación en
[`patches/README_dq_ref.md`](../../h1_2_teleoperation/patches/README_dq_ref.md).
Verificado que aplica limpio sobre el fuente de upstream y que el resultado es
Python válido, pero **no se ha ejecutado dentro del lazo de teleoperación**:
esta máquina no tiene `xr_teleoperate`.

Lo primero que hay que comprobar allí es que la teleoperación arranca y que el
brazo no vibra. Si vibra, el sospechoso es el ruido del IK entrando por `kd`:
bajar `dq_filter_hz` de 10 a 5 o 3 (la tabla del README dice lo que cuesta).

### F7.2 y F7.3 — HECHAS el 2026-09-09

Ambas están en `patches/xr_teleoperate_h1_2_tuning.patch`, que incluye
también el de `dq_des`. Ver [`README_tuning.md`](../../h1_2_teleoperation/patches/README_tuning.md).
Falta probarlo con el visor y el robot colgado, que necesita supervisión.

<details><summary>Cómo se planteó (ya no aplica)</summary>

### F7.2 — conectar la compensación de gravedad

`H1_2_ArmController.ctrl_dual_arm(q, tauff)` ya acepta el par por articulación
y la teleoperación le pasa ceros. Falta el puente:

```python
from h1_2_joint_control.gravity import GravityModel   # no necesita ROS
g = GravityModel()
# en el lazo, antes de ctrl_dual_arm:
tau = g.tau(q_27)                      # q de las 27, no solo los brazos
tauff = np.array([tau[i] for i in H1_2_JointArmIndex])
```

Cuesta 28 µs por evaluación. Vale el **85–98 % del error permanente** (§9 de
resultados). Cautelas: rampa de 1 s al activar para no meter un escalón de par,
y saturación a `0.5·tau_max` como red independiente.

</details>

<details><summary>F7.3, cómo se planteó (ya no aplica)</summary>

### F7.3 — ganancias por articulación

`xr_teleoperate` usa **cuatro constantes** para las catorce articulaciones
(`kp_low`/`kd_low` para hombros y codo, `kp_wrist`/`kd_wrist` para muñecas).
El conjunto `tuned_gff` es **por articulación** y va de kp 40 a 280 y kd 1.8 a
20.25, así que no se puede expresar con cuatro números. Hace falta un segundo
parche con una tabla.

**No está escrito.** Lo dejo así a propósito: es un parche que conviene
escribir con la teleoperación delante para poder probarlo, no a ciegas.

</details>

### F7.4 — aceptación

`04_sweep_arms.py --gains tuned_gff --gravity-ff` en el modo de operación real.
Requiere el camino ROS.

---

## 4. Qué NO desplegar sin leer esto

- **`tuned_gff` sin `--gravity-ff`.** Esos kp bajos funcionan porque el par de
  sostenimiento lo da el feedforward. Sin él, el error permanente vuelve a ser
  `tau_g/kp` y con kp = 70 en el hombro eso es mucho peor que con los 280 de
  `tuned`.
- **`tuned_gff` sin el parche de `dq_des`.** Los kd llegan a 20.25 y se
  sintonizaron **con** velocidad de referencia. Sin ella el orden entre
  candidatos se invierte (medido: kd 13.5 gana a kd 3 por un 31 % con `dq_des`,
  y pierde por un 32 % sin ella).
- **Los valores de `shoulder_pitch` y `wrist_yaw` de `tuned_gff`** difieren
  entre brazos (111 contra 280, y 40 contra 100) porque en esas dos el criterio
  es plano y el ganador lo decidió el ruido. Conviene elegir a mano el kp
  **bajo** de los dos por margen de par.

---

## 5. Encargo para la sesión de Claude Code en la otra máquina

Copiar tal cual:

> Contexto: repositorio `smorales2405/h1_2_utec`, rama `main`. La sintonización
> articular del H1-2 está hecha y documentada en `h1_2_joint_control/docs/`.
> Lee primero `docs/09_TRASPASO.md`, y luego `docs/06_RESULTADOS.md` secciones
> 9 a 13, que son los resultados vigentes.
>
> Esta máquina tiene `xr_teleoperate` instalado; la máquina donde se hizo la
> sintonización no lo tenía, así que el parche de `dq_des`
> (`h1_2_teleoperation/patches/xr_teleoperate_h1_2_dq_ref.patch`) está escrito y
> verificado sintácticamente pero **nunca se ha ejecutado dentro del lazo**.
>
> Tareas, en orden:
> 1. Aplicar el parche de `dq_des` y comprobar que la teleoperación arranca y
>    que los brazos no vibran. Si vibran, bajar `dq_filter_hz`.
> 2. Conectar `h1_2_joint_control/gravity.py` a `tauff_target` de
>    `H1_2_ArmController` (F7.2 del traspaso). Necesita `pinocchio` y el URDF de
>    `oscar-ramos/h1_2_utec`; si no está, `export H12_URDF=...`.
> 3. Escribir el parche de ganancias por articulación para llevar `tuned_gff`
>    de `config/gains.yaml` a `xr_teleoperate`, que hoy usa cuatro constantes
>    para catorce articulaciones.
>
> Precondiciones de seguridad: robot colgado del arnés, nadie al alcance, mando
> a mano, paro `L2 + B`. Topes de autocolisión en `config/gains.yaml`
> (`shoulder_roll_vs_elbow_deg`): con el codo flexionado el hombro no baja de
> ±5°, con el brazo estirado no baja de ±10°, y ±15° si se mueven las muñecas.
>
> Una advertencia de método, aprendida a base de tropezar: en este montaje han
> aparecido cuatro resultados que parecían físicos y eran fallos de medida.
> Antes de creerse cualquier número que contradiga la teoría, comprobar el
> instrumento.
