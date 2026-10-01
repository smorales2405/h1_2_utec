# Capacitación H1-2 (Robotics 4.0): lo que hay que saber antes de tocar código

Resumen de la capacitación técnica *Robot humanoide Unitree H1-2 — Operación segura, control por SDK y
desarrollo* (Robotics 4.0, R40-CAP-H1_2-0001 v1.10, septiembre de 2026, 111 láminas, 18 módulos),
contrastado con el código que se usó en ella (`code_cap`). Sirve de referencia antes de escribir código o
modificar archivos que comanden al robot, en simulación (MuJoCo) o en el robot real.

**Criterio:** cuando la web de Unitree y el SDK/URDF se contradicen, mandan **el SDK y el URDF**. Unitree
cambia la documentación a menudo: verificar siempre contra el firmware de nuestra unidad.

Fuentes al final del documento.

---

## 0. Checklist antes de escribir o modificar código de control

1. **Mensajes `unitree_hg`**, nunca `unitree_go` ni `unitree/idl/go2/` (eso es del H1 original). 27 juntas,
   arrays de 35 slots. → §1
2. **Interfaz y dominio DDS parametrizables:** `(1, "lo")` contra el simulador, `(0, "eth0")` en el robot.
   Nunca vacío ni fijo en el código. → §4
3. **Canal correcto:**
   - robot de pie, solo brazos/cintura → `rt/arm_sdk` (peso en el slot 27, FSM 201, rampas lentas) → §8
   - cuerpo entero → `rt/lowcmd`, **solo** con Debug real (`CheckMode` vacío + 3 s sin publicadores +
     confirmación) y robot colgado → §3, §6
   - marcha → `LocoClient` con `Move` de 1 s → §9
   - alto y bajo nivel **no se mezclan**.
4. **LowCmd:** `mode_pr = 0`, `mode_machine` copiado del LowState, `mode = 1`, **CRC al final**, 500 Hz. → §6
5. **Primer movimiento interpolado desde la posición medida** (≥ 3 s); perfiles suaves; cada paso parte de
   lo último mandado.
6. **Límites:** recortar a la intersección web ∩ URDF con 0.05 rad de margen; ojo con el signo espejado
   izquierda/derecha. → §5
7. **Watchdogs:** `motorstate ≠ 0` o `rt/lowstate` > 0.5 s sin llegar → amortiguación automática;
   inclinación excesiva → abortar; salida con el hilo parado y rampa de kp a 0.
8. **Confirmación tecleada** antes de mover (una palabra explícita) y aviso de `L2 + B`.
9. **Registro** CSV + JSON por ensayo.
10. **Probar primero en `unitree_mujoco` (C++)**, sabiendo lo que la simulación **no** cubre: modo AB,
    `rt/arm_sdk`, Debug/`CheckMode`, marcha, manos, `mode_machine = 0`. → §10
11. En el robot: intérprete Python explícito, `PYTHONNOUSERSITE=1`, `unset CYCLONEDDS_URI`. → §4

> En el H1-2 **muchos fallos no generan error**. Verificar el efecto, no el código de retorno.

---

## 1. H1 y H1-2 no son el mismo robot

Casi todo el material de internet sobre «el H1» es del H1 original. Copiar ese código en un H1-2 falla
justo en estos puntos:

| Aspecto | H1 | H1-2 |
|---|---|---|
| Grados de libertad | 19 | **27** (6+6 piernas, 1 cintura, 7+7 brazos) |
| Tobillo | 1 DOF en serie | **2 DOF paralelo (A/B)**, modos PR/AB |
| Peso | ≈47 kg | **≈70 kg** |
| Espacio de nombres IDL | `unitree_go` (idl/go2/) | **`unitree_hg`** (idl/hg/) |
| Tamaño de `motor_cmd[]` | 20 | **35 (se usan 27)** |
| Peso de `rt/arm_sdk` | índice 9 | **índice 27** (escribir en el 9 mueve la rodilla derecha) |
| `mode_machine` | no existe | 4 o 6, **se lee** del LowState (la nuestra: 6) |
| PC de desarrollo | .162 / .163 | 192.168.123.164 |
| LiDAR | 192.168.123.120 | 192.168.124.20 |
| Orden del mensaje = orden del URDF | no | **sí** |
| Sentarse / bailar / escaleras | sí / sí / — | **no / no / no** |

- Ejemplos del SDK válidos para el H1-2: `example/h1_2/low_level/h1_2_low_level_example.py` (Python);
  `h1_27dof_example`, `h1_2_ankle_track`, `h1_2_arm_sdk_dds_example` (C++). **No sirven:**
  `example/h1/low_level/*`, `h1_arm_sdk_dds_example`, `humanoid.cpp`.
- El alto nivel no tiene versión propia del H1-2: se usa el `LocoClient` del H1. → §9
- Las rutinas del **G1 de 23 DoF** usan índices (12, 15-19, 22-26) que caen dentro del rango del H1-2 pero
  son **otras juntas**. Por eso las rutinas JSON llevan `"robot": "unitree_h1_2"`.

**Datos del robot:** altura (1503+285) mm, marcha < 2 m/s, motor M107 hasta 360 N·m, batería 15 Ah
(0.864 kWh, 67.2 V máx.). Dos ordenadores: PC1 de control (cerrado) y PC4 de desarrollo. Sensores: LiDAR
MID-360 y RealSense D435(i) en la cabeza.

---

## 2. Seguridad y operación

### Las diez reglas no negociables

1. Robot **colgado** del soporte, con las cuatro ruedas bloqueadas, antes de cualquier ejemplo de bajo nivel.
2. **`L2 + B` es la parada de emergencia.** Pone el robot en amortiguación: deja de sostenerse y **cae
   lentamente**. Es una caída controlada, no un freno. Mando siempre en la mano.
3. Nunca hacer hot-swap de los conectores de aviación (la avería no la cubre la garantía).
4. Modo Debug (`L2 + R2`) antes del SDK de bajo nivel, y **verificarlo con `CheckMode`**. → §3
5. Interpolar siempre desde la posición **medida** hasta la deseada.
6. Vigilar `motor_state[i].motorstate`: si no es 0, amortiguación. **Lo hace el programa, no el operador.**
7. El robot no sube escaleras.
8. `HighStand()` y `SetBalanceMode(2)` comprometen la estabilidad.
9. El movimiento continuo (`continous_move=True`, `SwitchMoveMode(true)`) deja la orden vigente
   **864 000 s (10 días)** en lugar de 1 s. No usar.
10. Probar en `unitree_mujoco` antes de tocar el robot real.

### La postura de encendido fija el cero de las juntas

Es la causa principal de fallos. El cero del tren superior y de los tobillos se fija en la postura que
tengan al encender. Antes de pulsar:

- brazos aducidos hasta el tope y luego bajados en vertical;
- antepié levantado hasta el tope en los dos pies;
- robot colgado del soporte, sin tocar el suelo.

Si el robot anda torcido o una junta parece desplazada: **apagar, recolocar y volver a encender. No
compensarlo por software.**

### Arranque y apagado

| Paso | Qué se hace |
|---|---|
| 1. Colgar | Robot en el soporte, cuatro ruedas bloqueadas |
| 2. Baterías | Las dos, por el lateral, hasta oír el clic |
| 3. Postura de cero | Brazos aducidos y bajados, antepié al tope |
| 4. Encender | Pulsación corta y luego mantener > 2 s |
| 5. Arranque completo | ≈120 s (suena el tobillo contra el tope); esperar 30 s; `L2 + B` y luego `L2 + UP` |
| 6. Bajar la cuerda | Hasta que apoyen los dos pies; `R2 + X`: da unos pasos y se equilibra |
| 7. Soltar el gancho | Solo con el movimiento estable; se pilota con los joysticks |

**Apagado:** volver a enganchar el robot, `L2 + B` y mantener pulsados a la vez los dos botones de batería.

### Lista de comprobación de cada sesión

| Antes de encender | Antes de ejecutar código |
|---|---|
| Soporte con las cuatro ruedas bloqueadas | Robot colgado y sin tocar el suelo |
| Zona despejada | `L2 + B` probado y funcionando |
| Mando encendido y en la mano del operador | Si es bajo nivel: Debug confirmado (`L2 + A` da la postura de diagnóstico, y `CheckMode` vacío) |
| Brazos y antepié en la postura de cero | Programa probado antes en `unitree_mujoco` |
| Baterías insertadas (clic) y cargadas | Interfaz de red correcta en el argumento del programa |
| Nadie dentro del radio de los brazos | Un solo operador con el mando; el resto, fuera |

**Protección automática:** si el tronco se inclina demasiado o cadera y rodilla llegan al tope mecánico,
el robot pasa solo a amortiguación: se cae.

### Calibración (app Unitree Explore)

- El robot sale calibrado de fábrica. **No se recalibra** en el uso normal: cambia el cero de todas las
  juntas y con él todo el control de marcha. Solo si no consigue ponerse de pie tras reiniciar con la
  postura de encendido correcta. Tras calibrar: *Offset Configuration → Reset* y reiniciar.
- Offset por motor: entre −10 y +10, unidad no indicada. Pasos pequeños, un motor cada vez, robot colgado,
  y anotar cada cambio en la bitácora. Los números *Motor-N* son los índices de §5.
- La calibración de la IMU no se usa: Unitree no publica el procedimiento para el H1-2.

**Regla general:** el robot pesa 70 kg y no detecta a las personas. Colgado, con espacio libre, con el mando
en la mano y con una sola persona al cargo. Nada que mueva el robot sin el visto bueno del responsable del
laboratorio.

---

## 3. Mando, modos y FSM

### Combinaciones del mando

| Combinación | Efecto | Combinación | Efecto |
|---|---|---|---|
| `L2 + B` | Amortiguación: parada de emergencia | Joystick izquierdo | Velocidad vx, vy |
| `L2 + R2` | Modo Debug / Develop | Joystick derecho | Velocidad de guiñada ωyaw |
| `L2 + A` | En Debug: postura de diagnóstico | `X` / `Y` | Bajar / subir altura de pie |
| `L2 + UP` | Preparado (en 5 s); en nuestro H1-2 deja **FSM 201** | `A` / `B` | Bajar / subir elevación de pierna |
| `L2 + Y` | Par cero (desde amortiguación) | `START` | Alternar de pie / andando |
| `R2 + X` | Entrar en modo movimiento | `SELECT + Y` / `SELECT + A` | Saludar / dar la mano |

- El diagrama de estados oficial es del H1: en el H1-2 **no existen** Seating, Dance, `L2 + LEFT` ni `L1 + Y`.
- **El mando tiene prioridad sobre el programa:** si un eje del joystick no está a cero, manda el joystick.
- Prioridades: la amortiguación gana a todo salvo a Develop. Se entra en Develop automáticamente cuando el
  servicio de movimiento termina, normal o anormalmente.
- El mando se lee por software: 40 bytes en `rt/lowstate.wireless_remote[40]` (unión de bits R1, L1, start,
  select, R2, L2, F1, F2, A, B, X, Y, up, right, down, left; joysticks lx, ly, rx, ry en [−1, 1]). Sirve para
  implementar una parada propia dentro del bucle de control.

### FSM

- Documentación: 0 par cero, 1 amortiguación, 2 de pie bloqueado, 204 control principal, 205 adaptación de
  carga.
- **En nuestro firmware se ven 1, 3 y 201; la marcha es FSM 201.** La tabla de la web no cubre este firmware.
- `StandUp()`, `Start()` y `SetFsmId()` **devuelven 0 y no hacen nada**: la FSM solo la cambia el mando.

### Debug de verdad = `CheckMode` con `name` vacío

| Lo que se ve | Qué significa |
|---|---|
| `rt/lowcmd` a ~500 Hz | Manda el control interno (FSM 201; kp 400 en rodillas, 600 en cintura). Usar `rt/arm_sdk` |
| `rt/lowcmd` sin publicar y `CheckMode = 'ai'` | **No es Debug:** tus `rt/lowcmd` se ignoran **sin error** |
| `rt/lowcmd` sin publicar y `CheckMode = ''` | Debug: obedece `rt/lowcmd`; **las piernas no se sostienen** |

- Que nadie publique en `rt/lowcmd` **no** prueba que haya Debug. Caso real (25-09-2026): selector en FSM 0,
  `rt/lowcmd` en silencio, `CheckMode` = `'ai'` → los brazos no se movieron.
- `ReleaseMode()` sí pone Debug en el H1-2 y **suelta las piernas**: robot colgado. Limitarlo a 3 intentos y
  tras una confirmación tecleada (el ejemplo oficial lo repite sin fin).
- **Regla para cualquier programa de bajo nivel:** `CheckMode` con `name` vacío (o `ReleaseMode()`
  confirmado) → 3 s escuchando `rt/lowcmd` sin que nadie publique → confirmación tecleada → solo entonces se
  crea el publicador.
- **Alto y bajo nivel son excluyentes:** en Debug el `LocoClient` no existe (todas sus llamadas devuelven
  3104).

```python
status, result = msc.CheckMode()          # MotionSwitcherClient
if status == 0 and not result.get("name"):
    return True                           # Debug confirmado
if input("Escribe SOLTAR ...") != "SOLTAR":
    return False
for n in range(3):                        # número de intentos acotado
    msc.ReleaseMode(); time.sleep(1.0)
    status, result = msc.CheckMode()
    if status == 0 and not result.get("name"):
        return True
return False
```

---

## 4. Red y entorno

### Direcciones

| Equipo | Dirección |
|---|---|
| PC1, control de movimiento (cerrado: no se desarrolla ahí) | 192.168.123.161 |
| PC4, desarrollo (cable) | 192.168.123.164 |
| PC4 por WiFi `UTEC_H1_2` | 192.168.0.143 |
| LiDAR MID-360 | 192.168.124.20 |
| Mano Inspire **izquierda** | 192.168.124.**211** |
| Mano Inspire **derecha** | 192.168.124.**210** |
| PC propio por cable | 192.168.123.99/24 |

- Conexión física: cable al puerto de depuración del lateral derecho, con el adaptador 8+2. Comprobar con
  `ping 192.168.123.161` y `ping 192.168.123.164`.
- SSH: `unitree@192.168.0.143` (WiFi) o `unitree@192.168.123.164` (cable). Para datos a 500 Hz o vídeo,
  cable. Por WiFi: ~7 ms de media con picos de medio segundo.
- Las credenciales de fábrica están publicadas: cambiarlas.

### DDS

- CycloneDDS 0.10.2. Kits: `unitree_sdk2` (C++), `unitree_sdk2_python`, `unitree_sdk2_rs`, `unitree_ros2`.
- **El DDS de Unitree solo existe en la `eth0` del robot (192.168.123.x) y no atraviesa routers.** Desde un
  PC por WiFi, los programas DDS se ejecutan **en el robot** (los lanzadores de `code_cap` se reenvían por
  `ssh -t` y ejecutan el código del robot, no el del PC).
- `ChannelFactoryInitialize(0, "eth0")` en el robot; **`ChannelFactoryInitialize(1, "lo")` contra el
  simulador**. Pasar siempre la interfaz: sin ella el SDK elige sola (y el ejemplo C++ del `LocoClient` usa
  `lo` por defecto: no ve el robot).
- `Init` una sola vez, antes de crear cualquier canal. Si el callback tarda, usar cola (`queuelen > 0`).
  `GetLastDataAvailableTime()` sirve para detectar que el robot dejó de publicar.
- `enableSharedMemory = false` si se desarrolla fuera del robot.

| Tópico | Para qué |
|---|---|
| `rt/lowstate` | Estado de los 27 motores, IMU y mando (500 Hz) |
| `rt/lowcmd` | Control de bajo nivel (exige Debug) |
| `rt/arm_sdk` | Solo brazos y cintura, sin salir de la marcha |
| `rt/inspire_hand/{ctrl,state,touch}/{l,r}` | Manos diestras |
| `rt/lf/mainboardstate` | Fallos de la placa base |

### Entorno del PC de desarrollo del robot

- **Tres Python distintos.** El `python3` del sistema importa un `unitree_sdk2py` **sin `h1`** (el de
  `inspire_hand_ws`); `~/unitree_sdk2_python/.venv/bin/python` tiene `LocoClient`; `~/teleop_venv/bin/python`
  tiene además pymodbus, pyrealsense2 y flask. Regla: **cada programa con su intérprete explícito**, nunca
  depender del `python3` del sistema ni de `~/.local`.
- `export PYTHONNOUSERSITE=1` siempre (`~/.local` se cuela en cualquier Python 3.10).
- El `.bashrc` del robot exporta un `CYCLONEDDS_URI` del SLAM de fábrica que **rompe el DDS del SDK**:
  `unset CYCLONEDDS_URI RMW_IMPLEMENTATION`.
- Compilar en el propio robot evita problemas de ABI. En C++, enlazar contra `/opt/unitree_robotics` (el
  `unitree_sdk2Config.cmake` del `build/` del SDK no sirve para `find_package`).

### ROS 2

- Foxy en Ubuntu 20.04, Humble en 22.04.
- Compilar `cyclonedds` **sin** el entorno de ROS 2 cargado; después, con él. Desactivar conda antes de
  `colcon` (si no: `No module named catkin_pkg`).
- `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y un `CYCLONEDDS_URI` con la tarjeta correcta. Si
  `ros2 topic list` sale vacío, casi siempre es la tarjeta equivocada o falta el RMW.
- Para el H1-2 hacen falta los mensajes `unitree_hg` (ejemplo: `example/src/src/h1-2/lowlevel/low_level_ctrl_hg`).
- Visualización desde un PC por WiFi: el robot reenvía `rt/lowstate` por UDP 47927 (30 Hz, 364 B) y la nube
  del LiDAR por 47928; en rviz2, marco fijo `pelvis`. El LiDAR está montado invertido: rpy = (π, 0.300, 0).

---

## 5. Juntas, índices y límites

### Los 27 índices

El orden del array DDS **coincide con el del URDF**, sin huecos y sin intercalar (a diferencia del G1):

| # | Junta | # | Junta |
|---|---|---|---|
| 0-2 | LeftHip Yaw / Pitch / Roll | 13-15 | LeftShoulder Pitch / Roll / Yaw |
| 3 | LeftKnee | 16 | LeftElbow |
| 4 | LeftAnklePitch (= B en modo AB) | 17 | LeftWristRoll |
| 5 | LeftAnkleRoll (= A en modo AB) | 18 | LeftWristPitch |
| 6-8 | RightHip Yaw / Pitch / Roll | 19 | LeftWristYaw |
| 9 | RightKnee | 20-22 | RightShoulder Pitch / Roll / Yaw |
| 10 | RightAnklePitch (= B) | 23 | RightElbow |
| 11 | RightAnkleRoll (= A) | 24-26 | RightWrist Roll / Pitch / Yaw |
| 12 | WaistYaw (torso) | **27** | **Peso de `rt/arm_sdk`** (no es una junta) |

- 28-34: sin uso (el `LowCmd_` de `unitree_hg` tiene 35 slots).
- 17 y 24 son **wrist_roll**: la web de Unitree a veces los llama *elbow_roll* o *wrist_yaw*. Manda el SDK.

### Pares y carga

| Junta | Par límite | | |
|---|---|---|---|
| Rodilla | ≈360 N·m | Carga de brazo nominal | ≈7 kg |
| Cadera / cintura | ≈220 N·m | Carga de brazo pico | ≈21 kg |
| Tobillo | ≈75×2 N·m | | |
| Hombro / codo | ≈120 N·m | | |
| Muñeca | ≈30 N·m | | |

### Límites: la web y el URDF no coinciden (rad)

| Junta | Web (About_H1-2) | URDF |
|---|---|---|
| knee | −0.26 ~ 2.05 | −0.12 ~ 2.19 |
| torso | −3.14 ~ 1.57 | −2.35 ~ 2.35 |
| elbow (izq.) | −2.53 ~ 1.6 | −0.95 ~ 3.18 |
| wrist_roll | −2.967 ~ 2.967 | −3.01 ~ 2.75 |
| wrist_pitch | −0.471 ~ 0.349 | −0.4625 ~ 0.4625 |
| wrist_yaw | −1.012 ~ 1.012 | −1.27 ~ 1.27 |
| hip_yaw | −0.43 ~ 0.43 | igual |
| ankle_pitch | −0.897 ~ 0.524 | igual |
| ankle_roll | −0.262 ~ 0.262 | igual |

**Criterio:** el URDF es la fuente de verdad para simulación y planificación; en el robot real, mantener
margen respecto al **más restrictivo** de los dos. Los límites reales dependen del firmware y de si lleva
manos: verificarlos en la unidad antes de usar el rango completo.

Límites que usa el selector de poses del robot real (intersección web ∩ URDF; se aplican con
**0.05 rad de margen**):

| Índice | Izquierda | Índice | Derecha |
|---|---|---|---|
| 12 torso | (−2.35, 1.57) | | |
| 13 shoulder pitch | (−3.14, 1.57) | 20 | (−3.14, 1.57) |
| 14 shoulder roll | (−0.38, 3.40) | 21 | (−3.40, 0.38) |
| 15 shoulder yaw | (−2.66, 2.66) | 22 | (−2.66, 2.66) |
| 16 elbow | (−0.95, 1.60) | 23 | (−0.95, 1.60) |
| 17 wrist roll | (−2.967, 2.75) | 24 | (−2.75, 2.967) |
| 18 wrist pitch | (−0.4625, 0.349) | 25 | (−0.4625, 0.349) |
| 19 wrist yaw | (−1.012, 1.012) | 26 | (−1.012, 1.012) |

**Simetría:** los límites izquierda/derecha tienen el signo invertido en hombro (p/r/y), codo, hip_pitch y
hip_roll. **Copiar una pose de un brazo al otro no es copiar el vector** (el editor de poses espeja
cambiando el signo de roll y yaw de hombro y muñeca).

### Ejes e IMU

- Con todas las juntas a cero: X adelante, Y a la izquierda, Z arriba (regla de la mano derecha).
- El servicio de movimiento usa el mismo convenio: vx > 0 avanza, vy > 0 a la izquierda, ωyaw > 0 antihorario.
- **La IMU está en el torso, no en la pelvis.** Si gira la cintura (índice 12), hay que compensar esa
  rotación para obtener la orientación de la pelvis. Cuaternión en orden (w, x, y, z).

---

## 6. Control de bajo nivel (`rt/lowcmd`)

Escritura directa en los 27 motores, **sin las protecciones del control embarcado**. Exige Debug real (§3)
y robot colgado.

### Anatomía de un `LowCmd_` válido

```cpp
cmd.mode_pr()      = 0;              // 0 = PR
cmd.mode_machine() = mode_machine_;  // LEÍDO de rt/lowstate
for (int i = 0; i < 27; ++i) {
  cmd.motor_cmd().at(i).mode() = 1;  // 1 = enable
  cmd.motor_cmd().at(i).q()    = q_target[i];
  cmd.motor_cmd().at(i).dq()   = dq_target[i];
  cmd.motor_cmd().at(i).tau()  = tau_ff[i];
  cmd.motor_cmd().at(i).kp()   = kp[i];
  cmd.motor_cmd().at(i).kd()   = kd[i];
}
// SIN ESTA LÍNEA EL MOTOR IGNORA EL COMANDO
cmd.crc() = Crc32Core((uint32_t*)&cmd, (sizeof(cmd) >> 2) - 1);
publisher->Write(cmd);
```

En Python: `cmd.crc = CRC().Crc(cmd)`, **siempre al final**, después de rellenar todo.

Par aplicado: **T = kp (q − q_m) + kd (q̇ − q̇_m) + τ**.

**Los cuatro errores más frecuentes:**
1. Olvidar el CRC32: el motor ignora el comando y no se produce ningún error.
2. Fijar `mode_machine` a 4: funciona en unos H1-2 y en otros no. Se lee del LowState (la nuestra vale 6; en
   simulación, 0).
3. No entrar en Debug: el robot vibra por conflicto de comandos con el control interno.
4. Escribir el objetivo sin interpolar con kp alta: movimiento brusco.

### Ciclo y ganancias

- Ciclo de **2 ms (500 Hz)**: `RecurrentThread(interval=0.002)`. Un hilo de control y otro de escritura.
- Python no da tiempo real: va bien para lazos de ~50 Hz; para 500 Hz robusto, C++.

| Reductora | Dónde | kp | kd |
|---|---|---|---|
| S | Tobillos y brazos | 80 | 2 |
| M | Caderas y cintura | 100 | 3 |
| L | Rodillas | 200 | 5 |

Son las de `h1_27dof_example` y las que usa `code_cap`:

```python
Kp = [100, 100, 100, 200, 80, 80] * 2 + [100] + [80] * 14
Kd = [3, 3, 3, 5, 2, 2] * 2 + [3] + [2] * 14
```

(El ejemplo oficial en Python usa kp 100 en 0-12, 50 en los brazos y kd 1.)

### Arranque sin movimientos bruscos

```cpp
double ratio = std::clamp(time_ / duration_, 0.0, 1.0);
q_target[i] = (q_objetivo[i] - q_medida[i]) * ratio + q_medida[i];
```

- Con `duration_` de unos 3 s el robot llega sin sobresaltos. **Nunca escribir el objetivo final en el
  primer ciclo.** El selector impone un primer movimiento de al menos 3 s.
- Perfiles usados en `code_cap`: coseno `(1 − cos(π t/T)) / 2` y mínima sacudida `s = 10x³ − 15x⁴ + 6x⁵`
  (velocidad cero en los extremos). Cada paso parte de lo **último mandado**, no de lo medido, para que la
  consigna sea continua.
- Es un PD puro, sin compensación de gravedad: con el brazo en alto el hombro cede ~0.15 rad.

### Vigilancia obligatoria dentro del bucle

- `motorstate ≠ 0` en cualquier junta → amortiguación (kp = 0, se mantiene kd) **desde el programa**.
- `rt/lowstate` más de 0.5 s sin llegar → amortiguación.
- Verificar el CRC del estado recibido (`low_state CRC Error` ≈ versión del SDK distinta de la del firmware).
- Salida controlada: **primero parar el hilo** y después mandar la última postura (que no escriban dos a la
  vez); luego rampa de kp hasta 0 en ~2 s.
- El ejemplo oficial `h1_2_low_level_example.py` **no hace nada de esto**: repite `ReleaseMode()` sin
  límite, no vigila `motorstate`, no comprueba el CRC del estado, y con Ctrl-C deja de publicar sin pasar a
  amortiguación. Además lleva las 27 juntas (piernas incluidas) a cero: solo colgado.

### Qué trae el `LowState_`

- `motor_state[35]`: q, q̇, q̈, par estimado, dos temperaturas, tensión, `motorstate` (código de fallo) y
  `reserve`.
- `imu_state`: cuaternión (w, x, y, z), giróscopo, acelerómetro y rpy.
- `wireless_remote[40]`, `mode_pr`, `mode_machine`.
- **No trae BMS ni fuerza de pie** (a diferencia del H1). `rt/lf/bmsstate` llega todo a cero desde el PC de
  desarrollo: la batería no se lee desde ahí.

---

## 7. El tobillo paralelo

Cuatro juntas en serie (pitch/roll × 2), dos motores por tobillo (A y B).

- **Modo PR — `mode_pr = 0` (por defecto, el que hay que usar):** se escribe en 4/5 (izquierda) y 10/11
  (derecha) como pitch y roll normales; el robot convierte internamente a A y B.
- **Modo AB — `mode_pr = 1`:** se controlan los motores directamente y los mismos índices cambian de
  significado, **con cruce**: 4 = LeftAnkle**B** (antes pitch), 5 = LeftAnkle**A** (antes roll).
- El cambio PR ↔ AB se puede hacer en funcionamiento, pero con las juntas cerca de cero: el significado de
  `q` cambia al instante.
- Rango: pitch −0.897 ~ +0.524 rad, roll ±0.262 rad. Las amplitudes de prueba son pequeñas (±0.25 rad); los
  valores de los ejemplos del G1 tocarían el tope.
- `h1_2_ankle_track` **no** es un ejemplo de tobillos: su primera etapa manda las 27 juntas a cero durante
  3 s. Solo colgado, en Debug y con supervisión.
- Medido andando (FSM 201): recorridos de 17-20° en pitch y solo 4.4° en roll, a 1.43 Hz de cadencia.

---

## 8. Brazos sin detener la marcha (`rt/arm_sdk`)

Se publica un `LowCmd_` en `rt/arm_sdk`. Del array solo se atienden:

| Índice | Significado |
|---|---|
| 12 | Cintura (WaistYaw), opcional |
| 13-19 | Brazo izquierdo |
| 20-26 | Brazo derecho |
| **27** | **Peso de mezcla** `motor_cmd[27].q` ∈ [0, 1] |

- El peso mezcla la postura del control de Unitree (0) con la que tú mandas (1). Subirlo despacio de 0 a 1
  para tomar el control; bajarlo de 1 a 0 para devolverlo. Cuanto más rápido cambia, más brusca la
  transición. Lleva CRC y `mode_machine` igual que `rt/lowcmd`.
- **No hace falta Debug.** Hace falta el control interno activo: **FSM 201** (o 204) y el control publicando
  `rt/lowcmd`. **En Damp se ignora sin error.** Funciona con `CheckMode = 'ai'`. Es la vía correcta para
  gestos, manipulación ligera y demostraciones con el robot de pie.

### El control de marcha reacciona a los brazos

Con rampa y movimiento de 5 s (como el ejemplo oficial, que solo sirve colgado) el robot dio pasos y giró
+35° y −28°. **Configuración validada (0 pasos, < 0.7° de giro):**

- Rampa del peso 0 → 1 en **12 s**, sosteniendo la postura medida pero sin separarse más de **0.035 rad
  (2°)** de la inicial: `obj[j] = q_ini[j] + clamp(q_med[j] − q_ini[j], ±0.035)`. Copiarla sin tope hizo
  caer el codo 18°.
- Ir a la pose en **30 s con perfil de mínima sacudida**.
- **Brazo derecho atrás** como contrapeso del centro de masas.
- Salida 1 → 0 en 5 s.
- Publicación a **50 Hz**. `KP_BRAZO = 100` (aun así el brazo se queda ~5° corto); torso kp 200. El ejemplo
  oficial usa kp 60 / kd 1.5 en brazos.
- Comprobar que tiene efecto (el brazo sigue la consigna, error < 0.20 rad) antes de confiar en él. Abortar
  si la inclinación pasa de 10° (prueba de brazos) o 20° (caja), soltando el brazo en 2 s.
- No corregir el rumbo con `Move` mientras se manipula: mantiene al robot dando pasos.

`unitree_mujoco` **no implementa `rt/arm_sdk`**: en simulación se valida el mismo recorrido del brazo por
`rt/lowcmd` de cuerpo entero con las piernas sostenidas (`caja_cuadrado.py --sim`). → §10

---

## 9. Alto nivel (`LocoClient`)

- No hay cliente propio del H1-2: C++ `#include <unitree/robot/h1/loco/h1_loco_client.hpp>`
  (`unitree::robot::h1::LocoClient`); Python `unitree_sdk2py.h1.loco` (solo en el `.venv` del SDK). Servicio
  RPC `loco`, API 2.0.0.0, parámetros JSON `{"velocity": [vx, vy, w], "duration": 1}`.
- **`SetTimeout(10.0)`**: el defecto de 1 s se queda corto.
- **Cualquier `Move` exige el robot de pie en FSM 201** (`L2 + UP`), espacio libre y `L2 + B` a mano.
- **`Move(vx, vy, vyaw)` va con `duration = 1 s`:** si el programa o el SSH se cortan, el robot para solo en
  1 s. Es la parada de seguridad: reenviar la orden a 10-20 Hz. **Nunca** `continous_move=True` ni
  `SwitchMoveMode(true)`.

| Magnitud | Rango (el robot satura sin error) |
|---|---|
| vx | [−0.8, 1.0] m/s |
| vy | [−0.5, 0.5] m/s |
| ωyaw | [−0.5, 0.5] rad/s (por debajo de ~0.15 apenas gira) |

- Funciones peligrosas: `HighStand()` (degrada velocidad y estabilidad) y `SetBalanceMode(2)` (no da pasos
  ante un empujón: se deja caer).
- Python solo implementa `SetFsmId` (Damp, StandUp, Start, ZeroTorque), `SetStandHeight` (High/LowStand) y
  `SetVelocity` (Move, StopMove). C++ además los `Get*`, `SetBalanceMode`, `SetSwingHeight`, `SetNextFoot`,
  `GetOdom`, `SetTargetPos`, `WaveHand`, `ShakeHand`.
- El ejemplo oficial en C++ guarda los argumentos en un `std::map`: **se ejecutan en orden alfabético**; sin
  `--network_interface=eth0` usa `lo`; imprime «Done!» ignorando los códigos de retorno.

### Rumbo y distancia

- **El control de Unitree deriva ~2°/s en recta**, y no siempre al mismo lado. Cerrar el rumbo con un P sobre
  el yaw de la IMU (`cuadrado.py`: `KP_RECTA = 1.5`, `VYAW_RECTA_MAX = 0.25`; giros `KP_GIRO = 1.2`,
  `VYAW_GIRO_MIN = 0.15`, tolerancia 3° estable 0.6 s, pausa de 0.8 s antes de cada giro). Rumbos acumulados
  desde el inicial; yaw envuelto a ±180°.
- **La referencia se toma en el último momento**, no al arrancar el programa. Caso real: el robot se recolocó
  158° con el mando mientras esperaba, el P saturó y la primera recta curvó ~65°.
- **`GetOdom` devuelve (0, 0, 0)** en el PC de desarrollo: la distancia se estima por tiempo,
  `t = lado / (vx · factor)`, con el factor calibrado con cinta.
- Cortes de seguridad usados: `rt/lowstate` > 0.5 s sin llegar, motor en fallo, roll/pitch > 20°, giro sin
  acabar en 20 s.

### Códigos de error del RPC

| Código | Significado | Código | Significado |
|---|---|---|---|
| 3001 | Error desconocido | 3202 | Error interno del servidor |
| 3102 | Error al enviar la petición | 3203 | API no implementada |
| 3103 | API no registrada | 3204 | Error de parámetro |
| **3104** | **Tiempo de espera agotado** | 3205 | Petición rechazada |
| 3105 | Petición y respuesta no coinciden | 3206 / 3207 | Lease inválido / ya existe |
| 3106 | Datos de respuesta inválidos | 5201 | Error al conmutar un servicio |
| 3107 | Lease inválido | 5202 | Servicio protegido |

El 3104 casi siempre es: interfaz de red equivocada, robot en Debug, o el timeout de 1 s.

---

## 10. Simulación con MuJoCo

### Lanzar el H1-2

```bash
cd ~/unitree_mujoco/simulate/build
./unitree_mujoco -r h1_2 -s scene.xml
```

- En la consola debe salir `Using unitree_hg ...` y `Elastic band attached to: torso_link`: el robot aparece
  colgado de la cinta y ya publica `rt/lowstate`.
- Modelo: `unitree_robots/h1_2/h1_2_handless.xml` (27 actuadores, sin manos). `-r` tiene prioridad sobre el
  `robot:` del `config.yaml`.
- `simulate/config.yaml`: `enable_elastic_band: 1`, `domain_id: 1`, `interface: "lo"`.
- Teclas: `7`/`8` subir/bajar la cinta, `9` soltar o volver a colgar, Espacio pausa, Retroceso reinicia la
  postura, Ctrl + doble clic + arrastrar empuja una pieza.
- El simulador publica `rt/lowstate` y escucha `rt/lowcmd` igual que el robot: el mismo programa corre contra
  él cambiando a **dominio 1 sobre `lo`** en vez de dominio 0 sobre `eth0`.

**Usar siempre el simulador en C++.** El de Python (`simulate_python/unitree_sdk2py_bridge.py`) solo usa
`unitree_hg` cuando `ROBOT == "g1"`; con `h1_2` publica `unitree_go` y los programas del H1-2 **no reciben
nada, sin error** (además su cinta busca un `base_link` que el H1-2 no tiene). El de C++ elige `unitree_hg`
automáticamente (27 > 20 motores). Comprobado en la instalación local.

`simulate unitree_robots/h1_2/scene.xml` solo sirve para mirar el modelo: sin cinta ni DDS, el robot se
desploma.

### Lo que la simulación no reproduce

| En la simulación | Consecuencia |
|---|---|
| `mode_machine = 0` (en el robot 4 o 6) | Leerlo siempre, nunca fijarlo |
| No lee `mode_pr`: `motor_cmd[i]` va directo al actuador `i` | El modo AB no se puede probar (`USAR_MODO_AB = False`) |
| No implementa `rt/arm_sdk` | Los brazos se validan por `rt/lowcmd` con las piernas sostenidas |
| No hay control de marcha, `LocoClient` ni manos | La marcha y el agarre solo se prueban en el robot |
| No hay Debug ni MotionSwitcher | La lógica de `CheckMode`/`ReleaseMode` no se ejercita en la sim |
| PD puro sin compensar la gravedad | El hombro cede ~0.15 rad con el brazo en alto; en reposo, < 0.03 rad del objetivo |

### Ejemplos de la capacitación (`code_cap/pc/simulacion_mujoco_h1_2/`)

| Script | Qué hace |
|---|---|
| `scripts/test_unitree_sdk2_mod.py` | Comprueba el DDS: imprime la IMU y manda 1 N·m a las 27 juntas |
| `scripts/h1_2_low_level_example.py` | Tres etapas de 3 s: todo a cero desde la postura medida; tobillos en PR ±0.25 rad a 1 Hz; + muñecas roll ±0.5 rad. Ctrl+C publica 50 veces «todo a cero». **No pone Debug**; en el robot llevaría las piernas a cero: solo colgado |
| `scripts/h1_2_arms_example.py --pose X.json` | Ejecuta una rutina con perfil coseno; valida el campo `"robot"`, índices 12-26 y que llegue `rt/lowstate` con 27 motores en 8 s |
| `herramientas_extra/h1_2_mujoco_selector.py` | Menú con las rutinas de `poses/` |
| `herramientas_extra/play_pose_mujoco_h1_2.py` | Una rutina con ganancias blandas (kp 40 / kd 1): para ver el gesto, no para juzgar el seguimiento |
| `herramientas_extra/capture_pose_mujoco_h1_2.py` | Solo escucha: captura la postura del simulador como rutina |
| `herramientas_extra/editor_poses_mujoco_h1_2.py` (`editor_mujoco.sh`) | Editor con los sliders de MuJoCo |
| `herramientas_extra/h1_2_robot_selector.py` | Copia del selector del robot real: con `lo` prueba todas sus protecciones |

### Formato de una rutina

```json
{ "nombre_rutina": "1_saludo_derecha",
  "robot": "unitree_h1_2",
  "modelo": "h1_2_27dof",
  "pasos": [
    { "nombre": "saludo 1",
      "posiciones": { "20": -0.7854, "23": -0.497325 },
      "duracion": 1.0 } ] }
```

- Claves = índices DDS (solo 12-26), valores en radianes, `duracion` en segundos para llegar.
- También se aceptan `.txt` con una línea por paso: `indice valor duracion`.
- Las juntas que la rutina no nombra se quedan donde estaban al arrancar.
- El editor de sliders es **cinemático** (pelvis fija a z = 1.40, sin gravedad, contactos ni actuación): no
  detecta que un brazo atraviese el torso. Revisar con `p` y probar después en `unitree_mujoco`.

### Modelos

`unitree_ros/robots/h1_2_description/`: `h1_2.urdf`, `h1_2_handless.urdf` (para control) y
`h1_2_with_FTP_hand.urdf` (con las bielas del tobillo paralelo). En este repo,
`ros_h1_2_ws/src/h1_2_inspire_description/` tiene el modelo con las manos RH56DFTP.

### Estado de la instalación en el portátil `utec-Precision-3581` (comprobado el 2026-09-30)

- `~/unitree_mujoco` con el binario `simulate/build/unitree_mujoco`; `~/unitree_sdk2`,
  `~/unitree_sdk2_python` y `/opt/unitree_robotics` presentes. Ubuntu 22.04, ROS 2 Humble.
- `config.yaml` con `robot: "go2"` (no importa si se lanza con `-r h1_2`), `domain_id: 1`, `interface: "lo"`,
  `enable_elastic_band: 1`.
- `mujoco` de Python 3.14.0; la capacitación compila MuJoCo 3.2.7 para el simulador C++.

---

## 11. Manos Inspire RH56DFTP

- RH56DFTP: RS-485 y Modbus TCP, 17 sensores táctiles; 6 grados de libertad, 12 juntas. (La RH56DFX es solo
  RS-485, sin tacto; el RS-485 admite un dispositivo a ~20 Hz y no publica tacto.)
- **Izquierda 192.168.124.211, derecha 192.168.124.210** (al revés que la documentación; las dos reportan
  `HAND_ID = 1`: solo se sabe mirando). Puerto 6000. Cuelgan de `br0` (192.168.124.0/24).
- DOF: 0 meñique, 1 anular, 2 medio, 3 índice, 4 pulgar (flexión), 5 pulgar (rotación).
  **0 = cerrado, 1000 = abierto.**

| Registro | Nombre | Uso |
|---|---|---|
| 1486 | ANGLE_SET | Consigna; −1 = no tocar |
| 1498 | FORCE_SET | Límite de fuerza por DOF (gf); fábrica 2000 = sin límite |
| 1522 | SPEED_SET | Velocidad (150 = lento) |
| 1546 | ANGLE_ACT | Ángulo medido |
| 1582 | FORCE_ACT | Fuerza medida (gf) |
| 1594 / 1618 | I / T | Corriente / temperatura |
| 1606 / 1612 | err / status | 1 byte por DOF |
| 1004 | CLEAR | 1 borra un error |
| 1005 | SAVE | **Nunca**: graba en la mano |

- Leer 1546..1620 de un solo bloque devuelve valores inválidos (fuerzas de 11 300 gf): **leer cada bloque
  por separado**. Para rotar el pulgar, abrir antes los dedos.
- Por DDS (driver del teleop): `rt/inspire_hand/ctrl/{l,r}` (usuario → robot), `state/{l,r}` y
  `touch/{l,r}` (robot → usuario). El campo `mode` es una máscara: bit 0 ángulo, bit 1 posición, bit 2
  fuerza, bit 3 velocidad; `mode = 15` las cuatro.
- `inspire_sdkpy` revienta con valores negativos en `angle_set` (−1 es el no-op del protocolo): lo corrige
  `h1_2_teleoperation/patches/inspire_sdkpy_uint16.patch`.
- **Agarre validado (caja):** `FORCE_SET` 500 gf por dedo y 600 el pulgar, con la consigna **cerrada del todo
  durante toda la marcha** (fijarla al ángulo actual hace que la fuerza caiga a ~800 gf y la caja resbale).
  Cada dedo al 60-130 % de su objetivo, límite de seguridad 1200 gf (el firmware sobrepasa la consigna),
  total mínimo 600 gf, hasta 3 intentos. Meñique y anular suelen quedar flojos.
- Puerto serie: `sudo usermod -aG dialout $USER` y volver a entrar en la sesión.

---

## 12. Diagnóstico

### Fallos que no dan error

`rt/lowcmd` con `CheckMode = 'ai'` · `rt/arm_sdk` en Damp · `StandUp()` / `Start()` / `SetFsmId()` · falta
de CRC · batería a cero por `rt/lf/bmsstate` · simulador Python con `h1_2`.

### Diccionario de síntomas

| Síntoma | Causa habitual |
|---|---|
| El robot vibra al usar el SDK | No está en Debug: el control interno también publica en `rt/lowcmd` |
| El comando no tiene efecto | Falta el CRC32, o `CheckMode` da `'ai'` |
| Movimiento anómalo, juntas desviadas | Postura de encendido incorrecta: reencender |
| `low_state CRC Error` | Versión del SDK distinta de la del firmware |
| Funciona en un H1-2 y en otro no | `mode_machine` fijado en vez de leído |
| Movimiento brusco al arrancar | No se interpoló desde `motor_state[i].q()` |
| El `LocoClient` devuelve 3104 | Modo Debug, interfaz equivocada o timeout corto |
| El robot anda torcido o deriva | Deriva de ~2°/s del control: cerrar el rumbo con la IMU |
| `rt/arm_sdk` no mueve los brazos | FSM en Damp: hace falta FSM 201 |
| Un lazo de rumbo satura desde el inicio | Referencia tomada antes de recolocar el robot |
| `No module named unitree_sdk2py` en el robot | `python3` del sistema: usar el del `.venv` del SDK |
| `No module named catkin_pkg` | Miniconda antepone su Python al de ROS 2 |
| `ros2 topic list` vacío | `CYCLONEDDS_URI` con la tarjeta equivocada o falta el RMW |
| Una batería no arranca | Reinsertar hasta el clic; intercambiar las dos |

### Códigos de fallo de motor

`motor_state[i].motorstate` es un entero donde cada bit es un fallo. La tabla depende de `reserve[2]`
(generación del driver: 0, 2 o 4): **el mismo número significa cosas distintas según la generación**.

```cpp
for (int i = 0; i < 27; ++i) {
  uint32_t st = low_state.motor_state()[i].motorstate();
  if (st) {
    uint32_t gen = low_state.motor_state()[i].reserve()[2];
    printf("[ERROR] motor %d gen %u code 0x%08X\n", i, gen, st);
  }
}
```

| Código (tabla `reserve[2] == 0`) | Error |
|---|---|
| 0x01 | Sobrecorriente |
| 0x02 | Sobretensión |
| 0x04 | Sobrecalentamiento del driver |
| 0x08 | Subtensión de bus |
| 0x10 | Sobrecalentamiento del bobinado |
| 0x20 | Encóder anómalo |
| 0x40000000 | Timeout motor → PC |
| 0x80000000 | Timeout PC → motor |

Placa base, por `rt/lf/mainboardstate` (fuente 200): 0x01000 parada de emergencia disparada, 0x00400
protección por subtensión.

### Conclusiones técnicas de la capacitación

1. Muchos fallos no generan error: **verificar el efecto**, no el código de retorno.
2. Debug = `CheckMode` vacío, no «nadie publica».
3. La referencia (rumbo, postura) se toma en el último momento.
4. Transiciones lentas: 12 s de rampa y 30 s de brazo evitan los pasos.
5. `duration = 1` convierte cualquier corte en una parada de 1 s.
6. Una confirmación tecleada antes de mover; nunca por defecto.
7. Registrar siempre: CSV + JSON por ensayo.
8. Verificar la red y el USB antes de diagnosticar el sensor (rp_filter, D435i a 5000 Mbps).

---

## Fuentes

- Presentación: `Capacitacion_H1_2_Robotics40.pdf` (R40-CAP-H1_2-0001 v1.10). En el PC de UTEC:
  `/home/utec/Documents/UTEC/CAPACITACION/`.
- Código de la capacitación: `code_cap/`, en la misma carpeta. Es la copia del 25-09-2026 de
  `/home/unitree/robotics40` en el PC de desarrollo del robot, que es la versión en uso. Empezar por
  `code_cap/README.md`: la tabla de lanzadores dice qué mueve el robot y qué hace falta antes.
  - Simulación: `code_cap/pc/simulacion_mujoco_h1_2/` (guía `Simulacion_H1_2_Mujoco.md`).
  - Selector de poses del robot real, con sus límites y ganancias:
    `code_cap/selector_poses_real/scripts/herramientas_extra/h1_2_robot_selector.py`. **Solo se ha probado en
    MuJoCo**; en el robot aún no ha movido nada.
  - Lectores de solo lectura: `code_cap/lectores_h1_2/`.
- Documentación oficial de Unitree: About_H1-2, H1-2_Basic_Services_Interface,
  H1-2_Motion_Services_Interface, H1-2_Joint_motor_sequence, H1-2 Ankle control routine, H1-2_Arm_Control,
  Quick Start, Remote Control Description, H1-2 User Manual V1.0.
- Repositorios: `unitree_sdk2`, `unitree_sdk2_python`, `unitree_ros`, `unitree_ros2`, `unitree_mujoco`
  (github.com/unitreerobotics).
- Hallazgos propios de este repositorio que coinciden con la capacitación (`rt/lowcmd` ya tiene dueño, el
  canal bueno para los brazos es `rt/arm_sdk`): `README.md` y `h1_2_joint_control/docs/01_DIAGNOSTICO.md`.
