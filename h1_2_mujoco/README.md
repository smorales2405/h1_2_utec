# h1_2_mujoco — editor de poses del H1-2 con manos, en MuJoCo

Diseñar rutinas de **torso, brazos y manos** en este PC, ver si algo choca, y guardarlas para el selector
del robot real con manos (`h1_2_joint_control/scripts/selector_poses_manos/`).

Parte del editor de la capacitación (`code_cap/pc/simulacion_mujoco_h1_2/editor_mujoco.sh`), que no se
toca. Lo que cambia: el modelo es el H1-2 **con las manos Inspire RH56DFTP** de este repositorio, hay
gestos de mano, y las colisiones se comprueban.

```bash
cd h1_2_mujoco
./editor_mujoco.sh                   # abre el visor y deja el menú en la terminal
./editor_mujoco.sh --margen 0.03     # avisar de lo que quede a menos de 3 cm (por defecto 2 cm; 0 = solo choques)
```

## Cómo se usa

1. Se abre el visor, con la escena del editor de `code_cap`, y el **H1-2 levantado** (pelvis fija a 1.4 m), los brazos a 0 y las **manos abiertas**
   (q = 0). No hay gravedad, contactos ni actuadores: **no se mueve solo**.
2. **Barra espaciadora = pausa.** Con la pausa, el panel **«Joint»** (a la derecha) tiene un slider por junta:
   - torso y brazos (`torso_joint`, `left_shoulder_pitch_joint`…), limitados a lo que acepta el selector real;
   - dedos: las 6 juntas actuadas de cada mano (`*_little_1`, `*_ring_1`, `*_middle_1`, `*_index_1`,
     `*_thumb_1`, `*_thumb_swing`), de 0 (abierta) a q_max (cerrada). Los sliders de las falanges
     acopladas (`*_2`, `*_3`) **no hacen nada**: el editor las arrastra con su junta actuada.
3. En la terminal, `c nombre t=2` captura la postura (brazos y dedos) como paso.
4. `p` previsualiza la rutina y dice, paso a paso, si algo choca por el camino.
5. `s nombre` guarda **`selector_poses_manos/poses/<N>_nombre.json`** con el siguiente número libre.

| Comando | Qué hace |
|---|---|
| `c [nombre] [t=2]` | capturar la postura del visor (brazos y manos) como paso |
| `mano <gesto> izq\|der\|ambas [t=1]` | llevar la mano al gesto, animado como lo hará el robot |
| `l` | listar las 15 juntas y las dos manos (grados) con sus límites |
| `v` / `b` | ver los pasos / borrar el último |
| `ir <n>` / `reemplazar <n>` | llevar el visor al paso n / sustituirlo por la postura actual |
| `p` | previsualizar la rutina, comprobando colisiones |
| `espejo izq\|der [manos]` | copiar un brazo al otro en espejo; con `manos`, también la mano |
| `cero` | torso, brazos y manos a 0 (manos abiertas) |
| `col` | colisiones de la postura actual |
| `cargar <n\|fichero>` | abrir una rutina (también las de `code_cap`, sin manos) |
| `s <nombre>` | guardar para el selector |
| `x` | salir (o cerrar la ventana: lo no guardado va a `h1_2_mujoco/borradores/`) |

## Gestos

Definidos en `selector_poses_manos/gestos_mano.py`, que comparten el editor y el selector real. Valores
en radianes del URDF (0 = abierta); entre paréntesis, el ANGLE_SET que manda el selector
(`conversion_angle_set.py`: 1000 = abierto, 0 = cerrado).

| Gesto | meñique, anular, medio | índice | pulgar flexión (`thumb_1`) | pulgar rotación (`thumb_swing`) |
|---|---|---|---|---|
| `abierta` | 0 (1000) | 0 (1000) | 0 (1000) | 0 (1000) |
| `cerrada` | 1.62 (0) | 1.62 (0) | 0.492 (300) | 0 (1000) |
| `pulgar_arriba` | 1.62 (0) | 1.62 (0) | 0 (1000) | 0 (1000) |
| `senalar` | 1.62 (0) | 0 (1000) | 0.66 (0) | 0.61 (500) |

**`cerrada` sigue el orden de `puno()`** (`code_cap/caja_cuadrado.py`): abre la flexión del pulgar, lleva
su rotación a su sitio, cierra los cuatro dedos y flexiona el pulgar. El resto de posturas van de una
vez, en línea recta: su orden lo decide quien diseña la rutina, y si algo choca se ve en la terminal.

## Formato de la rutina

El del selector de `code_cap`, con una clave `manos` por paso:

```json
{ "robot": "unitree_h1_2",
  "modelo": "h1_2_27dof_manos_rh56dftp",
  "pasos": [
    { "nombre": "saludo", "duracion": 2.0,
      "posiciones": { "12": 0.0, "13": -0.5, "...": "juntas 12-26 en rad" },
      "manos": { "izq": { "little_1": 0.0, "ring_1": 0.0, "middle_1": 0.0,
                          "index_1": 0.0, "thumb_1": 0.0, "thumb_swing": 0.0 },
                 "der": { "...": "las mismas 6 claves" } } } ] }
```

- Un paso sin `manos` las deja como estén. Las rutinas de `code_cap` (sin manos) se cargan y se ejecutan tal cual.
- El selector de `code_cap` ignora la clave `manos`: con él, una rutina de aquí mueve solo los brazos.

## Colisiones

- **Cómo.** Una segunda copia del modelo, con contactos, comprueba la postura del visor cada 50 ms (y en cada
  fotograma de `p` y `mano`). La copia del visor sigue sin contactos y no se mueve sola.
- **Qué avisa.** `[COLISIÓN]` cuando dos cuerpos se interpenetran: brazo con brazo, brazo con el cuerpo, una
  mano con cualquier parte del robot (incluidos sus propios dedos). `[CERCA]` cuando dos partes distintas
  quedan a menos del margen (2 cm). Lo que choca se pinta de **rojo** en el visor.
- **Excluidos**, porque ya se tocan en la pose cero: `pelvis`/`torso_link`, `*_wrist_roll_link`/`*_wrist_yaw_link`
  y `*_hand_base_link`/`*_thumb_1`. Los pares que ya están a menos del margen en la pose cero
  (hombro y torso, dedos vecinos) solo avisan si chocan.
- `*_wrist_yaw_link` no tiene colisión en el URDF: aquí se le da su malla visual.

**Límites de la comprobación:**
- MuJoCo choca con la **envolvente convexa** de cada malla: en zonas cóncavas (hueco de la palma, torso)
  puede avisar de más, nunca de menos.
- El modelo no es el robot: el acoplamiento de los dedos es lineal (0.85) y el real no (a medio recorrido
  la punta del modelo va ~13° más doblada); los ceros de la mano tienen ~3° de error (3-6° en la rotación
  del pulgar); en el robot el brazo cede hasta ~0.15 rad por gravedad. De ahí el margen.
- Los brazos siguen en el robot el mismo camino que en `p` (los dos interpolan en línea recta). La mano real
  no exactamente: su control interno va con retraso. En las manos, lo fiable son los pasos capturados.

## Archivos

| Archivo | Qué es |
|---|---|
| `editor_mujoco.sh` | lanzador (busca un python con `mujoco`; `PY=/ruta/python` para otro) |
| `editor_poses_mujoco_h1_2_manos.py` | el editor |
| `modelo_h1_2_manos.py` | carga el URDF en MuJoCo (sin modificarlo), juntas mimic y comprobación de colisiones |

Necesita `mujoco` ≥ 3.2 (probado con 3.14.0, el `python3` de este PC), el URDF
`ros_h1_2_ws/src/h1_2_inspire_description/urdf/h1_2_with_RH56DFTP_hands.urdf` y, para los límites de los
brazos y los gestos, `h1_2_joint_control/scripts/selector_poses_manos/`. No usa el SDK de Unitree.
