# selector_poses_manos — rutinas de brazos y manos en el robot real

> **NUNCA SE HA EJECUTADO EN EL ROBOT.** Probado contra un robot y unas manos simulados (DDS en `lo`,
> Modbus TCP en local, 2026-10-01). Además, el selector de `code_cap` del que parte **tampoco ha movido
> nunca el robot**: la primera vez, colgado y con el responsable del laboratorio delante.

Es el selector de `code_cap/selector_poses_real/` con las **manos Inspire RH56DFTP**. Los brazos van igual
(las 27 juntas por `rt/lowcmd`, robot en Debug y **colgado**: en Debug las piernas no se sostienen); las
manos, por Modbus TCP. Las rutinas se diseñan en MuJoCo con `h1_2_mujoco/editor_mujoco.sh`.

## Archivos

| Archivo | Qué es |
|---|---|
| `selector_manos.sh` | lanzador, en el PC2 del robot |
| `h1_2_robot_selector_manos.py` | el selector |
| `gestos_mano.py` | gestos de mano y orden de `cerrada`; lo usa también el editor de MuJoCo |
| `conversion_angle_set.py` | radianes del URDF ↔ ANGLE_SET de la mano real |
| `poses/` | rutinas `.json`; `0_pose_segura.json` es la postura por defecto (brazos de `code_cap`, manos abiertas) |

## Llevarlo al robot

En el PC2 vive en **`~/utec/selector_poses_manos/`** (copiada el 2026-10-02). Se copia la carpeta entera: el
selector importa `gestos_mano.py` y `conversion_angle_set.py` de su propia carpeta y lee las rutinas de
`poses/`. Desde la raíz del repositorio, en el PC:

```bash
rsync -av --exclude '__pycache__' h1_2_joint_control/scripts/selector_poses_manos unitree@192.168.0.143:~/utec/
```

Para actualizarla tras cambiar el código o añadir rutinas, el mismo comando: solo copia lo que cambió.
**Sin `--delete`**, para no borrar las rutinas que se hayan creado en el robot. Por cable, `unitree@192.168.123.164`.

No hace falta configurar nada más en el robot (comprobado el 2026-10-02):
- `~/teleop_venv/bin/python` (Python 3.10, `pymodbus` 3.6.9) tiene todo lo que importa el selector; su copia
  del SDK (`~/pc2_offline/src/unitree_sdk2_python`) es idéntica a `~/unitree_sdk2_python` en las piezas que
  usa. `PY=/ruta/python` para otro.
- El lanzador quita el `CYCLONEDDS_URI` del SLAM que exporta el `.bashrc` del robot.

Se lanza desde el PC con `ssh -t` (hace falta terminal para teclear `COLGADO`, `SOLTAR` y `MOVER`):

```bash
ssh -t unitree@192.168.0.143 ~/utec/selector_poses_manos/selector_manos.sh --sin-manos
```

> **Si se corta el SSH**, el selector muere sin su cierre: ni pose segura, ni rampa a amortiguación, ni manos
> abiertas. Por WiFi es más probable: mejor por cable o desde una terminal en el propio PC2. El robot no tiene
> `tmux` ni `screen`.

## Cómo se lanza

```bash
./selector_manos.sh --sin-manos      # PRIMERA PRUEBA: solo brazos
./selector_manos.sh                  # brazos y manos, eth0 y las rutinas de poses/
```

Pide escribir `COLGADO`; si el servicio de movimiento sigue activo, `SOLTAR` (llama a `ReleaseMode()`: las
piernas se sueltan); y luego `MOVER`. Después: número = rutina, `l` = listar, `x` = salir.

**La pose segura es la postura por defecto.** Tras `MOVER`, el robot va a ella (3 s, manos abiertas). Cada
rutina parte de ahí y termina en ella: si su último paso no es la pose segura, el selector añade uno de 3 s
(no hace falta guardarlo en la rutina). Entre rutinas, el robot se queda sujetándola. Solo al salir (`x`)
pasa a amortiguación. Sin `poses/0_pose_segura.json`, el selector no arranca.

Antes, en el robot:
- robot **colgado** con los pies en el aire, Debug con `L2+R2` (`L2+A` da la postura de diagnóstico) y alguien
  con el mando: **L2 + B**;
- **nadie más mandando**. El lanzador no arranca si está activo el servicio `h1_2_six_seven`
  (`sudo systemctl stop h1_2_six_seven`) o un driver de manos (`inspire_ftp_dual_driver`: teleop XR o
  `manos.sh driver`);
- las manos sin error: `manos/manos.sh leer`, y si hace falta `manos/manos.sh borrar-error izq|der`.

Contra `unitree_mujoco` (que no tiene manos): `python3 h1_2_robot_selector_manos.py lo --sin-manos`.

## Protecciones

Las de los brazos son las del selector de `code_cap`: `CheckMode` vacío (o `ReleaseMode()` confirmado con
`SOLTAR`), 3 s escuchando `rt/lowcmd` sin que nadie publique, `MOVER`, límites web ∩ URDF con 0.05 rad de
margen, primer movimiento de al menos 3 s, amortiguación si falla un motor o se pierde `rt/lowstate`, y al
salir pose segura y kp → 0. Las de las manos:

- Al conectar solo **lee** (err y postura); no escribe nada hasta `MOVER`. Si una mano no conecta o tiene
  `err`, no arranca: `--sin-manos` para seguir solo con los brazos.
- Durante cada paso manda **ANGLE_SET interpolado a 25 Hz** desde lo último mandado, convertido con
  `conversion_angle_set.py`: los dedos llegan a la vez que el brazo y por el mismo camino que en el editor.
  El primer movimiento de cada mano dura al menos 3 s. `SPEED_SET` se pone a 1000 durante la sesión (la
  rampa la marca la interpolación) y se restaura al salir.
- **`cerrada`** va por fases en el orden de `puno()`: abrir la flexión del pulgar → rotación a su sitio →
  cerrar los cuatro dedos → flexionar el pulgar. Cada fase dura al menos 0.6 s y la siguiente no empieza
  hasta que la mano llega (±40 de ANGLE_SET, como mucho 2 s; si no llega, la mano se para).
- Si un dedo pasa de **800 gf** (el corte de `puno()`) o la mano da `err`, ese dedo se abre y la mano deja de
  recibir órdenes hasta el final de la sesión; los brazos siguen.
- Al salir, o si el robot pasa a amortiguación, **las manos se abren** (2 s).

## Formato

Las rutinas son las del editor de MuJoCo (ver `h1_2_mujoco/README.md`): `posiciones` con las juntas 12-26
en radianes y `manos` con las 6 juntas actuadas del URDF en radianes por mano. Un paso sin `manos` las deja
como están, así que las rutinas de `code_cap` (solo brazos) también valen.
