# manos/ — las manos Inspire RH56DFTP

Dos manos de 6 DOF por **Modbus TCP, puerto 6000**, en la red `br0`:

| Mano | IP | ¡Ojo! |
|---|---|---|
| izquierda | 192.168.124.211 | al revés que en la doc de Unitree; las dos dicen HAND_ID 1 |
| derecha | 192.168.124.210 | |

DOF: `[meñique, anular, medio, índice, pulgar (flexión), pulgar (rotación)]`; **0 = cerrado, 1000 = abierto**.

## Lanzador: `manos.sh`

```bash
./manos.sh leer                  # SOLO LECTURA: ángulo, fuerza, err, status y temperatura de cada dedo
./manos.sh puno ambas            # cerrar en puño (izq | der | ambas)
./manos.sh abrir izq
./manos.sh borrar-error izq      # un dedo en fallo (err 5 = bloqueo + sobrecorriente, status 7)
./manos.sh prueba                # las dos: abrir/cerrar 15 s, CSV en datos_manos/
./manos.sh prueba-pulgar         # igual, con la oposición del pulgar hacia la palma
./manos.sh preforma izq          # coger_caja.py: mano abierta con el pulgar enfrente de los dedos
./manos.sh cerrar izq            #   cierra, cada dedo se congela al tocar y aprieta un poco
./manos.sh soltar izq
./manos.sh pulgar-tactil izq     # cierra solo el pulgar hasta que su táctil toca
./manos.sh test read izq         # hand_test.py (UTEC): read | wiggle | sweep | open, --dof, --target...
./manos.sh sonda                 # probe_hands.py (UTEC), solo lectura
./manos.sh driver                # puente Modbus <-> DDS en primer plano (el del teleop XR)
```

Todos usan `~/teleop_venv` (el que tiene `pymodbus`).

## Ficheros

| Fichero | Qué es | Estado |
|---|---|---|
| `mano.py` | Órdenes sueltas (leer, puño, abrir, borrar error) con la clase `Mano` de `caja_cuadrado.py` | nuevo; `puno` es el de la sesión 10 |
| `prueba_manos.py` | Las dos manos a la vez, ciclos de 15 s con registro a 64 Hz | probado 2026-09-25, 0 errores |
| `prueba_manos_pulgar.py` | Igual con oposición del pulgar | probado 2026-09-25, 0 errores |
| `coger_caja.py` | Agarre con parada por fuerza de contacto | probado 2026-09-23 |
| `pulgar_tactil.py` | Pulgar con parada por el sensor táctil | probado 2026-09-23 |
| `h1_2_utec/` | **Copia** de los scripts del repo `smorales2405/h1_2_utec` (no son nuestros). El teleop usa el de `~/` | funcionan |
| `experimental/` | `dedos_tactil.py`, `agarre_completo.py`: agarre por táctil | **no salieron** (fuerza desigual, el reparto no converge) |

## Lo que hay que saber

- **Nunca escribir el registro SAVE (1005)**: guardaría la configuración en la mano. Ninguno de estos lo hace.
- **Leer los registros de uno en uno**: leer 1546..1620 de golpe da basura (fuerzas de 11300 gf, todo a 255).
  Ángulo 1546, fuerza 1582, corriente 1594, `err` 1606, `status` 1612, temperatura 1618.
- `FORCE_SET` (1498) viene de fábrica en 2000 = sin límite; el firmware se pasa del límite que se le ponga.
- Para girar el pulgar, **primero abrir los dedos**; si no, choca con ellos.
- La rotación del pulgar no pasa de ~973 aunque se pidan 986.
- Las manos cuelgan del adaptador USB `net4`, el mismo que el LiDAR: quitarlo deja sin manos.
