# Editor de la pose de observación

Interfaz de prueba para elegir la pose de observación de los brazos del reto de manipulación. Funciona
como `h1_2_mujoco/editor_gui.sh`: MuJoCo sin física, en una sola ventana. **No mueve el robot ni escribe en
`config/`**: cuando una pose te convenza, la copias a mano a `config/poses_brazos.yaml`.

```bash
cd retos_capacitacion/manipulacion/sim/editor_observacion
./editor_observacion.sh                     # en la PC (necesita mujoco, pinocchio y PyQt5)
./editor_observacion.sh --poses otro.json   # otro archivo de poses
```

## Qué se ve

Es el gemelo de `manipulacion.py` (`sim/escena.py`): el robot de pie en la postura medida en FSM 201
(`POSTURA_PIE`), con las manos abiertas, delante del carrito medido (`config/montaje.yaml`), **sin bandejas
ni objetos**. Con el ratón: el botón izquierdo gira la vista, el derecho la desplaza y la rueda acerca; un doble
clic vuelve a la vista inicial.

## Controles

- **Brazos en espejo.** Hay 7 sliders en grados para el brazo izquierdo, en el orden de `arm_sdk` (13–19). El
  derecho es su espejo (`manip.cinematica.espejo`, juntas 20–26), como lo usa `manipulacion.py`; su valor se
  ve a la derecha de cada fila. El rango de cada slider son los límites del programa (los de la web ∩ los
  del URDF, menos 0.05 rad), recortados para que también el espejo del brazo derecho quepa en los suyos.
  Manos, torso y piernas no se tocan.
- **Poses de observación.**
  - Se guardan en `poses_observacion.json`, en esta carpeta, con «Guardar nueva…», «Sobrescribir» y
    «Borrar».
  - Al pulsar una pose en la lista, se carga en los sliders y se ve.
  - En gris aparecen, solo para ver, la observación en uso y las alternativas de `config/poses_brazos.yaml`.
  - «Copiar YAML» deja en el portapapeles las líneas `observacion: izq/der` de la pose de los sliders.
- **Movimiento.**
  - «Reposo → pose» y «Pose → reposo» mueven los dos brazos a la vez, como lo hace `manip/brazo.py`: en línea
    recta en el espacio articular, con mínima sacudida y la duración de la sección 10. Esa duración es de al
    menos 8 s, con un pico de 0.5 rad/s como mucho, y se multiplica ×2 en las primeras tiradas, como
    `--escala-tiempo 2`, o ×1 si eliges la mínima.
  - «Reproducir a» solo acelera la animación.
  - Durante el camino se ven las colisiones. Al terminar, el registro resume el peor valor de cada pareja,
    con el camino entero muestreado.
- **Colisiones.** Son las del planificador (`sim/colisiones.py`): **choque** si hay interpenetración
  (< 0, en rojo) y **cerca** si queda a menos de la holgura del plan (2 cm, en ámbar). Se miran las del
  robot contra el carrito (tablero, labio, asas, patas y baldas) y contra su propio cuerpo. Solo se pintan las
  partes del robot; la lista de abajo dice contra qué choca.
- **Comprobar como manipulacion.py.** Lo hace sobre el gemelo del programa, **con las bandejas**, como en
  la tirada.
  - El tramo reposo ↔ pose de cada brazo, con `Planificador.entre_reposo_y_observacion`. Si no pasa, la
    tirada no empieza.
  - Los criterios de `herramientas/poses_brazos.py`, como avisos:
    - la mano a 12 cm o más sobre la mesa;
    - el brazo sin tapar la zona de recogida en la imagen de la cámara;
    - desde la pose, se llega limpio a las esquinas y al centro de la zona, con la palma abajo y a la altura
      de transporte.

## Ficheros

| Fichero | Qué es |
|---|---|
| `editor_observacion.sh` | Lanzador (`PY=/ruta/python` para elegir el Python) |
| `editor_observacion.py` | La ventana (PyQt5) |
| `nucleo.py` | Escena, espejo, trayectoria, colisiones, criterios y archivo de poses, sin Qt |
| `poses_observacion.json` | Las poses guardadas: `izq` en rad (13–19), `der` = espejo (20–26), `izq_grados` y notas |

Solo **lee** el código de la manipulación (`manip/`, `sim/escena.py`, `sim/colisiones.py`, `config/`): no lo
cambia. La animación no tiene física: el robot real lleva el brazo con kp ≤ 150 y se queda unos grados
corto. La política de la marcha equilibra el cuerpo; un movimiento amplio puede hacerle dar pasos, y eso
solo lo dice el robot (Hito 3).
