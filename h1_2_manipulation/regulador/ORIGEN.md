# Copia fija del regulador de fuerza

Copiados **tal cual, sin cambiar una línea**, de `smorales2405/inspire_hand_interface`:

| Aquí | Origen | sha256 |
|---|---|---|
| `nucleo.py` | `Control/nucleo.py` | `93b32b6315e86cc7771050bb4467de0846e3c59d0eb629fa296a55aa4f71ae05` |
| `hand_modbus.py` | `Caracterizacion/hand_modbus.py` | `56ff50bfb998d361e84c282487b5ed1ab651ec354fb3214856874a3bf62a6581` |

- Commit del repo al copiar: `fb858ca` (2026-11-24). Último cambio de los dos ficheros: `775db1d` (2026-09-21).
- Es una **copia fija** (opción b): una mejora del regulador en su repo **no llega aquí sola**. Para traerla, se
  vuelven a copiar los dos ficheros, se actualiza esta tabla y se repiten las pruebas (`tests/`).
- `nucleo.py` importa `hand_modbus` como módulo suelto. `agarre/reto.py` pone esta carpeta en `sys.path`.

## Qué se usa y qué no

- **Se usa de `nucleo.py`:**
  - `PI`: fuga, banda muerta, cuantización asimétrica con acumulador de resto y periodo refractario;
  - `Disparador`: el control avanza con muestra fresca o con el plazo agotado;
  - `EstimadorRLS`: la matriz de ganancia de la pinza en línea, en el modo 1;
  - `DetectorResbalon`: el actuador cede y la reacción es aflojar;
  - `DetectorEscape`: el objeto se escapa y la reacción es apretar;
  - `Guarda`;
  - `Bitacora`: el CSV con las mismas columnas que las tandas del banco.
- **No se usa:**
  - `Tara` (escribe `FORCE_CLB`). En el robot no se tara por hardware: la tara cambiaría el cero que usa también
    el brazo sin regulador (su calibración del residuo) y contaminaría la comparación. El cero se resta por
    software, medido con la mano abierta ≥ 10 s después de soltar (la misma regla de E3.5).
  - `Lector`: lee por `HandModbus`. Aquí la lectura va por el cliente y el cerrojo de la `Mano` del reto, que
    comparte la conexión con el supervisor.
- `lazo.py` (los modos del banco) no se copia. Los lazos coordinados que el banco validó (A3/A4 apriete y balance,
  B1 apriete y reparto) están reescritos para el agarre del robot en `../agarre/lazo.py`, con la misma ley de
  control y citando de dónde sale cada constante.
