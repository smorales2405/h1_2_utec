# h1_2_manipulation — el agarre con el regulador de fuerza

El reto de manipulación (`../retos_capacitacion/manipulacion`) como banco de pruebas del **regulador de fuerza** de
la Inspire RH56DFTP (`inspire_hand_interface/Control`). El objetivo es mejorar el agarre y la tasa de éxito con los
cuatro objetos (cubo, cilindro, prisma y esfera) y **compararlo con el agarre del reto**, que va por `FORCE_SET` del
firmware.

**El reto no se toca.** De él se importa todo lo demás tal cual: robot, brazos y arnés, cámara, percepción,
planificador, vigilante, supervisor y salida segura. Lo único distinto es:

- **la mano** (`agarre/mano_regulada.py`): hereda de la `Mano` del reto y solo cambia cómo cierra sobre el objeto,
  cómo lo sujeta y cuándo dice que se ha escapado;
- **el banco** (`agarre/banco.py`, con `--banco`): el bucle de ensayos, en vez del de llevar cada objeto a su bandeja.

Con `regular = False` la mano es la del reto, línea por línea. Ese es el brazo **firmware** de la comparación.

**Resultados:** `docs/RESULTADOS.md`, cada prueba en el robot con lo que se vio y lo que se cambió por ella.

## Uso

En el robot, igual que el reto. Desde la PC, los lanzadores se reenvían por SSH (`comun.sh`) y graban con la cámara
externa si está conectada.

```bash
./desplegar.sh                                      # el reto (su desplegar.sh) y esto -> ~/utec/h1_2_manipulation
./banco.sh --banco cubo --ensayos 20 --arnes        # el banco: 10 con regulador y 10 con firmware, intercalados
./banco.sh --banco esfera --ensayos 20 --arnes --sostener-s 5
./manipulacion_regulada.sh --nivel 2 --arnes        # el reto con la mano regulada (los cuatro objetos)
./manipulacion_regulada.sh --nivel 2 --arnes --brazo firmware   # la mano del reto, registrada igual
./desplegar.sh --traer                              # datos/ del robot -> datos/
python3 analisis/comparar.py --ultima               # el informe del último banco
python3 manipulacion_regulada.py --banco cubo --ensayos 4 --sim   # en la PC, en el mundo simulado
```

Las opciones del reto (`--arnes`, `--escala-tiempo`, `--sin-comprobar-montaje`, `--asignacion`, `--sim`...) pasan
tal cual. Las propias se ven con `python3 manipulacion_regulada.py -h`.

### El banco

Un solo objeto en la zona de recogida y N ensayos. Cada ensayo:

1. observar y planificar;
2. bajar y cerrar con el brazo que toque;
3. levantar y llevar el objeto sobre su bandeja;
4. **sostenerlo** `--sostener-s` segundos;
5. volver por el mismo camino, bajarlo por la recta de levantar hasta donde lo cogió y soltarlo.

6. volver a la observación y **mirar dónde quedó** (también tras el último): lo que se movió, en mm y grados, es una
   medida más.

**Cada ensayo empieza con el objeto en su sitio, sobre la cruz de cinta.** Devolverlo lo mueve: el 2026-11-25, unos
15 mm y 10° por ensayo con el cubo, y el agarre cambiaba con dónde y cómo estaba (`datos/banco_20261125_095047_cubo`).

- **Al lanzar el banco, deja el objeto fuera de la cruz.** El robot mira primero las cruces (con el detector del nivel 1,
  sin profundidad) y luego pide ponerlo **centrado sobre la cruz y recto** (las caras paralelas a los bordes de la mesa).
- **Su sitio** es donde lo ve esa primera vez. Lo apartado de la cruz (`sitio_desde_cruz_mm` en `resumen.json`) mide
  la percepción, como la esfera sobre la cruz del §79 del reto. Si la cruz estaba tapada, solo falta esa medida.
- **Tras cada ensayo**, si no quedó en su sitio (más de `--tol-sitio-mm` 5 mm o `--tol-sitio-deg` 5°), o si no se
  sostuvo, pide recolocarlo y dice hacia dónde (desde el robot: hacia él o lejos, a su izquierda o a su derecha, y el
  giro visto desde arriba). Lo vuelve a mirar y, si sigue fuera, dice otra vez cuánto falta. A la tercera lo acepta y
  queda apuntado en `sitio_mm` y `sitio_deg`.
- **Enter** para seguir, **q** para acabar, **ESPACIO** para la salida segura.
- `--sin-recolocar`: como antes. El siguiente lo encuentra donde quedó y solo pide recolocarlo si se cae o no se agarra.

En la simulación lo recoloca solo, donde estaba al empezar.

El orden de los brazos sigue el §8 del plan del regulador: una sola tanda, brazos intercalados en bloques
equilibrados (AB o BA al azar) y sin más de dos seguidos del mismo. La semilla queda en el registro. **No se compara
contra otra tanda ni contra otro día.**

Registro en `datos/banco_<fecha>_<objeto>/`:

- lo mismo que una tirada del reto: `eventos.csv`, `manos.csv`, `lowstate.csv`, fotos, `resumen.json` y la cámara
  externa;
- `ensayos.csv`: una fila por ensayo con su brazo, el resultado y sus medidas;
- `regulador_NN_<objeto>_<mano>[_firmware].csv`: cada agarre leído a ~200 Hz, con las columnas de las tandas del
  banco (`Control/data`) más la fase del supervisor.

En el brazo firmware el mismo lazo **solo mira** y no manda nada (`observar_firmware`), así que los dos brazos se miden
con las mismas lecturas.

## Cómo agarra el regulador

Todo vive en `agarre/lazo.py`; las constantes están en `config/regulador.yaml`, comentadas con su origen. Un agarre
regulado sigue cuatro pasos.

1. **Aproximación híbrida.**
   - Cada dedo regulado cierra rápido (SPEED 100) hasta su ángulo de aproximación, que queda justo antes del objeto,
     y desde ahí sigue lento (40) sin pararse. Es la conmutación de velocidad del Exp 2.
   - Al tocar, cada dedo se **congela** donde está: el primero en llegar no empuja el objeto ni el brazo mientras
     llegan los demás. En el reto, el dedo que llegaba antes empujaba el objeto y el brazo cedía hasta 8 mm (§87).
   - Contacto es que la fuerza suba 60 gf sobre su mínimo de los últimos 0.3 s, ignorando el arranque.
2. **Apriete en coordenadas** (A3, A4 y B1), no un lazo de fuerza por dedo:
   - **apriete** = (pulgar + dedos de enfrente) / 2, a la consigna del objeto;
   - **balance** = pulgar − dedos, a lo que marca al establecerse el agarre (no a 0: los sensores no son
     comparables entre sí);
   - **reparto** = índice − medio, solo en el modo 2, con la misma regla;
   - el PI en gramos de cada coordenada da el cambio de fuerza de cada dedo, que pasa a comando según el modo:
     - modo 1: con la K del RLS (A4);
     - modo 2: con la ganancia propia de cada dedo, como en B1, porque la J de 3×3 medida no es autoconsistente;
   - el vector se escala a `dq_max` y cada dedo da su cuanto (5 cerrando, 3 abriendo) con acumulador de resto;
   - el control solo avanza con muestra fresca y respeta el periodo refractario de 0.2 s;
   - `Mano.cerrar` vuelve cuando el apriete está en su banda (o pasados 3 s) y el lazo sigue en su hilo hasta abrir.
   - **Establecido el agarre, solo el apriete**, con banda de ±50 gf. El balance y el reparto se quedan como
     estaban al establecerse (`sostener_solo_apriete`, `banda_apriete_sostener_gf`).
3. **Reacciones**, con los dos detectores de A1:
   - **el actuador cede** (`POS` retrocede contra su comando y sigue atrás 0.3 s después): aflojar ese dedo y bajar
     la consigna un 15 %. Si vuelve antes, solo se apunta (`cede_breve`);
   - **el objeto se escapa** (cae la fuerza en todos los dedos con `POS` quieto): subir la consigna un 25 %, hasta
     1.3 veces la del objeto y siempre por debajo de 0.9 veces el techo;
   - **guarda**: un dedo por encima de 700 gf en 3 lecturas seguidas: aflojar. Si salta más de 3 veces en 2 s, fallo
     y salida segura.
4. **Objeto perdido** (`Mano.escapado`, lo usa el supervisor del reto). Se da por perdido si pasa una de estas cosas:
   - todos los dedos por debajo de 80 gf durante 0.2 s;
   - el pulgar, o todos los dedos, vacíos en su tope;
   - el apriete por debajo del 35 % de la consigna durante 0.6 s.

| Objeto (mano) | Modo | Dedos regulados | Consigna del apriete |
|---|---|---|---|
| cubo (der) | 2 | pulgar, índice, medio | 550 gf |
| esfera (der) | 2 | pulgar, índice, medio (el anular va por `FORCE_SET`, como en el reto) | 550 gf |
| cilindro (izq) | 1 | pulgar, índice | 400 gf |
| prisma tumbado (izq) | 2 | pulgar, índice, medio | 420 gf |
| prisma de pie (izq) | 1 | pulgar, índice | 380 gf |

### Decisiones que difieren del banco del regulador

- **Sin tara por hardware.** No se escribe `FORCE_CLB`, porque cambiaría el cero que usa el brazo firmware (su
  calibración del residuo) y contaminaría la comparación. El cero se resta por software, medido con la mano abierta
  y al menos 10 s después de soltar (E3.5). Se mide en cada preforma que cumpla esas condiciones; si no, se queda el
  anterior.
- **`FORCE_SET` 800 en bruto y techo 700** (el banco usa 700 y 600). En el modo 2 el pulgar carga con el índice y el
  medio: con el cubo da una mediana de 475 gf y picos de 650–790 con el firmware. 800 es el `force_set_max` del reto.
  Si el regulador funciona bien, el apriete se queda en su consigna, muy por debajo.
- **Detector de resbalón en espera tras mandar abrir.** Durante su ventana más el viaje del dedo (0.8 s) no mira ese
  dedo. A velocidad lenta, el dedo sigue retrocediendo después de que el comando de abrir sale de la ventana del
  detector, y en simulación encadenaba resbalones falsos, cada uno aflojando otra vez.
- **Una regla más de objeto perdido** (el apriete al 35 % durante 0.6 s). Sin objeto, la flexión propia de los dedos
  al cerrarse supera el umbral fijo de 80 gf; el pulgar ya marca ~100 cerca de su tope.
- **Sosteniendo, solo el apriete y con banda de 50** (desde el 2026-11-25). En los dos primeros bancos con el cubo
  (`datos/banco_20261125_095047_cubo` y `104925`), el regulador no asentaba durante el transporte:
  - mandaba 17–58 órdenes por dedo en los ~26 s de transporte y sostén; el firmware, ninguna;
  - la σ del reparto era de 78–115 gf (firmware 6–40), la del balance 49–65 (8–30) y la del apriete 31–57 (16–28).

  La causa es el cuanto del dedo contra un cubo de PLA rígido. Medido en esos bancos con los escalones aislados del
  lazo, un dedo da 31 gf por unidad el índice, 25 el medio y 10.8 el pulgar, unas 2 veces la J del banco. Un escalón
  de 5 vale ~155 gf en el índice, y el reparto con banda de 40 no podía asentarse. Es lo que avisa el propio `PI` del
  núcleo: la banda muerta tiene que ser del tamaño del cuanto. Con el apriete solo, un escalón de un dedo lo mueve
  ≤ ~60 gf, menos que la banda entera de 100.
- **El resbalón tiene que durar 0.3 s.** En esos bancos, el pulgar retrocedía justo 10 cuentas en el transporte y
  volvía en ~0.1 s. El detector lo daba por resbalón y bajaba la consigna a 468 gf el resto del agarre (2 de 4
  ensayos con regulador). Un resbalón de verdad (~50 cuentas) no vuelve. El firmware se mide con la misma regla.

## De dónde salen los números

- **Ángulos de contacto** (`aprox`) y **consignas** (`fuerza_gf`): de las tiradas de nivel 2 del 15 al 19 de
  noviembre, leyendo en `manos.csv` dónde sube la fuerza de cada dedo al cerrar y el apriete durante los transportes
  que acabaron bien. Valores (mediana [rango]):
  - **cilindro** (izq): índice 720 [710–727], pulgar 389 [370–394]; apriete 397 [303–421].
  - **cubo** (der): medio 715, índice 743, pulgar ~457; apriete 585 [391–923].
  - **esfera** (der): medio 639 [599–722], índice 643 [567–706], pulgar ~642; apriete 583 [96–777].
  - **prisma tumbado** (izq): medio 676 [666–688], índice 706, pulgar 334 [303–352], junto a su tope 300; apriete
    422 [261–642].
  - **prisma de pie** (izq): índice 702, pulgar 342; una sola tirada con transporte (322).

  El margen de aproximación es de ~80 cuentas en los dedos y ~100 en el pulgar: en vacío y a SPEED 100 los dedos
  marcan hasta 100–190 gf en movimiento, y el arranque da un pico durante las primeras ~80 cuentas.
- **Ganancias, cuantos, refractario, detectores y J:** de `inspire_hand_interface/Control/regulador_results.md`,
  medidos con la mano derecha del robot. La izquierda (cilindro y prisma) **no está caracterizada**: parte de la J
  de la derecha, el RLS la corrige en el modo 1 y en el modo 2 solo se usa la diagonal.

## La copia del regulador

`regulador/` es una **copia fija** de `Control/nucleo.py` y `Caracterizacion/hand_modbus.py`, sin cambiar una línea
(`regulador/ORIGEN.md`: el commit y los sha256). Una mejora del regulador en su repo no llega sola: hay que volver a
copiar los ficheros.

## Simulación y pruebas

`agarre/sim_mano.py` es la mano simulada del reto con lo que el lazo necesita ver:

- `POS_ACT`;
- publicación cada 30.7 ms por dedo;
- cuanto mínimo;
- velocidad por dedo;
- relajación de la fuerza tras moverse;
- ruido;
- un objeto que **flota** entre el pulgar y los dedos, sujeto por un muelle. Así salen los acoplamientos que midió el
  banco: cerrar el índice sube la fuerza del pulgar y baja la del medio.
- el **cubo con la planta medida en el robot** (`PLANTA`): pulgar 10.8, índice 31 y medio 25 gf por unidad, e
  índice↔medio −5.6;
- `vaiven`: una fuerza que empuja el objeto entre los dedos, como el brazo en el transporte. Con la configuración
  anterior reproduce lo del robot (21–26 órdenes en 5–6 s y σ del reparto 25–98 gf); con la de ahora, 0–5 y ~6;
- `empujar(dedo, du, sostenido)`: un tirón que echa atrás un dedo; o vuelve solo, o se queda (resbalón de verdad).

No es la física del agarre (el objeto no gira ni rueda), así que sirve para probar el lazo, sus fases y sus
detectores, no para predecir la tasa de éxito. Esa sale del banco en el robot.

```bash
MUJOCO_GL=egl python3 -m pytest -q tests/
```

En `tests/` hay dos grupos:

- **pruebas del lazo** sobre la mano simulada, sin OpenGL:
  - el agarre de los cinco objetos;
  - la caída de fuerza compensada;
  - el resbalón y la reacción de apretar;
  - el objeto perdido;
  - la guarda;
  - que el brazo firmware escribe en la mano exactamente lo mismo que la `Mano` del reto;
- **pruebas de punta a punta en el mundo simulado del reto:** el banco con los dos brazos y el nivel 2 con la mano
  regulada.
