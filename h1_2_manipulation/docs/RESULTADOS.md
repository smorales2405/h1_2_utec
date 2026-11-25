# Resultados: la manipulación con el regulador de fuerza

Registro de las pruebas del regulador de fuerza (`agarre/lazo.py`, sobre la copia fija de `regulador/`) en el reto de
manipulación, y de lo que se cambió por cada una. Lo más reciente va al final. El README explica cómo funciona; aquí
queda qué salió y por qué se cambió.

## Cómo se añade una prueba

Una sección nueva al final, numerada, con:

- **fecha, carpeta y código:** `datos/<carpeta>` (se trae con `./desplegar.sh --traer`; los datos no se suben al
  repo) y el commit desplegado, o qué había sin commit;
- **qué se probó:** objeto, ensayos, brazos, opciones (`--sostener-s`, `--tol-sitio-mm`...) y lo que vio el usuario;
- **la tabla del banco:** `python3 analisis/comparar.py datos/<carpeta>`, con las medidas que importen;
- **lo que se vio en los datos** y **lo que se cambió** (con el motivo y su comprobación en simulación).

Protocolo de comparación (§8 del plan del regulador): una sola tanda, brazos intercalados en bloques equilibrados.
**No se juntan ni se comparan tandas distintas**, porque entre ellas cambian el montaje, el código y el día. Con 2
ensayos por brazo, los p de la tabla no dicen nada: están por completitud.

## Resumen

| # | Fecha | Prueba | Regulador | Firmware | Qué cambió después |
|---|---|---|---|---|---|
| 1 | 2026-11-24 | Simulación | — | — | (primera versión) |
| 2 | 2026-11-25 | Banco cubo, sin recolocar | 2 de 2 | 2 de 2 | Recolocar sobre la cruz y observación final |
| 3 | 2026-11-25 | Banco cubo, recolocando | 2 de 2 | 2 de 2 | Sosteniendo, solo el apriete; resbalón de 0.3 s |

## 1. Validación en simulación (2026-11-24)

Commit `02592c6`. La mano simulada de `agarre/sim_mano.py` (la planta del banco del regulador, con el objeto
flotando entre los dedos) y el mundo simulado del reto.

- **Los cinco objetos** (cubo, esfera, cilindro, prisma tumbado y de pie): cada dedo regulado toca y se congela, y el
  apriete se establece 0.6–0.8 s después del último contacto, con picos ≤ ~1.12 veces la consigna.
- **Reacciones:**
  - el objeto que se escurre se aprieta y se recupera;
  - el objeto quitado se da por perdido en ~0.8 s;
  - la guarda afloja por encima del techo.
- **El brazo firmware escribe en la mano lo mismo que la `Mano` del reto**, byte a byte; el lazo solo mira.
- **Banco simulado** de 4 ensayos con el cubo, todos sostenidos:

  | | Regulador | Firmware |
  |---|---|---|
  | Apriete sostenido (gf, consigna 550) | 541 | 664 |
  | Tiempo de cierre (s) | 3.8 | 2.5 |

Quedaba sin probar en el robot: la detección de contacto con los 100–190 gf que marcan los dedos moviéndose en vacío,
la mano izquierda (sin caracterizar), los tiempos de Modbus, la cámara externa y el despliegue.

## 2. Primer banco en el robot: cubo, sin recolocar (2026-11-25 09:50)

`datos/banco_20261125_095047_cubo`, commit `02592c6`. `./banco.sh --banco cubo --ensayos 4 --arnes`: mano derecha,
sostenido 3 s sobre la bandeja D y devuelto a su sitio; cada ensayo encontraba el cubo donde lo había dejado el
anterior. Calendario: firmware, regulador, firmware, regulador.

**4 de 4 sostenidos.** El despliegue, la cámara externa (fotos y clips de cada agarre) y la detección de contacto
funcionaron en el robot.

| | Firm 1 | Reg 2 | Firm 3 | Reg 4 |
|---|---|---|---|---|
| Pico del apriete al cerrar (gf) | 736 | 580 | 1145 | 559 |
| Apriete sostenido (gf) | 444 (sin índice) | 552 | 772 | 459 |
| Tiempo de cierre (s) | 2.54 | 3.22 | 2.41 | 3.02 |
| Cubo desplazado al devolverlo (mm / grados) | 15.5 / −14.1 | 16.4 / −10.4 | 14.7 / −6.1 | — |

- **El regulador cierra en la consigna:** picos de 559–580 gf frente a 736–1145 del firmware, a cambio de ~0.6 s más
  de cierre.
- **Agarre descentrado en los ensayos 1 y 2.** El usuario vio la mano sujetar el cubo más hacia la izquierda del
  robot y no en su centro; en los ensayos 3 y 4, centrado.
  - Pasó con los dos brazos (el 1 es firmware y el 2 regulador), así que viene del plan o de la percepción del reto,
    no del regulador. Es el «pequeño desfase hacia la izquierda del robot» del §60 del reto.
  - En el ensayo 1 el índice quedó al borde y salió flojo al cerrar (−28 gf): el cubo lo llevaron el pulgar y el
    medio.
- **Devolverlo mueve el cubo** unos 15 mm y 6–14° (horario) por ensayo, con los dos brazos por igual. El cubo visto
  pasó de −3° a −33° en cuatro ensayos, y el agarre cambiaba con dónde y cómo estaba.
- **En el ensayo 4 (regulador)**, el pulgar retrocedió 10 cuentas en el transporte (el umbral justo) y el detector de
  resbalón bajó la consigna a 468 gf hasta el final.

**Cambios:** el banco pide poner el objeto sobre la cruz de cinta antes de cada ensayo y lo comprueba con la cámara
(tolerancia de 8 mm y 5°). Tras cada ensayo, también tras el último, mira dónde quedó el objeto (`agarre/banco.py`).

## 3. Intento interrumpido (2026-11-25 10:34)

`datos/banco_20261125_103404_cubo`: salida segura al arrancar, porque `arnes.py` aún estaba en COLGAR. No se movió
nada.

## 4. Segundo banco: cubo, recolocando sobre la cruz (2026-11-25 10:49)

`datos/banco_20261125_104925_cubo`, con la recolocación del §2 (sin commit entonces; va en el del §4).
`./banco.sh --banco cubo --ensayos 4 --arnes`; el usuario puso el cubo sobre la cruz cuando se le pidió. Calendario:
regulador, firmware, regulador, firmware.

**4 de 4 sostenidos.**

| | Reg 1 | Firm 2 | Reg 3 | Firm 4 |
|---|---|---|---|---|
| Pico del apriete al cerrar (gf) | 638 | 817 | 558 | 863 |
| Apriete sostenido (gf) | 544 | 573 | 448 | 650 |
| Tiempo de cierre (s) | 3.72 | 2.48 | 3.18 | 2.54 |
| Cubo desplazado al devolverlo (mm / grados) | 10.7 / −5.7 | 9.8 / −1.4 | 13.2 / −4.1 | 10.4 / −1.8 |
| Apartado de su sitio al empezar (mm) | 0 | 4.3 | 7.5 | 4.6 |

- **La cruz y la percepción.** Antes de ponerlo, el robot vio la cruz en (0.4612, −0.0374); con el cubo encima, lo
  vio a 1.5 mm de ella. La percepción del cubo (la tapa en el color) coincide con la cruz. El desfase a la izquierda
  del §2 no viene de dónde se ve el cubo, sino de la calibración cámara–brazo, del modelo del brazo o de dónde quedan
  los dedos frente al gemelo. Como la cruz y el cubo los ve la misma cámara, esta medida no separa esas causas.
- **Recolocar.** Se pidió tras los ensayos 1 y 3 (los del regulador, el cubo a 10 y 20 mm de su sitio, sobre todo
  alejado del robot). Tras los del firmware quedó dentro de los 8 mm. Recolocado por el usuario, quedó a 4.3 y 4.6 mm;
  el ensayo 3 empezó a 7.5 mm, aceptado con la tolerancia de 8.
- **El regulador sigue cerrando en la consigna** (558–638 gf frente a 817–863), con 0.6–1.2 s más de cierre.
- **Sosteniendo, el regulador no asentaba.** En los dos bancos, durante el transporte y el sostén (~26 s), mandó
  17–58 órdenes por dedo; el firmware, ninguna:

  | σ en transporte y sostén (gf) | Regulador | Firmware |
  |---|---|---|
  | Reparto (índice − medio) | 78–115 | 6–40 |
  | Balance (pulgar − dedos) | 49–65 | 8–30 |
  | Apriete | 31–57 | 16–28 |

  El índice y el medio se pasaban el cubo del uno al otro: la fuerza del índice iba de 80 a 400 gf mientras la del
  medio hacía lo contrario.
- **La causa, medida.** Con los escalones aislados del lazo en los dos bancos, un dedo contra el cubo de PLA da (gf por
  unidad de comando):

  | Pulgar | Índice | Medio | Índice ↔ medio |
  |---|---|---|---|
  | 10.8 | 31 | 25 | −5.6 |

  Es ~2 veces la J del banco del regulador. Un escalón del índice (5 unidades) vale ~155 gf, cuatro veces la banda
  del reparto (40), así que el lazo no podía asentarse. Es lo que avisa el `PI` del núcleo: la banda muerta tiene que
  ser del tamaño del cuanto del dedo.
- **Resbalón falso otra vez.** En el ensayo 3, el pulgar retrocedió 10 cuentas en el transporte y volvió en ~0.1 s;
  la consigna bajó a 468 gf y sostuvo con 448. Van 2 de 4 ensayos con regulador entre los dos bancos.

**Cambios** (`config/regulador.yaml`, `agarre/lazo.py`):

- **Establecido el agarre, solo el apriete** (`sostener_solo_apriete`), con banda de ±50 gf
  (`banda_apriete_sostener_gf`). El balance y el reparto se quedan como estaban al establecerse. Un escalón de un dedo
  mueve el apriete ≤ ~60 gf, menos que la banda entera (100).
- **El resbalón tiene que seguir 0.3 s después** (`resbalon_persistir_s`). Si vuelve antes, se apunta como
  `cede_breve` (columna `ceden_breves`) sin aflojar. El firmware se mide con la misma regla.
- **Tolerancia del sitio a 5 mm.**
- **Simulación:** el cubo con la planta medida (`PLANTA` en `agarre/sim_mano.py`) y un `vaiven` que empuja el cubo
  entre los dedos como el brazo en el transporte:

  | Simulación, 5–6 s sosteniendo | Configuración anterior | Configuración nueva |
  |---|---|---|
  | Órdenes | 21–26 | 0–5 |
  | σ del reparto (gf) | 25–98 | ~6 |

  Pruebas nuevas: `test_sostiene_el_cubo_sin_vaiven_de_los_dedos` y `test_un_tiron_del_pulgar_no_es_resbalon`.

## Pendiente

- **Probar en el robot los cambios del §4:** banco del cubo de 4 ensayos. Que el regulador apenas mande órdenes
  sosteniendo, que no haya resbalones falsos y que el apriete sostenido se quede cerca de 550 gf. Después, la tanda de
  20 (10 y 10).
- **Los demás objetos:** esfera (derecha), cilindro y prisma (izquierda). La mano izquierda no está caracterizada:
  usa la J de la derecha (el RLS la corrige en el modo 1; en el modo 2, solo la diagonal). Con objetos rígidos, medir
  su planta como la del cubo.
- **El desfase a la izquierda del robot**, que es del reto: la calibración cámara–brazo, el modelo del brazo o los
  dedos frente al gemelo.
- **Un punto ciego del detector de resbalón:** durante los 0.8 s siguientes a mandar abrir un dedo no mira ese dedo,
  así que un resbalón real justo entonces no se vería.
