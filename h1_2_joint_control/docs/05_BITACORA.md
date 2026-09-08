# Bitácora

Un apunte por sesión: qué se midió, con qué, y qué se decidió. Los números
crudos están en `logs/index.csv`, una línea por ensayo.

---

## 2026-09-04 — Diagnóstico y puesta a punto de la herramienta

**Máquina**: portátil `mito`, Ubuntu 22.04, ROS 2 Humble, NIC `enp12s0`
(192.168.123.51). **Robot**: H1-2, PC1 en .161 y PC2 en .164, ambos accesibles.

### Estado encontrado

* `/lowstate` a 500 Hz, `mode_machine = 6`, los 14 motores de los brazos en
  `mode = 1` (habilitados), temperaturas de 46 a 51 °C en reposo.
* `/lowcmd`: **1 publicador a 500 Hz que no somos nosotros**.
* `motion_switcher.CheckMode()` → `name = 'ai'`: el controlador de locomoción
  está activo. Es quien publica en `/lowcmd`.
* `/arm_sdk`: 1 suscriptor, 0 publicadores. Libre.
* Postura de partida de los brazos: codo izquierdo 1.490 rad (85.4°), codo
  derecho 1.344 rad (77.0°), resto cerca de cero.

### Conclusión del diagnóstico

Los dos síntomas —codo que no se mueve, muñecas que vibran— se explican por la
disputa de `/lowcmd` con el servicio `ai`. Ver
[`01_DIAGNOSTICO.md`](01_DIAGNOSTICO.md).

Comprobado además que en este portátil no están instalados `unitree_sdk2py` ni
`xr_teleoperate`, y que `arm_joint_test.py` lleva cableada una ruta de la otra
máquina (`/home/utec/…`): aquí habría fallado en el `import`. El ensayo que dio
«el codo no se mueve» fue en la máquina `utec`.

### Hallazgos sobre las herramientas

1. **Matar procesos `ros2` de la CLI degrada el dominio DDS local.** Medido:
   `/lowstate` cae de 500 Hz a 62.6 Hz justo después de que `subprocess` mate un
   `ros2 topic hz`, y tarda unos 10 s en recuperarse. `00_diagnose.py` se
   reescribió para medir todo dentro del proceso.
2. **Cerrar mal un nodo rclpy rompe la ejecución siguiente.** Destruir el nodo
   con el ejecutor todavía girando deja el participante DDS a medias; el
   siguiente proceso no recibe `/lowstate`. `H12Client.shutdown()` apaga en
   orden.
3. **El CRC en Python puro no da para 250 Hz.** La versión bit a bit tarda
   2.25 ms por mensaje y el lazo se quedaba en 185 Hz. Tabulado (el paso del
   registro es lineal sobre GF(2)) baja a 0.29 ms y el lazo sube a 248 Hz.
   Verificado contra la implementación literal en 300 casos aleatorios y contra
   los valores que producía el CRC de `test_mandar_modificado.py`.
4. **Frecuencia máxima del lazo en Python**: 250 Hz con 0.5 % de ciclos tarde;
   500 Hz con 2.2 %. Se deja 250 Hz por defecto, que es además la de
   `xr_teleoperate`.
5. **Suscribirse a `/lowstate` y `/lowcmd` a la vez** (500 Hz cada uno) satura
   un `SingleThreadedExecutor`: la tasa de `/lowstate` se lee falseada a 72 Hz.
   El diagnóstico suelta `/lowcmd` en cuanto lo ha contado.

### Validación del software, sin tocar el robot

Cadena completa ejercitada con `--dry-run` (publicación en un tópico que nadie
escucha): construcción del mensaje, CRC, lazo a frecuencia fija, registro,
métricas, CSV y gráficas. Todo correcto.

Corregido de paso un fallo real: el `t` de las muestras contaba desde que se
tomaba el control y no desde que arrancaba el registro, con lo que el instante
del escalón que se pasa a `metrics.step_response()` no cuadraba con los datos y
las métricas de escalón salían mal.

### Batería de pruebas sin robot

`scripts/99_selftest.py`, 32 comprobaciones, todas en verde. Cubre:

* la tabla de articulaciones contra el URDF, campo a campo;
* el CRC tabulado contra la implementación literal (200 casos aleatorios) y
  contra los valores que producía el CRC de `test_mandar_modificado.py`, ya
  validado con el robot real; incluye una prueba de que el orden `tau`/`kp`/`kd`
  del *layout* importa;
* las métricas contra señales de respuesta conocida: un seno de 20 Hz y
  amplitud 1 debe dar `chatter_dq = 0.707`; un segundo orden con ζ = 0.5 debe
  dar 16.3 % de sobreimpulso (sale 16.3 %);
* que la `dq` de cada trayectoria es de verdad la derivada de su `q`;
* la carga de `gains.yaml` y sus topes de seguridad.

La prueba del URDF encontró un fallo real: los topes de tobillos y
`wrist_pitch` estaban redondeados (0.524 en vez de 0.523598), y tres de ellos
quedaban **por fuera** del límite declarado. Corregido con los valores exactos.

### Dos correcciones de seguridad durante la revisión

1. **El aborto dejaba de publicar de golpe.** Al saltar un tope, el lazo salía
   del bucle y el robot se quedaba con el último peso de `arm_sdk` que le
   habíamos mandado, posiblemente 1.0. Ahora el lazo sigue publicando mientras
   baja el peso en rampa y solo se para al llegar a 0.
2. **El instante del escalón no cuadraba.** El `t` de las muestras contaba
   desde que se tomaba el control, no desde que arrancaba el registro, así que
   `metrics.step_response()` partía los datos por donde no era.

### Corte de la sesión

A las ~16:10 el robot dejó de responder: ni PC1 (.161) ni PC2 (.164) contestan
al ping, aunque el enlace Ethernet sigue activo (`LOWER_UP`) y `/lowcmd` había
estado a 500 Hz un minuto antes. Los ensayos sobre el robot real quedan
pendientes de que vuelva a estar disponible.

Antes de eso, con el robot todavía en línea, se llegó a validar la cadena
completa en `--dry-run`: mensaje, CRC, lazo a 250 Hz, registro, métricas, CSV y
gráficas. El ruido de fondo medido en reposo (temblor `chatter_dq`) fue de
0.008–0.010 rad/s en codo y muñeca, que es la línea base del encoder y encaja
con el umbral de «limpio» (< 0.02) de la metodología.

### Sesión sobre el robot real (16:15 – 16:45)

Robot **colgado del arnés**, confirmado por el operador. Secuencia y hallazgos:

1. **Peldaños 2 y 3 por `arm_sdk`** (peso 0 y peso 1, sosteniendo sin mover):
   limpios. Deriva de 0.04 mrad, temblor 0.005–0.017 rad/s en las 14. Pero la
   prueba no distingue «funciona» de «me ignoran», porque no se comanda nada.
2. **Primer movimiento por `arm_sdk`: no se mueve.** `L_wrist_yaw`, escalón de
   0.10 rad, peso 1: el error se queda en los 100 mrad comandados y el par en
   0.06 Nm.
3. **Cinco variantes del mensaje `arm_sdk`**, incluida una copia literal del
   ejemplo oficial (sin `mode`, sin `mode_machine`, sin CRC, a 50 Hz): ninguna
   mueve nada. Nuestro publicador sí empareja con el suscriptor del robot
   (`get_subscription_count() == 1`), así que el mensaje llega y el servicio no
   lo aplica.
4. **La causa, leyendo `/lowcmd`**: el servicio `ai` publica `kp = kd = tau = 0`
   en los 27 motores. El robot está en reposo con los motores habilitados pero
   sin par. Lo más probable es que `arm_sdk` solo se mezcle cuando el
   controlador está realmente controlando. No comprobado con el robot activo.
5. **Esto además afina el diagnóstico original**: lo que compite en `/lowcmd` no
   es un comando de «mantener postura» sino uno de ganancia CERO. Publicar a
   250 Hz contra 500 deja a la articulación con un tercio del kp pedido; a
   500 contra 500, la mitad, conmutando entre kp y 0 a centenares de hercios.
   Eso es exactamente «no llega» y «vibra».
6. **Modo debug**: `06_debug_mode.py enter` → `/lowcmd` a 0 Hz. Primer escalón
   comandado (`L_wrist_yaw`, 0.10 rad, kp=50): subida 48 ms, 0 % de
   sobreimpulso, 0.16° de error final, sin temblor. **Funciona.**
7. **Barridos del codo y de las 14 articulaciones**: ver
   [`06_RESULTADOS.md`](06_RESULTADOS.md).
8. Al terminar, `06_debug_mode.py exit`: controlador `ai` restaurado y
   verificado (500 Hz, kp = kd = 0). El robot queda como se encontró.

### Tres correcciones que salieron de los ensayos

1. **El sentido del ensayo estaba mal elegido.** Los brazos son especulares, así
   que `+0.12 rad` separa el izquierdo del cuerpo y mete el derecho contra el
   torso. `R_shoulder_roll` daba +6.25° de error y 15 Nm, y con compensación de
   gravedad llegó a 29.6 Nm y saltó la protección. No era el controlador: era un
   choque. Corregido con `pick_amplitude()`, que va hacia donde queda más
   recorrido. Con el sentido bueno: 1.67° y 4.3 Nm.
2. **`tau_ff` se aplicaba desde el principio del movimiento**, sumándose al par
   que el PD ya generaba para recorrer el camino. Ahora se mete en rampa
   después de llegar.
3. **Salir del control en `lowcmd` era un corte seco.** Ahora la «autoridad»
   significa lo mismo en los dos canales: en `arm_sdk` es el peso de mezcla, en
   `lowcmd` escala kp, kd y tau_ff. Entrar y salir es una rampa en ambos.

La protección de par actuó dos veces y en las dos soltó el brazo correctamente.
Tras soltar, el puente de motores del robot los deshabilita solo (`mode = 0`) y
los brazos se quedan donde estaban.

### Añadido después de parar los ensayos

`scripts/08_read_state.py`: lectura rápida del estado, equivalente al
`arm_joint_test.py read` de la otra máquina pero sin depender de
`unitree_sdk2py`. Menos de 1 s, solo lectura, con `--watch`, `--json` y
selección de articulaciones.

Escribiéndolo salió un fallo que afectaba a **todos** los scripts: bajo `rclpy`,
**SIGTERM cierra el contexto DDS pero no interrumpe el bucle del hilo
principal**. Medido con un caso mínimo:

```
señal 2  (SIGINT ):  KeyboardInterrupt capturado a los 1.00 s  ✔
señal 15 (SIGTERM):  BUCLE NO INTERRUMPIDO tras 5 s   rclpy.ok()=False
```

En un script que está mandando al robot eso es serio: un `kill <pid>` o un
`timeout` dejaban el proceso **vivo y publicando**, sin nadie mirando. Se
detectó porque tres procesos de `--watch` sobrevivieron a `timeout` y hubo que
matarlos con `kill -9`.

Corregido con `_install_sigterm_handler()` en `client.py`: SIGTERM lanza el
mismo `KeyboardInterrupt` que produce Ctrl-C y dispara la salida ordenada.
Verificado: `02_move.py` bajo SIGTERM vuelve a la postura inicial, baja las
ganancias en rampa e informa del estado del lazo. SIGINT ya funcionaba bien, así
que la vía de escape documentada (Ctrl-C) nunca estuvo rota.

### Postura de ensayo y topes de autocolisión (aportados por el operador)

Ángulos de hombro para que los brazos no toquen el torso:

* Para ensayar **cualquier otra** articulación: `R_shoulder_roll` a **−18°** y
  `L_shoulder_roll` a **+18°**.
* Para ensayar **el propio** `shoulder_roll`: como mucho hasta **−10°** (derecho)
  y **+10°** (izquierdo).

Llevado a `config/gains.yaml` como `test_posture_deg` y `soft_limits_deg`. Esto
cierra el agujero que produjo el choque de `R_shoulder_roll`: antes la postura
de partida era «donde estuviera el robot», y con los hombros cerca de cero
cualquier ensayo de roll iba contra el cuerpo.

Desde ±18° quedan 8° de recorrido hacia el cuerpo, suficientes para la amplitud
de 0.12 rad (6.9°) que se venía usando.

Detalle en [`07_POSTURA.md`](07_POSTURA.md), que documenta además algo que
faltaba por escrito: **qué hacen las otras trece articulaciones mientras se
prueba una**. Se sostienen clavadas en la postura que tenían al ceder el
control, con sus ganancias del conjunto activo; no se dejan libres ni se mandan
a un ángulo absoluto. En canal `lowcmd` las piernas sí quedan libres
(`--legs free`), que es lo que ya hacía el servicio `ai`.

### Validación de la postura de ensayo (17:57)

Modo debug, canal `lowcmd`, ganancias de `xr_teleoperate`.

Primera pasada, con la consigna a pelo: **el hombro no llega**. Se le pide
−18.0° y se queda en −15.5°, con 6.4 Nm de par. `tau_g/kp` = 45.7 mrad = 2.6°,
que es exactamente el error medido. Como la postura es anticolisión, quedarse
2.5° más cerca del cuerpo no vale.

Corregido: `go_to_test_posture()` cierra el lazo (manda, mide, desplaza la
consigna, hasta 6 iteraciones o 0.5° de tolerancia). Dos pasadas de validación:

| | `L_shoulder_roll` | `R_shoulder_roll` |
|---|---:|---:|
| pasada 1 | +18.15° | −18.38° |
| pasada 2 | +17.71° | −17.65° |

Dentro de ±0.4°. Temblor 0.009–0.013 rad/s (ruido de fondo), par 6.3–7.6 Nm de
40 disponibles. Robot devuelto a modo `ai` al terminar.

### Barrido de las 14 desde la postura validada (18:03)

12/14 dentro de tolerancia, pero el reparto cambia respecto al barrido anterior:
los codos y las seis muñecas bajan a **menos de 0.25°** de error final, y las
dos `shoulder_roll` suben a ~4°.

Esta vez **no es un choque**: la postura las deja a ±18° y el ensayo las aleja
hasta ±27°, donde el brazo está más horizontal y el par de gravedad pasa de 9 a
15 Nm. Comprobado con `07_gravity_ff.py`, y el modelo acierta a la décima:

| | ángulo | `tau_g` | err medido | `tau_g/kp` previsto | con `tau_ff` |
|---|---:|---:|---:|---:|---:|
| L_shoulder_roll | +27.3° | 9.63 Nm | +4.13° | +3.94° | +1.10° (73 %) |
| R_shoulder_roll | −27.0° | −10.36 Nm | −4.17° | −4.24° | −0.36° (91 %) |

Conclusión para la teleoperación: `shoulder_roll` irá retrasado y tanto más
cuanto más levantado el brazo. Ninguna ganancia razonable lo arregla; hace falta
`tauff`. Las otras doce no necesitan nada.

Robot devuelto a modo `ai` y verificado.

## 2026-09-05 — Sintonización completa de los 14 y barrido final

126 ensayos, sin un solo aborto. Robot colgado, canal `lowcmd`, postura de
ensayo aplicada. Detalle en [`06_RESULTADOS.md`](06_RESULTADOS.md) §7.

**Resultado**: mejora del 31 al 46 % en error de seguimiento respecto a
`xr_teleoperate`, y el barrido de comprobación da **14/14** dentro de
tolerancia. El error permanente de los `shoulder_roll` cae de ±4.2° a ±1.0°.

### Tres cosas que obligaron a cambiar el método sobre la marcha

1. **kp no tiene techo natural.** En hombros y codo el error lo domina la
   gravedad, `tau_g/kp`, así que baja monótonamente con kp y el barrido se va
   al borde siempre. El límite lo pone la saturación, no el seguimiento:
   `kp_max = tau_abort_fraction · tau_max / salto`. Con 0.10 rad: 280 en los
   hombros de 40 Nm, 126 en codo y hombro-yaw de 18 Nm.
   Esto descartó un kp = 280 que el barrido sin tope había propuesto para
   `L_shoulder_yaw`, que es un motor de 18 Nm, y marcó como excesivo el
   kp = 140 del codo que se había sintonizado el día anterior.

2. **La métrica de sobreimpulso estaba mal.** Ver §7 de resultados: se medía
   contra la consigna en vez de contra el valor final, y con gravedad eso mete
   la caída `tau_g/kp` en el denominador. Como la caída crece al bajar kp, el
   artefacto crecía al bajar kp y el resultado salía invertido respecto a la
   teoría: 42 % de "sobreimpulso" con kp=280 y 72 % con kp=140. Corregido, los
   `shoulder_roll` quedan en un 20 % real.
   Lo delató el propio dato: que bajar kp con kd fijo empeorara el
   sobreimpulso es imposible.

3. **El kd sintonizado depende de si se manda `dq_des`.** Chirp de 0.2 a 3 Hz
   en el codo:

   | | `dq_des` activa | `dq_des = 0` |
   |---|---:|---:|
   | kd = 13.5 | **16.49 mrad** | 35.49 mrad |
   | kd = 3.0 | 23.89 mrad | **26.81 mrad** |

   El orden se invierte. Con velocidad de referencia, kd alto gana un 31 %; sin
   ella pierde un 32 %. El conjunto `tuned` lleva el aviso en `gains.yaml`.

### Lo que NO resultó ser un problema

Sospechaba que los kd altos elegidos a 0.5 Hz estarían sobreajustados y
fallarían a 3 Hz. Medido con chirp: kd = 13.5 gana a kd = 3 en error (16.5
contra 23.9 mrad), en temblor y hasta en par de pico. La sospecha era
razonable y era falsa.

## 2026-09-08 — F0, y un criterio de aceptación que medía lo que no era

Ejecutada F0 con el robot conectado (publicación inerte, sin mover nada). El
criterio literal del protocolo **falla**: p99/T = 1.45 a 250 y a 500 Hz, y de 16
a 205 «muestras perdidas» de `/lowstate` por bloque de 120 s.

Y sin embargo el sistema es apto. Las dos razones son estructurales:

* la trayectoria se evalúa contra reloj (`func(now - t0)`), no contra contador
  de ciclos, así que un ciclo tarde publica la consigna correcta para ese
  instante;
* el lazo lee la última posición conocida en cada ciclo, así que perderse
  mensajes intermedios es inocuo.

La magnitud que sí importa es la **edad del estado al usarlo**, que nadie estaba
midiendo: mediana 0.37 ms, p99 1.23, máxima 3.17. A 0.5 rad/s eso son 1.59 mrad
en el peor caso, contra errores medidos de 5 a 32 mrad.

`10_loop_timing.py` mide ahora las dos cosas y reporta las dos conclusiones.

### Dos cosas que no se hicieron, a propósito

**No se adoptó la mejora de temporización.** Cuatro estrategias probadas; la
mejor (sueño + 1 ms girando) dio p99/T = 1.03 contra 1.11. Pero el mismo código
sin tocar dio 1.11 en una corrida y 1.45 en otra: la varianza entre corridas es
del tamaño del efecto. Adoptarla con N=1 sería el error que prohíbe la regla 5
del propio protocolo.

**No se tocó la máquina.** El gobernador está en `powersave` y `ulimit -r = 0`,
así que `SCHED_FIFO` necesitaría sudo. Ninguna mitigación se aplica porque el
criterio que importa ya se cumple.

### Corrección de una recomendación mía

Había propuesto saltarse F0 apoyándome en las medidas parciales. Era una mala
recomendación: la fase costó 20 minutos y produjo una corrección real del
criterio de aceptación. La lección vale para el resto del protocolo —
**comprobar que cada criterio mide la magnitud por la que uno se preocupa, y no
una correlacionada**.

## 2026-09-08 (tarde) — F2.1: dos hipótesis mías, las dos falsas

19 puntos del brazo izquierdo con `11_gravity_id.py`, cada uno alcanzado desde
los dos sentidos. Detalle en [`06_RESULTADOS.md`](06_RESULTADOS.md) §8.

**Hipótesis 1, mía, escrita antes de medir**: «una parte apreciable de los
3.74 Nm de discrepancia es fricción, no masa». **Falsa.** La fricción existe
—0.14 a 0.95 Nm según la articulación— pero separarla deja el error del modelo
en 0.96 Nm de media y 4.63 máximo. No explica el hueco.

**Hipótesis 2, del protocolo**: es la masa de la mano, que el URDF pone en 316 g
cuando la real son ~800 g. **También falsa.** El ajuste por mínimos cuadrados
sobre la gravedad limpia pide **1.9 kg**, más del doble de la real, y aun así
deja residuos estructurados: −1.13, −1.43, −0.96 Nm en `shoulder_pitch`, casi
constantes con el ángulo. Un parámetro que necesita un valor imposible está
absorbiendo otra cosa.

Lo que sí quedó medido y sirve: **el mapa de fricción estática**. Las peores son
`shoulder_yaw` (0.95 Nm) y `elbow` (0.81), los dos motores de 18 Nm, donde eso
es un 5 % del par disponible. Explica la dispersión entre pasadas de §6 y la
banda de pegado del codo, que llevaban sueltas desde el 09-05.

**Consecuencia**: F2.1 no se puede cerrar como está escrita. Ni es la mano ni
hay una rama de contingencia que aplique. Propuesta en §8 de resultados:
sustituir el modelo físico por una **regresión empírica de `τ_g(q)`**, que es
más barata, no depende de que el URDF sea correcto, y da exactamente el número
que la teleoperación necesita.

De paso, el portero de autocolisión actuó en 127 ciclos (0.4 %) durante la
corrida, recortando consignas incompatibles con el ángulo de codo. Funciona.

## 2026-09-08 (tarde, II) — identificación de masas: el modelo pasa el criterio

Cambiado el enfoque de F2 tras los dos fracasos de §8. En vez de corregir el
URDF a mano, **identificar los parámetros de masa desde medidas**, aprovechando
que el par de gravedad es lineal en ellos y que `pinocchio` da el regresor.

`12_gravity_map.py` sortea 30 configuraciones por el espacio de trabajo y lee
**las siete articulaciones en cada una** — el par que sostiene cada una está ahí
sin coste extra. 210 medidas en 3.5 min, contra los 19 puntos agrupados de la
mañana. `13_gravity_fit.py` ajusta con cresta hacia el prior del URDF.

**rms 2.574 -> 0.337 Nm. Máximo 8.167 -> 1.126.** Criterio del protocolo
(< 1.0 Nm): se cumple.

Validación cruzada: entrenamiento 0.330, prueba 0.354. Sin sobreajuste pese a
76 parámetros y 210 ecuaciones — la regularización hizo su trabajo. Todas las
masas positivas.

La masa de mano identificada es **1.138 kg**, contra 316 g del URDF. La mano
sola pesa ~800 g y F4.1 pide pesar «mano + conector + tramo de cable»: encaja.
Y contrasta con el ajuste de un solo parámetro de la mañana, que pedía 1.9 kg,
imposible — la diferencia es que aquí también se ajustan los eslabones del brazo
y la mano no tiene que absorber todo.

`shoulder_yaw` es la que menos mejora (68 % contra 87-90 % del resto), y encaja
con que sea la de mayor fricción: parte de su residuo no es gravedad.

Pendiente antes de usarlo: el brazo derecho (otros 4 min de mapeo), y meter
`tau_ff` en el lazo con su prueba de humo.

## 2026-09-08 (noche) — F2.3, y un fallo del barrido que llevaba dias ahi

Detalle en [`06_RESULTADOS.md`](06_RESULTADOS.md) §10.

**El fallo primero.** La primera pasada de F2.3 dio 52-84 % de sobreimpulso en
`L_shoulder_roll` contra el 19 % sin compensar. No tenía sentido, así que se
repitieron los mismos kp aislados: 0.8 %, 1.1 %, 13.5 %, 17.3 %. Los números
del barrido eran falsos.

La causa es la **regla 1 del propio protocolo**, que `09_tune_all.py` no
respetaba: cambiar kp con `q_des` desactualizado produce un salto de par
`(kp'−kp)·e`, y el ensayo siguiente empezaba con la articulación asentándose de
ese golpe. Corregido igualando la consigna a la posición medida antes de tocar
las ganancias; verificado que el barrido reproduce ahora los valores aislados.

Afecta a `09_tune_all.py` desde que se escribió, incluida la campaña del 09-05.
Aquellos usaban seno con `skip=0.5` y hay más de 1 s entre el cambio de
ganancias y el inicio de la medida, así que **probablemente** no están
contaminados — pero no se ha verificado, y hasta entonces es «probablemente».

**El resultado de F2.3.** Dos de los tres criterios se cumplen, y con **kp = 70,
una cuarta parte del actual**: error permanente 0.40° (criterio < 0.5°) y
sobreimpulso 1.4 % (criterio < 10 %). El tercero —ganador no en el borde— falla
en la letra, pero el ganador se ha movido del borde superior al inferior, que es
exactamente lo que predecía la hipótesis: kp estaba alto para tapar la falta de
compensación.

Codo y muñeca ya no necesitan sintonización: con gravedad compensada dan 0-0.3 %
de sobreimpulso en todo el rango de kp probado.

**Una sospecha mía más, también falsa**: que evaluar `g()` en `q_des` en vez de
en la posición medida adelantaría el par durante el movimiento. Medido: 17.9 %
contra 17.3 % de sobreimpulso. No hay diferencia.

## 2026-09-08 (cierre) — el barrido no es reproducible; F1 deja de ser opcional

Al extender F2.3 a las siete articulaciones, `L_shoulder_roll` con las MISMAS
ganancias dio sobreimpulsos de 0.8 %, 3.0 % y 29.5 % en tres corridas
distintas. Detalle en [`06_RESULTADOS.md`](06_RESULTADOS.md) §11.

Por el camino se encontraron y corrigieron dos artefactos de medida reales
—cambio de ganancias con `q_des` desactualizado, y repeticiones que empezaban
sin asentar— y aun así la dispersión persiste. Hay un tercer factor sin
identificar; la pista es que el error permanente a kp = 70 pasa de +8.00 a
+31.03 mrad, o sea que el residuo del modelo de gravedad es 4× mayor en la
corrida completa.

**Con esa dispersión ningún barrido puede elegir entre candidatos.** Las mejoras
del 0 al 20 % del resumen del brazo izquierdo están por debajo del ruido.

Es la hipótesis H1 del protocolo, y F1 —bloqueante -- es la fase que se saltó.
`tuned_gff` queda marcado como PROVISIONAL en `gains.yaml` y `tuned` sin tocar.

Lo que sí se sostiene de F2, porque viene de corridas aisladas y repetidas que
concuerdan entre sí: la identificación de masas (validada por dos brazos
independientes), que `τ_ff` quita el 85-98 % del error permanente, y que con
gravedad compensada `shoulder_roll` a kp = 70 da ~1-3 % de sobreimpulso contra
~17 % a kp = 280.

### Pendiente

- [ ] **F1 antes de seguir sintonizando.** Sin `δ_min` no se puede afirmar que
      un candidato sea mejor que otro.
- [ ] Aislar el tercer factor de la dispersión. Sospecha: el residuo del modelo
      de gravedad depende de la configuración por la que se pasa.
- [ ] Repetir el barrido del codo en dos o tres posturas más: todo lo medido lo
      está con el codo a 85° y el brazo colgando.
- [ ] Sintonizar hombros y muñecas con el mismo método.
- [ ] Compensación de gravedad con `pinocchio` y llevarla a `tauff_target` de
      `H1_2_ArmController`.
- [ ] Mandar la velocidad de referencia en la teleoperación (vale un 35 % del
      error de seguimiento).
- [ ] Comprobar si `arm_sdk` funciona con el robot activo, de pie.

---

<!-- Plantilla para las siguientes entradas:

## AAAA-MM-DD — Título

**Objetivo**:
**Comandos**:
**Resultado**: (tabla o pegar el resumen del script)
**Decisión**:
**Ficheros**: logs/…
-->
