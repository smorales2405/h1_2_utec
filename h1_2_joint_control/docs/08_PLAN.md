# Plan de ejecución — revisión del protocolo de continuación

Revisión de [`protocolo_continuacion_sintonizacion_H1-2.md`](protocolo_continuacion_sintonizacion_H1-2.md)
a la luz de lo medido los días 2026-09-04 y 05, más siete comprobaciones nuevas
hechas el 07 **sin tocar el robot**.

---

## 1. Veredicto sobre el protocolo

**El diagnóstico es correcto y el orden de fases también.** H1 (N=1) y H4
(falta compensación de gravedad) son efectivamente los dos que invalidan más
cosas, y F0/F1 como bloqueantes está bien planteado: la dispersión medida
—21.2 contra 35.8 mrad para el mismo ensayo— es del orden de las mejoras que
se reportan, así que hasta acotarla ninguna de ellas es defendible.

Lo que sigue son las siete cosas que he comprobado y que **cambian el plan**.

### 1.1 La infraestructura ya está aquí — F2 y F3 son más baratas de lo que el protocolo asume

| Comprobado | Resultado |
|---|---|
| `pinocchio` | **3.8.0 instalado**, con `hppfcl` |
| URDF con manos Inspire | `h1_2_description/urdf/h1_2.urdf`, 59 eslabones, 67.37 kg |
| Geometría de colisión | **50 tags `<collision>`**, 48 objetos cargables |
| Coste de `g(q)` | **8 µs** = 0.20 % del ciclo de 4 ms |
| Coste de un chequeo de colisión (251 pares) | **0.1 ms** = 2 % del ciclo |

El protocolo apunta a `smorales2405/h1_2_inspire_description` para F2. **No hace
falta**: el URDF local ya trae las manos y la geometría de colisión, y las dos
cosas que F2 necesita caben de sobra dentro del lazo de 250 Hz. Cargar el modelo
tarda decenas de segundos, así que se hace una vez al arrancar; a partir de ahí
es gratis.

### 1.2 La masa de la mano está mal en el URDF, y corregirla NO arregla el modelo

F2.1 sospecha que el URDF arrastra la masa del RH56DFX sin táctiles. Confirmado:
**la masa distal a `wrist_yaw` es 0.316 kg** cuando la mano real pesa ~800 g.

Pero validando el modelo contra los cuatro puntos que ya están en el repositorio:

| caso | q | τ medido | τ modelo | error |
|---|---:|---:|---:|---:|
| L_shoulder_roll | +13.1° | +4.22 Nm | +3.31 Nm | −21.6 % |
| R_shoulder_roll | −10.5° | −4.34 Nm | −2.76 Nm | +36.4 % |
| L_shoulder_roll | +27.3° | +9.63 Nm | +6.61 Nm | −31.4 % |
| R_shoulder_roll | −27.0° | −10.36 Nm | −6.62 Nm | +36.1 % |

Error máximo **3.74 Nm**, contra un criterio de aceptación de 1.0 Nm. Y
escalando la masa de la mano:

| masa de mano | error máx | error medio |
|---:|---:|---:|
| 316 g (URDF) | 3.74 Nm | 2.31 Nm |
| 632 g | 2.80 Nm | 1.64 Nm |
| 789 g (≈ la real) | 2.33 Nm | 1.30 Nm |
| 947 g | 1.86 Nm | 0.96 Nm |

**Ni con 947 g —más que la mano real— se llega al criterio.** El modelo
subestima sistemáticamente en los cuatro puntos y en los dos signos.

> **Corrección de dato en el protocolo.** F2.1 cita esos puntos como «±18° y
> ±27°». Los ángulos reales son **+13.1°, −10.5°, +27.3° y −27.0°**. Para
> validar un modelo de gravedad el ángulo *es* la entrada, así que la cita hay
> que corregirla antes de usarla.

### 1.3 El confundido que el protocolo no contempla: `tau_est` no es `τ_g`

Ésta es, en mi opinión, la omisión más importante del documento.

Lo que `07_gravity_ff.py` mide es el par que sostiene la postura, y eso es
**gravedad más fricción estática**, no gravedad. La fricción se opone al último
sentido de movimiento, así que en una medida de una sola aproximación entra con
signo fijo. Que el modelo subestime en los cuatro puntos y en los dos signos es
exactamente la firma de eso.

Si se identifica la masa con estas medidas sin separar la fricción, **la
fricción se absorbe dentro de la masa**. El modelo cuadrará en estático y
sobre-compensará en movimiento, que es justo cuando `τ_ff` tiene que funcionar.

**Cambio obligatorio a F2.1**: cada postura de validación se mide **aproximando
desde los dos sentidos**.

```
τ_g       = (τ_subiendo + τ_bajando) / 2       ← gravedad, sin fricción
τ_fricción = |τ_subiendo − τ_bajando| / 2      ← y de regalo, el mapa de fricción
```

El segundo número no es un subproducto menor: la fricción de Coulomb por
articulación explica la dispersión de F1 y la banda de pegado que ya se vio en
el codo (§6 de resultados). Sale gratis del mismo ensayo.

### 1.4 F5 pide un chirp logarítmico; el implementado es lineal

`trajectories.chirp()` barre la frecuencia **linealmente**. F5 especifica
logarítmico con amplitud escalada `A(f) = v_max/(2πf)`. Son dos cambios de
código, pequeños, pero hay que hacerlos antes de F5 o los resultados no son los
que el protocolo describe.

### 1.5 Lo que ya está resuelto y el protocolo puede dar por cerrado

F7.3 dice «comprobar si `arm_sdk` responde con el controlador de locomoción
activo». La hipótesis del `README` era que `arm_sdk` no se aplica con el robot
en reposo. Sigue sin comprobarse con el robot activo, así que el paso se
mantiene — pero conviene recordar que el ensayo de cinco variantes del mensaje
ya descartó que fuera un problema de formato: el publicador empareja y el
servicio no aplica. Es estado del robot, no protocolo.

---

## 2. Colisiones: lo que el protocolo no cubre y hay que arreglar primero

Esta sección responde a la preocupación explícita de que los brazos no se
golpeen con el robot durante los movimientos.

### 2.1 La protección actual no escala a lo que el protocolo pide

Hoy la única protección es `soft_limits_deg` en `gains.yaml`: **dos números**,
`L_shoulder_roll ≥ +10°` y `R_shoulder_roll ≤ −10°`. Eso ha bastado porque
**todos los ensayos mueven una articulación cada vez** y las otras trece están
clavadas en una postura conocida.

F3 (posturas P2 y P3) y F6 (las siete a la vez) rompen esa premisa. Y un tope
por articulación **no puede** expresar una autocolisión, que por definición
depende de la configuración completa.

### 2.2 Cuánto varía el límite real — medido

Barrido del modelo de colisión, brazo izquierdo, ángulo mínimo de
`shoulder_roll` sin choque:

| codo | pitch −70° | −40° | −20° | 0° | +20° | +40° |
|---:|---:|---:|---:|---:|---:|---:|
| 0° | −22° | −21° | −11° | −10° | −10° | −12° |
| 20° | −22° | −21° | −11° | −10° | −6° | −11° |
| 40° | −22° | −21° | −11° | −10° | +1° | −20° |
| 60° | −22° | −21° | −11° | −4° | +1° | −20° |
| **85°** | −22° | −21° | −11° | **+5°** | −10° | −20° |
| 110° | −22° | −21° | −6° | +1° | −10° | −20° |
| 140° | −22° | −21° | −2° | −10° | −10° | −20° |
| 170° | −22° | −12° | −11° | −10° | −10° | −20° |

El límite real va de **+5° a −22°** según la postura: 27° de variación. Un
número fijo no lo describe.

### 2.3 La buena noticia: tu ±10° está bien elegido

- El peor caso de toda la rejilla necesita `roll ≥ +5°`, y el tope está en +10°:
  **5° de margen en el peor caso**, y hasta 32° en el mejor.
- El primer choque en la postura de ensayo (codo 85°, pitch 0°) es a **+4°**,
  entre `wrist_pitch_link` y `hip_pitch_link`. **No es el brazo contra el
  torso: es la muñeca contra la cadera.**
- **P1, P2 y P3 del protocolo están las tres libres de colisión.**

El tope empírico de ±10° queda validado por el modelo. No hay que cambiarlo.

### 2.4 Lo que falta comprobar del mapa — **DESCARTADO el 2026-09-10**

> El operador respondió esta pregunta directamente, y su respuesta es mejor
> evidencia que la que daría el barrido:
>
> - con el codo flexionado, el hombro puede llegar **hasta 0°**;
> - con el brazo estirado, **±10°**;
> - **±15° si se mueven las muñecas**, para que el pulgar de la mano abierta no
>   toque la pierna.
>
> Esa tercera regla **es** la dimensión que faltaba. El mapa se hizo con las
> muñecas a cero y por eso no la cubría; medida sobre la pieza real vale más
> que sobre la geometría de colisión del URDF, que puede estar simplificada.
> Y los números coinciden: el peor caso del modelo pedía `roll ≥ +5°` y la
> envolvente da ±10°, o sea 5° de margen.
>
> `config/gains.yaml` ya recoge las tres reglas. Lo que sigue abajo queda como
> registro de por qué se planteó.

<details><summary>Cómo se planteó</summary>


El barrido movió `shoulder_pitch`, `shoulder_roll` y `elbow` con `shoulder_yaw`
y muñecas en cero. Como el choque que aparece es **de la muñeca**, el ángulo de
muñeca importa y no está explorado. Hay que ampliar el mapa a `shoulder_yaw` y
`wrist_pitch` antes de F3.

Segunda cautela: la geometría de colisión del URDF puede estar simplificada
respecto a la pieza real. El modelo dice +4° y tú mediste que a ±10° va bien;
esa coincidencia es tranquilizadora, pero antes de fiarse del modelo en una
postura nueva conviene aproximarse despacio y mirar.

</details>

### 2.5 Arquitectura propuesta — tres capas

Con 0.1 ms por chequeo, la comprobación de colisión **cabe en el lazo**. Aun
así, la defensa correcta es en capas, porque cada una falla de forma distinta:

| Capa | Cuándo | Qué hace | Coste |
|---|---|---|---|
| **1. Validación previa** | antes de ejecutar | comprueba la trayectoria completa punto a punto contra el modelo; si algún punto choca, **no se arranca** | ~0.1 ms × nº de puntos |
| **2. Portero en el lazo** | cada ciclo | comprueba `q_des` antes de publicar; si choca, congela la consigna en el último punto libre y aborta | 2 % del ciclo |
| **3. Topes por articulación** | siempre | los `soft_limits_deg` de hoy, como red final | gratis |

La capa 3 se queda: es la que sigue protegiendo si el modelo está mal, si
`pinocchio` no carga, o si alguien ejecuta con `--no-collision`. Las capas 1 y 2
son nuevas.

Detalle importante de la capa 2: comprueba **`q_des`, no `q`**. Cuando la
articulación real llega a una postura en colisión ya es tarde; lo que hay que
vetar es la orden.

Y un matiz de la capa 1 que el protocolo necesita para F6: las trayectorias
T1–T3 mueven siete articulaciones a la vez, así que hay que validar la
**trayectoria interpolada**, no solo los waypoints P1/P2/P3. Dos posturas libres
pueden tener un camino que choca entre ellas.

---

## 3. Plan revisado

### Reordenación

```
F-C ──┬── F0 ──> F1 ──> F2 ──> F3 ──> F5 ──┐
      │                  │                  ├──> F7 ──> F8
      └──────────────────┴──> F4 ──> F6 ────┘
```

**F-C es nueva y va primero.** No necesita el robot, no depende de nada, y es lo
que hace ejecutables F3 y F6 sin riesgo de golpe.

### F-C — Modelo de colisión (sin robot, ~1 día)

1. `14_collision_model.py`: carga el URDF local, filtra los 15 pares adyacentes
   por la postura neutra, deja los 251 relevantes por brazo.
2. Ampliar el mapa de §2.2 a `shoulder_yaw` y `wrist_pitch`. Producto: tabla de
   envolvente segura, y confirmación (o no) de que ±10° sigue bastando.
3. Integrar como capas 1 y 2 en `client.py`, con `--no-collision` para
   desactivarlo explícitamente.
4. Validar contra lo que ya se sabe: el modelo debe decir que las 126 corridas
   del 09-05 fueron libres de colisión, y debe marcar como colisión una postura
   de `shoulder_roll` a 0° con el codo a 85°.

**Criterio de aceptación**: cero falsos negativos en las 126 corridas ya
ejecutadas, y `q_des` en colisión rechazada en menos de 0.5 ms.

### F0 — Temporización — **HECHA el 2026-09-08**

Resultado en `06_RESULTADOS.md`, cabecera. Resumen: **el criterio literal no se
cumple** (p99/T = 1.45 contra 1.20) **y aun así el sistema es apto**, porque el
criterio medía lo que no era.

La trayectoria se evalúa contra reloj, así que un ciclo tarde publica la
consigna correcta; y el lazo lee la última posición conocida, así que perder
mensajes intermedios es inocuo. Lo que sesga es la **edad del estado al
usarlo**: máxima 3.17 ms, que a 0.5 rad/s son 1.59 mrad contra errores medidos
de 5 a 32 mrad.

Lección aplicable al resto del protocolo: **conviene comprobar que cada criterio
de aceptación mide la magnitud por la que uno se preocupa**, y no una
correlacionada. Aquí la correlacionada daba un fallo que no lo era.

Nota metodológica: yo había propuesto saltarse F0 con las medidas parciales.
Era una mala recomendación — la fase produjo una corrección real del criterio.
Haberla hecho costó 20 minutos.

### F1 — Estadística (robot, ~4 h)

Tal como está, con una simplificación: **F1.1 y F1.3 se pueden fusionar**. Si
todas las corridas de F1.3 se hacen con y sin precalentamiento en orden
aleatorizado, se obtiene `δ_min` y el efecto frío/caliente del mismo bloque de
datos, en vez de dos campañas.

### F2 — Gravedad (robot, ~1 día) — **con los dos cambios de §1.2 y §1.3**

1. URDF **local**, no repositorio externo.
2. Validación **bidireccional** en cada postura, separando `τ_g` de `τ_fricción`.
3. Identificación de masa por mínimos cuadrados **sobre `τ_g`, no sobre
   `tau_est`**.
4. Si tras separar la fricción el residuo sigue teniendo estructura en `q`,
   entonces sí toca la rama de «error en los eslabones» del protocolo.

Predicción, para dejarla escrita antes de medir: **una parte apreciable de los
3.74 Nm de discrepancia es fricción, no masa.** Si al separar los dos términos
el error del modelo baja de 1.0 Nm sin tocar la masa más allá de los ~800 g
reales, la hipótesis queda confirmada.

### F3 — Postura (robot, ~1 día)

Sin cambios, **salvo que ahora P2 y P3 se validan con F-C antes de mandarlas**,
y las transiciones entre posturas también. El protocolo ya pide «verificar antes
que P3 respeta los topes blandos»; con F-C eso deja de ser una inspección visual
y pasa a ser una comprobación.

### F4, F5, F6 — sin cambios de fondo

F5 necesita el chirp logarítmico de §1.4.

**F6 es la única que sigue necesitando algo de F-C**, y no es el mapa: es la
**capa 1**, validar la trayectoria interpolada antes de ejecutarla. Un tope por
articulación no puede expresar una autocolisión, que depende de la
configuración completa. Para F3, F4, F5 y F8 da igual —mueven una articulación
cada vez y las otras están clavadas, así que la envolvente las describe bien—
pero F6 mueve las siete a la vez, y dos posturas individualmente seguras pueden
tener entre ellas una trayectoria que no lo sea. Son ~30 líneas contra el
modelo que ya existe, 0.1 ms por punto, y solo hacen falta el día de F6.

### F7, F8 — sin cambios

F7.1 (`dq_des`) sigue siendo la más urgente de las dos, porque es la condición
de validez del conjunto `tuned` que ya está en el repositorio.

---

## 4. Por dónde empezar

Si hubiera que elegir un orden por retorno sobre esfuerzo:

1. **F-C** — sin robot, desbloquea F3 y F6, y responde a la pregunta de las
   colisiones. Media jornada larga.
2. **F0** — sin robot, dos horas, y sin él ningún kd es reproducible.
3. **F2 con la corrección de fricción** — es la causa raíz de que kp esté pegado
   al techo, y probablemente disuelve H4, H9 y parte de H10 de una vez.
4. **F1** — cara en tiempo de robot, pero sin ella no se puede afirmar que
   `tuned` sea mejor que `xr_teleoperate`.
5. **F7.1** — el parche de `dq_des`; sin él el conjunto `tuned` no se puede
   desplegar tal cual.

Las tres primeras no necesitan el robot colgado, así que pueden hacerse mientras
esté ocupado en otra cosa.

---

## 5. Riesgos que no cubre ninguna de las dos versiones

- **La geometría de colisión del URDF no está validada contra la pieza real.**
  Coincide con el único punto empírico que hay (±10° / +4° del modelo), y eso es
  un punto, no una validación.
- **Las manos tienen 12 GDL cada una** y el mapa de colisión se hizo con los
  dedos en su postura neutra. Una mano cerrada sobre un objeto (F4, F8) ocupa un
  volumen distinto.
- **El brazo contrario se mueve.** El mapa de §2.2 mueve un brazo con el otro
  quieto. En F6 se mueven los dos, y la colisión brazo-brazo delante del torso
  no está explorada.
- **Nada de esto protege contra un fallo del propio lazo.** Si el proceso muere,
  el puente de motores los deshabilita y los brazos caen; con el robot colgado
  eso es aceptable, de pie no lo sería. Es una razón más para que F7.3 vaya al
  final.
