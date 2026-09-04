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

### Pendiente

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
