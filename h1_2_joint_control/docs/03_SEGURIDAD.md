# Seguridad

El H1-2 pesa unos 70 kg y sus brazos alcanzan lejos. Este paquete manda
directamente a los motores. Lo que sigue no es burocracia.

## Antes de cada sesión

1. **El robot colgado del arnés o firmemente sujeto.** Aunque el canal
   `arm_sdk` deja las piernas al controlador del robot, mover los brazos
   desplaza el centro de masas.
2. **Nadie dentro del alcance de los brazos**, ni delante ni detrás.
3. **Mando en la mano de alguien.** Paro de emergencia: **`L2 + B`**. Funciona
   también en modo debug: el robot pasa a amortiguación y se deja caer despacio
   — de ahí el arnés.
4. **Ejecutar antes `python3 scripts/00_diagnose.py`.** Es de solo lectura y
   dice si hay algo raro (motores deshabilitados, calientes, o alguien
   publicando donde no debe).

## Escalera de tres peldaños

Nunca saltar directamente al tercero.

```bash
# 1) sin tocar el robot: se publica en un tópico que nadie escucha
python3 scripts/02_move.py --joint L_elbow --traj step --amp 0.15 --dry-run

# 2) en el canal bueno, pero con autoridad nula (w = 0)
python3 scripts/01_hold.py --seconds 10 --weight 0

# 3) autoridad real, sin comandar movimiento
python3 scripts/01_hold.py --seconds 10
```

Solo si el peldaño 3 sale limpio —deriva de milirradianes, sin temblor— tiene
sentido comandar un movimiento.

## Lo que hace el cliente por su cuenta

`H12Client` vigila en **cada ciclo** y suelta el control si algo se sale:

| Vigilancia | Umbral por defecto | Dónde se cambia |
|---|---|---|
| Par sostenido | > 70 % del `tau_max` del URDF **durante más de 0.3 s** | `safety.tau_abort_fraction` |
| Temperatura de bobinado | > 80 °C | `safety.temperature_abort` |
| Topes articulares | recorte a los del URDF con 0.05 rad de guarda | `safety.joint_limit_margin` |
| Estado fresco | aborta si `/lowstate` lleva > 0.5 s sin llegar | `safety.lowstate_timeout` |
| Velocidad de la consigna | 1.0 rad/s en las rampas | `safety.max_ref_velocity` |

El umbral de par exige que se **mantenga** 0.3 s: un escalón produce un pico
legítimo, y abortar en el pico haría inservible la prueba de escalón. Lo que se
persigue es un atasco o un choque, no un transitorio.

Además, siempre:

* **`engage()` arranca con la consigna en la postura actual.** Nunca se
  comanda un salto al tomar el control, y se mide la deriva para comprobarlo.
* **El peso sube y baja en rampa** (1.5 s al entrar, 2 s al salir).
* **Al salir se vuelve primero a la postura inicial** y solo después baja el
  peso, para que el relevo con el controlador del robot no dé un tirón.
* **Ctrl-C, una excepción o un aborto ejecutan la misma salida ordenada**: está
  en el `finally` de un gestor de contexto.

## El canal `lowcmd`: por qué está desaconsejado

`--channel lowcmd` existe, pero:

* Exige soltar el controlador de alto nivel (`ReleaseMode`). **El robot deja
  de equilibrarse en ese instante.**
* Obliga a comandar los 27 motores. Las piernas pasan a depender de nuestro
  proceso: si el script muere, nadie las sostiene.
* Si el `ReleaseMode` falla en silencio, se acaba peleando por el tópico con el
  servicio — que es el problema original (ver
  [`01_DIAGNOSTICO.md`](01_DIAGNOSTICO.md)).

Para sintonizar los brazos **no hace falta**. Se documenta por completitud y
para las articulaciones que `arm_sdk` no cede (piernas), que quedan fuera del
alcance actual de este paquete.

## Si algo va mal

| Síntoma | Qué hacer |
|---|---|
| El brazo se va donde no debe | `L2 + B` en el mando, luego Ctrl-C en la terminal |
| El script se ha colgado | Ctrl-C. Si no responde, `L2 + B` y matar el proceso |
| El proceso ha muerto de golpe | El peso de `arm_sdk` deja de refrescarse y el robot recupera el brazo. Aun así, comprobar la postura antes de seguir |
| Un motor pasa de 80 °C | Parar y dejar enfriar. En reposo van a 46–51 °C |
| Tras un fallo, no llega `/lowstate` | Un participante DDS quedó a medias. Esperar ~10 s y volver a probar (ver `01_DIAGNOSTICO.md`, tercer hallazgo) |
