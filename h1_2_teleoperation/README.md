# Teleoperación XR del H1-2 real con Meta Quest 3

Los brazos del H1-2 y los dedos de sus manos **Inspire RH56DFTP** siguen a los
del operador, que lleva unas **Meta Quest 3**. Todo corre en el **PC2 del robot**
(`~/robotics40/meta`: `xr_teleoperate` de Unitree, commit `817fb00`, entorno
`env_tv`). Desde este PC solo se lanza, por SSH, con los scripts de
[`scripts/`](scripts/).

La teleoperación **en simulación** (Isaac Sim) es otra cosa y está en
[`README_SIM.md`](README_SIM.md), con sus scripts en `scripts_sim/`.

**Nada de esto se lanza sin el visto bueno del responsable del laboratorio.
L2 + B en el mando es la parada de emergencia.**

---

## 1. Cómo funciona

```
  ESTE PC (Ubuntu)                         H1-2
  scripts/teleop_xr.sh ── ssh -t ──▶ PC2  unitree-h1-2-pc4  (WiFi 192.168.0.143)
                                      ~/robotics40/teleop_xr.sh → meta/arrancar.sh
                                        1. servidor de imagen: teleimager + D435i (sudo)
                                        2. driver de las manos: Modbus TCP ⇄ DDS
                                        3. teleop_hand_and_arm.py (meta/lanzar_teleop.py)
                                             · Vuer/WebXR :8012  ◀── Meta Quest 3 (WiFi UTEC_H1_2)
                                             · rt/lowcmd (DDS eth0)  ──▶ PC1: brazos, en Debug
                                             · rt/inspire_hand/ctrl/{l,r} ──▶ driver ──▶ manos
                                                                   izq 192.168.124.211 · der .210
```

- `comun.sh` ve que este PC no es el robot y **reenvía la orden por SSH**:
  `./teleop_xr.sh` se convierte en
  `ssh -t unitree@192.168.0.143 "cd ~/robotics40 && ./teleop_xr.sh"`.
- **Siempre se ejecuta el código del robot.** Cambiar estos scripts aquí no
  cambia el teleop; el teleop se cambia en `~/robotics40/meta` del robot.
- **Las gafas se conectan directamente al robot**, no a este PC. Por eso aquí no
  hace falta conda, SDK, CycloneDDS, ROS ni certificados.

## 2. Los scripts

| Archivo | Qué hace | ¿Mueve el robot? |
|---|---|---|
| `teleop_xr.sh` | arranca el teleop; `./teleop_xr.sh parar` detiene la cámara y el driver de las manos | **sí** |
| `comun.sh` | el reenvío por SSH; lo cargan los demás y sin él no funciona ninguno | no |
| `estado.sh [s]` | `rt/lowstate`, FSM y `CheckMode` (¿Debug de verdad?) durante `s` segundos (5 por defecto) | no |
| `robot.sh [comprobar\|gafas]` | `comprobar`: red, clave SSH y estado. `gafas`: URL para las Quest. Sin argumentos: terminal en el robot | no |

Son copias exactas de los de `~/robotics40` (el robot y la ThinkStation de UTEC),
comparadas por sha256 el 2026-10-06.

- Los cuatro tienen que estar **en la misma carpeta**. Lánzalos por su ruta, sin
  enlaces simbólicos.
- **No uses `./robot.sh traer` aquí**: copiaría todo `~/robotics40` del robot
  dentro de esta carpeta del repositorio.

## 3. Requisitos

**En este PC** (una sola vez):

1. Conectarse a la **WiFi `UTEC_H1_2`**: el robot es la `192.168.0.143`.
2. Entrar al robot **por clave SSH**, sin contraseña. Si `./robot.sh comprobar`
   dice «sin acceso por clave»: `ssh-copy-id unitree@192.168.0.143`. Ya está
   hecho en el `utec-Precision-3581`.
3. Que **no se suspenda** durante la sesión: tapa abierta y cargador conectado.
   Una suspensión cuenta como corte del SSH (§6).
4. Que responda al **ping**, como Ubuntu hace por defecto: el teleop lo usa para
   detectar cortes (§6).

**En el robot** ya está todo instalado; no hay que tocar nada:
`~/robotics40/meta` (con `env_tv` y el certificado para la `.0.143`),
`~/teleop_venv`, `~/inspire_ftp_dual_driver.py` y la D435i en el USB del PC2.

## 4. Antes de cada sesión

- Robot **colgado del arnés**, sin tocar el suelo. Mando en la mano con
  **L2 + B** listo, y nadie al alcance de los brazos.
- **Batería**: no se puede leer por software (`rt/lf/bmsstate` llega a cero).
  Mírala en el robot o en el mando.
- Comprobación de solo lectura:

  ```bash
  cd ~/Documents/h1_2_utec/h1_2_teleoperation/scripts
  ./robot.sh comprobar
  ```

- Que **nadie más tenga el teleop lanzado**, por ejemplo desde la ThinkStation:
  no puede haber dos teleops mandando los brazos. Esto debe salir vacío:

  ```bash
  ssh unitree@192.168.0.143 'pgrep -af "lanzar_teleop|inspire_ftp_dual|teleimager.image_server"'
  ```

## 5. Ejecutar

```bash
cd ~/Documents/h1_2_utec/h1_2_teleoperation/scripts
```

1. **Debug con L2 + R2** en el mando. Las piernas se sueltan: por eso va colgado.
2. **Confirmar el Debug**: `./estado.sh` tiene que dar `CheckMode … 'name': ''`.
   Con `'ai'` el teleop intenta `ReleaseMode()` 3 veces. Si sigue en `'ai'`,
   **los brazos no obedecen**, aunque todo lo demás parezca ir bien.
3. **Lanzar** en una terminal interactiva (hacen falta las teclas `r` y `q`):

   ```bash
   ./teleop_xr.sh
   ```

   Arranca la cámara (puede pedir el `sudo` **del robot**) y el driver de las
   manos, que **las abre nada más arrancar**. Al inicializarse, **los brazos van
   solos a 0°** (colgando rectos), deprisa y antes de pulsar `r`. Al final
   imprime la URL para las gafas.
4. **En las Quest 3**: WiFi `UTEC_H1_2`, y en el navegador la URL completa, con
   su `?ws=` (sin él no conecta el websocket y no se ve nada):

   ```
   https://192.168.0.143:8012/?ws=wss://192.168.0.143:8012
   ```

   Aviso del certificado: *Avanzado → Continuar*. Después, «Virtual Reality»
   **sin los mandos en la mano**, y permitir todo. `./robot.sh gafas` imprime
   esta URL.
5. Pon los brazos como los tiene el robot y pulsa **`r`**: brazos y dedos
   empiezan a seguirte.
6. **`q`** para salir:
   1. Primero se **abren del todo las dos manos**. El teleop espera a leerlas
      abiertas, como mucho 3 s.
   2. Luego los **brazos vuelven a casa** (~0,5 s).
   3. El programa termina. **Brazos y piernas quedan sin fuerza** (Debug y nadie
      mandando). Para volver, el mando.
7. Parar la cámara y el driver de las manos:

   ```bash
   ./teleop_xr.sh parar
   ```

## 6. Si se corta el SSH

**El teleop sale solo, como con `q`**: abre las manos y los brazos vuelven a
casa. Detecta dos tipos de corte:

- **Se cuelga el terminal**: se cierra la terminal o el `ssh` de este PC. Lo
  detecta en unos 0,2 s.
- **Este PC deja de responder al ping durante 3 s**: WiFi caída o portátil
  suspendido. Hace falta porque el `sshd` del robot no tiene
  `ClientAliveInterval`, y con un corte silencioso no se enteraría en minutos.

Qué tener en cuenta:

- Lo ocurrido queda en el log del robot (§7). Al reconectar, `./teleop_xr.sh
  parar`: la cámara y el driver de las manos siguen corriendo.
- Una caída de la WiFi de más de 3 s también termina el teleop.
- Si este PC no contesta pings, solo se vigila el terminal, y el log lo avisa.

**Por cable**, sin depender de la WiFi para el SSH: conecta este PC al puerto de
depuración del robot, con una IP fija en `192.168.123.x/24`, y lanza
`ROBOT_SSH=unitree@192.168.123.164 ./teleop_xr.sh`. Las gafas siguen entrando
por `192.168.0.143`.

## 7. Logs y problemas frecuentes

Los logs están en el robot, en `~/robotics40/meta/logs/`: `teleop.log`,
`manos.log` e `image_server.log`. La terminal muestra `teleop.log` en directo.

| Síntoma | Causa y solución |
|---|---|
| Los brazos no siguen, aunque todo parece ir bien | No está en Debug (`'ai'`): L2 + R2 y comprobar con `./estado.sh` |
| La página de las gafas carga pero no se ve nada | Falta `?ws=` en la URL. El campo *Socket URI* de la página debe decir `wss://192.168.0.143:8012`, **con el puerto** |
| Un dedo deja de moverse (`err` 5) | `ssh -t unitree@192.168.0.143 ~/robotics40/manos/manos.sh borrar-error izq` (o `der`) |
| Los dedos no cierran del todo | Conocido: el retargeting se queda corto, sobre todo en la izquierda |
| «sin acceso por clave» | `ssh-copy-id unitree@192.168.0.143` |
| `No route to host` | Robot apagado, o este PC fuera de `UTEC_H1_2` |

## 8. Cambios hechos en el robot

**2026-10-07**, en `~/robotics40/meta` del robot:

- **`lanzar_teleop.py`**:
  - `q` abre las manos antes de que los brazos vuelvan a casa (también al salir
    con Ctrl-C o por un error).
  - Un corte del SSH equivale a `q`.
  - No toca el código de Unitree: aplica parches en tiempo de ejecución.
- **`arrancar.sh`**: el teleop escribe solo en `logs/teleop.log` y la terminal
  muestra un `tail` de ese log. Antes usaba `tee`, que un SSH cortado bloqueaba
  o rompía.

Los originales están junto a ellos como `*.antes_20261007`. Para volver atrás:

```bash
ssh unitree@192.168.0.143 'cd ~/robotics40/meta && for f in lanzar_teleop.py arrancar.sh README.md; do cp -p $f.antes_20261007 $f; done'
```

**Probado** sin mover el robot:

- pruebas unitarias;
- un corte real del SSH con un teleop falso;
- `--help` con el teleop real.

**Falta probarlo con el robot moviéndose**:

1. Pulsar `q` sin haber pulsado `r`.
2. Pulsar `q` con una mano cerrada.
3. Cerrar la terminal en mitad del teleop.

## 9. Lo que se retiró de este directorio (2026-10-09)

Se borró el despliegue físico antiguo: `xr_teleoperate` en la laptop con DDS por
cable, la cámara por el puente `videohub`, `README_DEPLOY.md`,
`03_launch_teleop.sh`, `arranca_teleop.sh`, `arm_joint_test.py`,
`busca_quest.sh`, `04_test_inspire_dds_loopback.py`, `robot_pc2/` y el parche
`televuer_una_sola_sesion.patch`. Daba muchos fallos al probarlo. Sigue en el
historial de git, por ejemplo:

```bash
git show 1df4e32:h1_2_teleoperation/README_DEPLOY.md
```

`patches/xr_teleoperate_h1_2_dq_ref.patch` y `xr_teleoperate_h1_2_tuning.patch`
(con sus README) se conservan porque los cita `h1_2_joint_control/docs`. El
teleop actual del robot no los aplica.
