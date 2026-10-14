# robot_config — configuración del sistema del H1-2

Cambios al sistema del PC2 del robot (drivers, red) que necesitan otros
proyectos de este repositorio, con sus pasos para repetirlos en otro H1-2.

| Carpeta | Qué configura | Lo necesita |
|---|---|---|
| [`wifi_tplink_tx20u_plus/`](wifi_tplink_tx20u_plus/) | Driver de la WiFi USB TP-Link Archer TX20U Plus y conexión a la red del laboratorio | la teleoperación XR ([`h1_2_teleoperation`](../h1_2_teleoperation/README.md)): las gafas llegan al robot por esta WiFi |
