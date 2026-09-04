"""Control y sintonización articular del Unitree H1-2 sobre unitree_ros2.

Módulos:
    joints        tabla de los 27 motores: índices, topes, par y velocidad
    crc           CRC propio de Unitree para `unitree_hg/LowCmd`
    config        carga de config/gains.yaml
    client        cliente de bajo nivel (canales arm_sdk y lowcmd) con vigilancia
    trajectories  consignas de prueba: escalón, seno, chirp
    metrics       métricas de seguimiento, de escalón y de temblor
    recorder      volcado de ensayos a CSV
"""
__version__ = "0.1.0"
