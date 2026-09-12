from setuptools import find_packages, setup

package_name = 'h1_2_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # La configuración va al `share/`, que es donde `gains.py` la busca.
        # Los `gravity_params_*.json` son los parámetros de masa identificados
        # sobre el robot: no se pueden regenerar sin él, así que viajan con el
        # paquete igual que las ganancias.
        ('share/' + package_name + '/config', [
            'config/gains.yaml',
            'config/gravity_params_20260908_145812.json',
            'config/gravity_params_20260908_150409.json',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utec',
    maintainer_email='molortegui@utec.edu.pe',
    description='Control articular de los brazos del Unitree H1-2 por unitree_hg/LowCmd.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # Diagnóstico y canal
            'read_state = h1_2_arm_control.read_state:main',
            'debug_mode = h1_2_arm_control.debug_mode:main',
            # Posturas
            'init_pose = h1_2_arm_control.init_pose:main',
            'rest_pose = h1_2_arm_control.rest_pose:main',
            'goto = h1_2_arm_control.goto:main',
            # Movimiento
            'hold = h1_2_arm_control.hold:main',
            'move_joint = h1_2_arm_control.move_joint:main',
        ],
    },
)
