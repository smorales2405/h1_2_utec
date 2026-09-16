from setuptools import find_packages, setup

package_name = 'h1_2_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # La configuración de RViz de la prueba del objeto azul.
        ('share/' + package_name + '/rviz', ['rviz/objeto_azul.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utec',
    maintainer_email='molortegui@utec.edu.pe',
    description='Cámara de cabeza (D435i) del H1-2 en ROS 2, vía el servicio videohub.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # Prueba rápida: lo primero que hay que correr.
            'camera_test = h1_2_vision.camera_test:main',
            # El nodo de la cámara.
            'head_camera = h1_2_vision.head_camera:main',
            # Un videohub de mentira, para trabajar sin el robot delante.
            'fake_videohub = h1_2_vision.fake_videohub:main',
            # Pruebas de visión sobre la imagen de la cámara.
            'detector_azul = h1_2_vision.pruebas.detector_azul:main',
        ],
    },
)
