from setuptools import setup
import os
from glob import glob

package_name = 'ubicacion_robot_en_vivo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@example.com',
    description='Paquete que muestra la posición en vivo del robot usando AMCL',
    license='MIT',
    tests_require=['pytest'],
  entry_points={
    'console_scripts': [
        'live_location = ubicacion_robot_en_vivo.ubicacion_en_vivo:main',
    ],
},
)