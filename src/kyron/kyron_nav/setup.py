from setuptools import setup
import os
from glob import glob

package_name = 'kyron_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm'))

    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='ariel507182003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kyron_initial_pose_pub = kyron_nav.kyron_initial_pose_pub:main',
            'kyron_nav_wf=kyron_nav.kyron_nav_wf:main'
        ],
    },
)
