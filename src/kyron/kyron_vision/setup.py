from setuptools import setup

package_name = 'kyron_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/kyron_vision/launch', ['launch/kyron_vision.launch.py']),
        ('share/kyron_vision/launch', ['launch/kyron_vision_irl.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='ariel507182003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_image=kyron_vision.capture_image:main',
            'id_cuerpo=kyron_vision.id_cuerpo:main',
            'id_color=kyron_vision.id_color:main',
            'id_cara=kyron_vision.id_cara:main',
            'listener=kyron_vision.listener:main',
            'listener_cuerpo=kyron_vision.listener_cuerpo:main'

        ],
    },
)
