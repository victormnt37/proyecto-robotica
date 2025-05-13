import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Ruta base al paquete
    pkg_kyron_vision = get_package_share_directory('kyron_vision')

    return LaunchDescription([
        Node(
            package='kyron_vision',
            executable='id_cara',
            name='id_cara',
            output='screen',
            parameters=[{'use_sim_time': True , 'modo':'sim'}]
        ),

        Node(
            package='kyron_vision',
            executable='id_cuerpo',
            name='id_cuerpo',
            output='screen',
            parameters=[{'use_sim_time': True,'modo':'sim'}]
        )
    ])
