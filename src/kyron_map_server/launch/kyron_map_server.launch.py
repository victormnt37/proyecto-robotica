# provide_map.launch.py
import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name='kyron_map_server'
    
    rviz_config_dir = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'kyron_map_server.rviz')
    map_file = os.path.join(get_package_share_directory(pkg_name), 'map', 'hospital_world.yaml')

    return LaunchDescription([
        Node(
            #Contiene un servidor que se encargará de proporcionarnos el mapa. Este servidor queda abierto, 
            # a la espera de recibir una petición de carga del mapa. Para ello utilzará la llamada al servicio map_server/load_map
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file} 
                       ]),

        Node(
            #lanza el visualizador RVIZ con la configuración que hemos guardado en el directorio rviz
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
            ),

        Node(
            #se trata de un nodo que se encarga de controlar el ciclo de vida de los nodos ROS2. 
            # Lanza una máquina de estados que decidirá en qué estado se encuentra cada nodo (unconfigured, inactive, active o finalized).
            # Para saber más acerca de este tema podéis acceder a la siguiente web. 
            # Este manejador del ciclo de vida de los nodos permite que el sistema tenga un comportamiento determinístico y facilita la depuración del mismo.
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}])            
        ])