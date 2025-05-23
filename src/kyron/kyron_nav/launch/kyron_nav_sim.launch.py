#Launch que lanza todo lo necesario para simular.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Ruta a los paquetes
    pkg_kyron_nav = get_package_share_directory('kyron_nav')
    pkg_kyron_mundo = get_package_share_directory('kyron_mundo')

    # Configuraciones
    rviz_config_dir = os.path.join(pkg_kyron_nav, 'config', 'kyron_rviz_config.rviz')
    nav2_yaml = os.path.join(pkg_kyron_nav, 'config', 'kyron_nav_params.yaml')
    map_file = os.path.join(pkg_kyron_nav, 'config', 'hospital_world.yaml')

    return LaunchDescription([
        
        #Ejecuta los comandos que necesitamos sin necesitar que llamarlos desde otra terminal
         
        # Lanzar Gazebo con el mundo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_kyron_mundo, 'launch', 'turtlebot3_hospital.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        
        # Lanzar Nav2 completo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 
                           'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_yaml  # Usa tu archivo de parámetros
            }.items()
        ),

        # Nodo del mapa
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
                'topic_name': 'map',
                'frame_id': 'map',
                'autostart': True
            }]
        ),
        

        # Nodo de localización AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),

        # Servidor de planificación global/local
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),


        # Lifecycle manager que controla todos los nodos de Nav2
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),

        # Iniciar RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen',
            prefix=['bash -c \'sleep 5; $0 $@\'']
        )
    ])