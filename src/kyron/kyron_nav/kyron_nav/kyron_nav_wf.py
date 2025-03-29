#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('kyron_nav_wf')
        
        # Crea el cliente de la acción FollowWaypoints
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.get_logger().info("Cliente de waypoints inicializado")

    def create_waypoint(self, x, y, yaw_degrees=0):
        
        """Crea un punto de ruta con posición y orientación"""
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'  # Usamos el sistema de coordenadas del mapa
        
        # Posición (x, y en metros)
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = 0.0  # Para movimiento en 2D
        
        # Orientación (convertimos ángulos grados a cuaternión)
        yaw_rad = math.radians(yaw_degrees)
        qz = math.sin(yaw_rad / 2)
        qw = math.cos(yaw_rad / 2)
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw
        
        return waypoint

    def send_waypoints(self):
        
        """Envía la lista de puntos al servidor"""
        
        # 1. Definir los waypoints (puntos de ruta)
        waypoints = [
            # Waypoint 1 (Orientación 0 grados)
            self.create_waypoint(
                x=0.5846771001815796,
                y=15.311471939086914,
                yaw_degrees=0
            ),
            # Waypoint 2 (Orientación 45 grados)
            self.create_waypoint(
                x=-5.106757640838623,
                y=-1.8345870971679688,
                yaw_degrees=45
            ),
            # Waypoint 3 (Orientación 90 grados)
            self.create_waypoint(
                x=4.829135417938232,
                y=1.2413616180419922,
                yaw_degrees=90
            ),
            # Waypoint 4 (Orientación 135 grados)
            self.create_waypoint(
                x=4.830772399902344,
                y=-5.883827209472656,
                yaw_degrees=135
            ),
            # Waypoint 5 (Orientación 180 grados)
            self.create_waypoint(
                x=-5.829804420471191,
                y=-14.206602096557617,
                yaw_degrees=180
            ),
            # Waypoint 6 (Orientación 225 grados)
            self.create_waypoint(
                x=3.6522417068481445,
                y=-21.372636795043945,
                yaw_degrees=225
            ),
            # Waypoint 7 (Orientación 270 grados)
            self.create_waypoint(
                x=-5.516279220581055,
                y=-24.34018898010254,
                yaw_degrees=270
            )
        ]
        
        # 2. Esperar conexión con el servidor
        self.get_logger().info("Esperando servidor...")
        self.action_client.wait_for_server()
        
        # 3. Crear y enviar el goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        
        self.get_logger().info("Enviando waypoints...")
        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info("Waypoints enviados correctamente!")

def main(args=None):
    rclpy.init(args=args)
    
    # Crear y ejecutar el cliente
    follower = WaypointFollower()
    follower.send_waypoints()
    
    # Mantener el nodo activo
    rclpy.spin(follower)
    
    # Limpieza
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()