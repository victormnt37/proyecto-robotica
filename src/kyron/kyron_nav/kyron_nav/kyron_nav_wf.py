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
            self.create_waypoint(1.0, 0.0, 0),   # Punto 1: 1m adelante, sin rotación
            self.create_waypoint(2.0, 1.0, 45),  # Punto 2: 2m adelante y 1m izquierda, rotado 45°
            self.create_waypoint(1.0, 1.0, 90)   # Punto 3: 1m adelante y 1m izquierda, rotado 90°
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