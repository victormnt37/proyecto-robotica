import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import math
from rclpy.task import Future


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('kyron_nav_wf')
        
        # Crear cliente de acción
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.get_logger().info("Cliente de waypoints inicializado")

    def create_waypoint(self, x, y, yaw_degrees=0):
        """Crea un punto de ruta con posición y orientación"""
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.header.stamp = self.get_clock().now().to_msg()  # Timestamp necesario
        
        # Posición
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = 0.0
        
        # Orientación (convertimos ángulos grados a cuaternión)
        yaw_rad = math.radians(yaw_degrees)
        qz = math.sin(yaw_rad / 2)
        qw = math.cos(yaw_rad / 2)
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw
        
        return waypoint

    def send_waypoints(self):
        """Define y envía los waypoints al servidor de acción"""
        
        # Lista de waypoints
        waypoints = [
            self.create_waypoint(0.5, 1.0, 0),
            self.create_waypoint(1.5, 2.0, 45),
            self.create_waypoint(2.5, 3.0, 90),
            self.create_waypoint(3.5, 4.0, 135),
            self.create_waypoint(4.5, 5.0, 180)
        ]
        
        # Esperar conexión con el servidor
        self.get_logger().info("Esperando conexión con el servidor...")
        self.action_client.wait_for_server()
        
        # Crear mensaje de acción
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        
        # Enviar acción
        self.get_logger().info("Enviando waypoints al robot...")
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Maneja la respuesta del servidor cuando se envía un goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rechazado por el servidor")
            return
        
        self.get_logger().info("Goal aceptado, esperando resultado...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Recibe feedback durante la ejecución de la acción"""
        self.get_logger().info(f"Waypoint alcanzado: {feedback_msg.feedback.current_waypoint}")

    def result_callback(self, future: Future):
        """Procesa el resultado final de la acción"""
        result = future.result().result
        self.get_logger().info("Waypoints completados exitosamente!")
        rclpy.shutdown()  # Finaliza el nodo al completar la acción


def main(args=None):
    rclpy.init(args=args)
    follower = WaypointFollower()
    follower.send_waypoints()
    rclpy.spin(follower)


if __name__ == '__main__':
    main()
