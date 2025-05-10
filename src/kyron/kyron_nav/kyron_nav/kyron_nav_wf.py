import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('kyron_nav_wf')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        # estados del robot
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.command_sub = self.create_subscription(String, '/robot_command', self.command_callback, 10)
        self.current_command = 'PATROLLING'
        self.goal_handle = None
        
        self.get_logger().info('Waypoint client iniciado')

    def define_waypoints(self):
        puntos = [
            (2.1817116737365723, 13.7138671875),
            (3.029665470123291, 11.146732330322266),
            (3.756692886352539, 8.287837982177734),
            (4.547539710998535, 4.923230171203613),
            (4.739471435546875, 2.716395378112793),
            (2.4916281700134277, 1.764068603515625),
            (-3.4919967651367188, 1.7400178909301758),
            (-4.780571460723877, 4.424581527709961)
        ]

        waypoints = []
        for x, y in puntos:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)

        return waypoints

    def publish_state(self, state_str):
        msg = String()
        msg.data = state_str
        self.state_pub.publish(msg)
        self.get_logger().info(f'[ESTADO] {state_str}')

    def send_waypoints(self, waypoints):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server no disponible')
            return

        self.publish_state('PATROLLING')

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Enviando waypoints al servidor...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rechazado')
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'waypoint_actual: {current_waypoint}')
        # Verifica si el usuario mandó detener
        if self.current_command != 'PATROLLING':
            self.get_logger().warn('Interrupción recibida. Cancelando patrullaje...')
            self.goal_handle.cancel_goal_async()
            self.publish_state('INTERRUPTED')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Resultados recibidos:')
        self.get_logger().info(f'Waypoints fallidos: {len(result.missed_waypoints)}')
        self.get_logger().info(f'Error code: {result.error_code}')
        self.publish_state('IDLE')
        rclpy.shutdown()

    def command_callback(self, msg):
        self.get_logger().info(f'[COMANDO RECIBIDO] {msg.data}')
        if msg.data in ['STOP', 'GOING_TO_ROOM']:
            self.current_command = msg.data

def main(args=None):
    rclpy.init(args=args)
    waypoint_client = WaypointFollower()
    waypoints = waypoint_client.define_waypoints()

    # Espera breve para comprobar si hay una interrupción reciente
    time.sleep(0.5)
    if waypoint_client.current_command != 'PATROLLING':
        waypoint_client.get_logger().warn('Otro comportamiento activo. Cancelando patrullaje.')
        waypoint_client.publish_state('INTERRUPTED')
        rclpy.shutdown()
        return

    waypoint_client.send_waypoints(waypoints)
    rclpy.spin(waypoint_client)

if __name__ == '__main__':
    main()
