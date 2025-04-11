import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('kyron_nav_wf')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.localized = False
        # Suscripción a /amcl_pose para verificar localización
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self.timer = self.create_timer(1.0, self.check_server_and_send)

    def amcl_callback(self, msg):
        # Verificar si la covarianza es baja (indica buena localización)
        covariance = msg.pose.covariance
        # Covarianza en x y y (índices 0 y 7 en la matriz 6x6)
        if covariance[0] < 0.1 and covariance[7] < 0.1:
            self.localized = True
            self.get_logger().info("Robot está localizado correctamente.")
        else:
            self.localized = False
            self.get_logger().warn("Esperando localización precisa...")

    def check_server_and_send(self):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Esperando al servidor de acción...")
            return
        if not self.localized:
            self.get_logger().warn("Esperando a que el robot se localice...")
            return
        self.get_logger().info("Servidor listo y robot localizado, enviando waypoints...")
        self.send_waypoints()
        self.timer.cancel()

    def define_waypoints(self):
        waypoints = [
            (2.1817116737365723, 13.7138671875),
            (3.029665470123291, 11.146732330322266),
            (3.756692886352539, 8.287837982177734),
            (4.547539710998535, 4.923230171203613),
            (4.739471435546875, 2.716395378112793),
            (2.4916281700134277, 1.764068603515625),
            (-3.4919967651367188, 1.7400178909301758),
            (-4.780571460723877, 4.424581527709961),
            (-3.342264175415039, 7.7682061195373535),
            (-2.499321937561035, 9.3995943069458),
            (-1.415143370628357, 11.875675201416016),
            (-0.4009385108947754, 14.255132675170898)
        ]
        goal_poses = []
        for x, y in waypoints:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)
        return goal_poses

    def send_waypoints(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.define_waypoints()
        self.send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('¡Objetivo rechazado por el servidor!')
            return
        self.get_logger().info('¡Objetivo aceptado!')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Ejecutando waypoint: {current_waypoint + 1}')

    def result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().error(f'No se alcanzaron los waypoints: {result.missed_waypoints}')
        else:
            self.get_logger().info('Navegación completada con éxito')

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()