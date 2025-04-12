import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('kyron_nav_wf')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info('Waypoint Follower Client initialized')

    def define_waypoints(self):
        """Lista de puntos que ha de recorrer el robot."""
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

        waypoints=[]

        for x,y in puntos:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)

        return waypoints


    def send_waypoints(self, waypoints):
        """Enviar la lista de puntos al action server."""
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server no disponible')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Enviando waypoints al servidor...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        """Manejar la respuesta del servidor."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):

        """Handle feedback from the action server."""
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Currently at waypoint: {current_waypoint}')

    def get_result_callback(self, future):

        """Handle the result from the action server."""
        result = future.result().result
        self.get_logger().info('Result received:')
        self.get_logger().info(f'Missed waypoints: {len(result.missed_waypoints)}')
        self.get_logger().info(f'Error code: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_client = WaypointFollower()
    waypoints = waypoint_client.define_waypoints()
    waypoint_client.send_waypoints(waypoints)
    rclpy.spin(waypoint_client)

if __name__ == '__main__':
    main()