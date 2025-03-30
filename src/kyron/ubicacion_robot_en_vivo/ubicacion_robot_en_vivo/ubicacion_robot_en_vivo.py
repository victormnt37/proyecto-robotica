# kyron_localization/kyron_localization/live_location.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class LiveLocation(Node):
    def __init__(self):
        super().__init__('live_location_node')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(
            f'Posición actual del robot:\n'
            f'X: {position.x:.2f}, Y: {position.y:.2f}\n'
            f'Ori (W): {orientation.w:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = LiveLocation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()