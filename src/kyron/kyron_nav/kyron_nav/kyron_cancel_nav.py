import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class CancelNavClient(Node):
    def __init__(self):
        super().__init__('kyron_cancel_nav')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

    def cancel_navigation(self):
        self.get_logger().info("Esperando conexión con action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Solicitando cancelación de la navegación activa...")
        cancel_future = self._action_client.cancel_all_goals_async()
        cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info('Cancelación completada exitosamente.')
        else:
            self.get_logger().warn(f'Código de retorno de cancelación: {cancel_response.return_code}')
        
        # Publicar estado detenido
        msg = String()
        msg.data = 'STOPPED'
        self.state_pub.publish(msg)

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CancelNavClient()
    node.cancel_navigation()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
