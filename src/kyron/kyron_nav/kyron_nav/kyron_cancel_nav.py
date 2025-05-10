import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class CancelNavigationNode(Node):
    def __init__(self):
        super().__init__('kyron_cancel_nav')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # estados del robot
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f'[ESTADO] Publicado estado: {state}')

    def cancel_navigation(self):
        self.get_logger().info('Esperando conexión con action server...')
        self._action_client.wait_for_server()

        # solicita la lista de objetivos activos y cancela el primero si hay alguno
        self.get_logger().info('Solicitando cancelación de la navegación activa...')
        cancel_future = self._action_client._cancel_all_goals()  # cancela todos los objetivos activos

        def cancel_done(fut):
            cancel_response = fut.result()
            if cancel_response.goals_canceling:
                self.get_logger().info('Navegación cancelada con éxito.')
                self.publish_state('STOPPED')
            else:
                self.get_logger().warn('No se encontraron objetivos activos para cancelar.')

            rclpy.shutdown()

        cancel_future.add_done_callback(cancel_done)

def main():
    rclpy.init()
    node = CancelNavigationNode()
    node.cancel_navigation()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
