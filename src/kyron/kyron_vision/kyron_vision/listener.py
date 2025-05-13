from kyron_interface.msg import PersonaIdentificada
import rclpy
from rclpy.node import Node

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(
            PersonaIdentificada,
            "/vision/id_cara",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibido: {msg.nombre_persona}')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()