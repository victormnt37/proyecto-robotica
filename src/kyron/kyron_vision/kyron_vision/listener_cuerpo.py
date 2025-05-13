from kyron_interface.msg import ConteoPersonas
import rclpy
from rclpy.node import Node

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_cuerpo_node')
        self.subscription = self.create_subscription(
            ConteoPersonas,
            "/vision/id_cuerpo",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Num_doctores: {msg.doctores}')
        self.get_logger().info(f'Num_pacientes: {msg.pacientes}')
        self.get_logger().info(f'Num_internados: {msg.internados}')
        self.get_logger().info(f'Num_enfermeros: {msg.enfermeros}')
        self.get_logger().info(f'Num_cirujanos: {msg.cirujanos}')


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()