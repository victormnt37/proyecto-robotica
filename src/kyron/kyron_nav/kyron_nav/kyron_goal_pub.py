# introducción de coordenadas por terminal para que el robot se mueva hacia ellas
# ejemplo:
# ros2 run kyron_nav kyron_goal_pub 1.7 13.0 0.003

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import argparse
import time

class GoalActionClient(Node):
    def __init__(self, x, y, w):
        super().__init__('goal_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # estados del robot
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.command_sub = self.create_subscription(String, '/robot_command', self.command_callback, 10)
        self.goal_handle = None
        self.current_command = 'GOING_TO_ROOM'

        self.x = x
        self.y = y
        self.w = w

    def publish_state(self, state_str):
        msg = String()
        msg.data = state_str
        self.state_pub.publish(msg)
        self.get_logger().info(f'[ESTADO] {state_str}')

    def command_callback(self, msg):
        self.get_logger().info(f'[COMANDO RECIBIDO] {msg.data}')
        if msg.data in ['STOP', 'PATROLLING']:
            self.current_command = msg.data
            if self.goal_handle:
                self.get_logger().warn('Cancelando navegación por interrupción...')
                self.goal_handle.cancel_goal_async()
                self.publish_state('INTERRUPTED')

    def send_goal(self):
        self._action_client.wait_for_server()

        if self.current_command != 'GOING_TO_ROOM':
            self.get_logger().warn('Otra acción está activa. Cancelando movimiento.')
            self.publish_state('INTERRUPTED')
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.x
        goal_msg.pose.pose.position.y = self.y
        goal_msg.pose.pose.orientation.w = self.w

        self.publish_state('GOING_TO_ROOM')
        self.get_logger().info(f'Enviando destino: x={self.x}, y={self.y}, w={self.w}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('La meta fue rechazada.')
            rclpy.shutdown()
            return

        self.get_logger().info('Meta aceptada. Esperando a que el robot llegue al destino...')
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 3:
            self.get_logger().info('Navegación completada.')
        elif status == 4:
            self.get_logger().info('La navegación fue cancelada.')
        elif status == 5:
            self.get_logger().error('La navegación falló.')
        else:
            self.get_logger().info(f"Resultado con status: {status}")

        self.publish_state('IDLE')
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Envía una goal pose y espera a que se complete")
    parser.add_argument("x", type=float, help="Coordenada X del destino")
    parser.add_argument("y", type=float, help="Coordenada Y del destino")
    parser.add_argument("w", type=float, help="Orientación W")
    args = parser.parse_args()

    rclpy.init()
    node = GoalActionClient(args.x, args.y, args.w)
    time.sleep(0.5)  # tiempo breve para recibir posibles comandos antes de empezar
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
