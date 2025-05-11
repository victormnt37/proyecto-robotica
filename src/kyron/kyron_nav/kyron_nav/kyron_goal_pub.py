# introducción de coordenadas por terminal para que el robot se mueva hacia ellas
# ejemplo:
# ros2 run kyron_nav kyron_goal_pub 1.7 13.0 0.003

import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import argparse

class GoalLifecycleActionClient(Node):
    def __init__(self, x, y, w):
        super().__init__('goal_lifecycle_action_client')
        self.x = x
        self.y = y
        self.w = w
        self._action_client = None
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando Action Client...")
        try:
            self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error al configurar: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activando y enviando goal...")
        try:
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Servidor de acción no disponible")
                return TransitionCallbackReturn.FAILURE

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = self.x
            goal_msg.pose.pose.position.y = self.y
            goal_msg.pose.pose.orientation.w = self.w

            self.get_logger().info(f'Enviando destino: x={self.x}, y={self.y}, w={self.w}')
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self._goal_response_callback)
            
            return super().on_activate(state)
        except Exception as e:
            self.get_logger().error(f"Error al activar: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # desactiva el nodo y cancela la navegación en curso
        self.get_logger().info("Desactivando y cancelando goal...")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando recursos...")
        self._action_client = None
        self._goal_handle = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Apagando nodo...")
        self._action_client = None
        self._goal_handle = None
        return TransitionCallbackReturn.SUCCESS

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('La meta fue rechazada.')
            self.deactivate()  # estado inactivo
            return

        self.get_logger().info('Meta aceptada. Esperando resultado...')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
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

        self.deactivate()  # vuelve a estado inactivo al completarse

def main():
    parser = argparse.ArgumentParser(description="Lifecycle Node para enviar goals de navegación")
    parser.add_argument("x", type=float, help="Coordenada X del destino")
    parser.add_argument("y", type=float, help="Coordenada Y del destino")
    parser.add_argument("w", type=float, help="Orientación W")
    args = parser.parse_args()

    rclpy.init()
    
    lifecycle_node = GoalLifecycleActionClient(args.x, args.y, args.w)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(lifecycle_node)
    
    try:
        if not lifecycle_node.configure().success:
            raise RuntimeError("Fallo en configuración")
        
        if not lifecycle_node.activate().success:
            raise RuntimeError("Fallo en activación")
        
        while rclpy.ok() and lifecycle_node._get_result_future is None:
            executor.spin_once(timeout_sec=0.1)
        
        while rclpy.ok() and not lifecycle_node._get_result_future.done():
            executor.spin_once(timeout_sec=0.1)
            
    except Exception as e:
        lifecycle_node.get_logger().error(f"Error: {str(e)}")
    finally:
        if lifecycle_node.state == State.PRIMARY_STATE_ACTIVE:
            lifecycle_node.deactivate()
        if lifecycle_node.state == State.PRIMARY_STATE_INACTIVE:
            lifecycle_node.cleanup()
        
        executor.shutdown()
        lifecycle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()