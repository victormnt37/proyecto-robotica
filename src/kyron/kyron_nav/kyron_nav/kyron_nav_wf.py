import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class WaypointLifecycleFollower(Node):
    def __init__(self):
        super().__init__('waypoint_lifecycle_follower')
        self._action_client = None
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._waypoints = None
        self.get_logger().info('Waypoint Lifecycle Client creado')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configura el Action Client"""
        self.get_logger().info("Configurando waypoint follower...")
        try:
            self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
            self._waypoints = self.define_waypoints()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error en configuración: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activa el nodo e inicia la patrulla"""
        self.get_logger().info("Iniciando patrulla...")
        try:
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('Action server no disponible')
                return TransitionCallbackReturn.FAILURE

            self.send_waypoints(self._waypoints)
            return super().on_activate(state)
        except Exception as e:
            self.get_logger().error(f"Error al activar: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Cancela la patrulla actual"""
        self.get_logger().info("Cancelando patrulla...")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Limpia recursos"""
        self.get_logger().info("Limpiando recursos...")
        self._action_client = None
        self._goal_handle = None
        self._waypoints = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Apagado del nodo"""
        self.get_logger().info("Apagando waypoint follower...")
        self._action_client = None
        self._goal_handle = None
        self._waypoints = None
        return TransitionCallbackReturn.SUCCESS

    def define_waypoints(self):
        """ Define una lista de puntos y los prepara para que se pueda utilizar como waypoints
        quel robot pueda seguir

        Devuelve
        List[pose]
        
        ."""        
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

    def send_waypoints(self, waypoints):
        """Envia la lista de puntos al action server."""
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Enviando waypoints al servidor...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Maneja la respuesta del servidor."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rechazado')
            self.deactivate()
            return

        self.get_logger().info('Goal aceptado, esperando resultado...')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Maneja el feedback del servidor."""
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Waypoint actual: {current_waypoint}')

    def get_result_callback(self, future):
        """Maneja los resultados."""
        result = future.result().result
        self.get_logger().info('Resultados recibidos:')
        self.get_logger().info(f'Waypoints fallidos: {len(result.missed_waypoints)}')
        self.get_logger().info(f'Error code: {result.error_code}')
        self.deactivate()  # vuelve a estado inactivo al completarse

def main(args=None):
    rclpy.init(args=args)
    
    lifecycle_node = WaypointLifecycleFollower()
    
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