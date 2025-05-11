import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import State
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CancelLifeCycleNav(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self._cmd_vel_publisher = None
        self._stop_service = None
        self._current_state = "READY"
        
        # Configuración QoS para comandos de velocidad (best effort)
        self._qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configura los recursos del nodo"""
        self.get_logger().info("Configurando nodo de parada...")
        try:
            # Publisher para comandos de velocidad (se activará en on_activate)
            self._cmd_vel_publisher = self.create_lifecycle_publisher(
                Twist, 
                '/cmd_vel', 
                self._qos
            )
            
            # Servicio de parada de emergencia
            self._stop_service = self.create_service(
                Trigger,
                '/emergency_stop',
                self.handle_stop_request
            )
            
            self._current_state = "CONFIGURED"
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Error en configuración: {str(e)}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activa el nodo"""
        self.get_logger().info("Nodo de parada activado")
        self._current_state = "ACTIVE"
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Desactiva el nodo"""
        self.get_logger().info("Nodo de parada desactivado")
        self._current_state = "INACTIVE"
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Limpia los recursos"""
        self.get_logger().info("Limpiando recursos de parada...")
        self._cmd_vel_publisher = None
        self._stop_service = None
        self._current_state = "READY"
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Apagado del nodo"""
        self.get_logger().info("Apagando nodo de parada...")
        self._cmd_vel_publisher = None
        self._stop_service = None
        return TransitionCallbackReturn.SUCCESS

    def handle_stop_request(self, request, response):
        """Maneja las solicitudes de parada de emergencia"""
        if self._current_state != "ACTIVE":
            response.success = False
            response.message = "El nodo no está activo"
            return response
        
        try:
            # Publicar comando de parada (velocidad cero)
            stop_msg = Twist()
            self._cmd_vel_publisher.publish(stop_msg)
            
            response.success = True
            response.message = "Robot detenido correctamente"
            self.get_logger().warn("¡PARADA DE EMERGENCIA ACTIVADA!")
        except Exception as e:
            response.success = False
            response.message = f"Error al detener el robot: {str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    # Crear y ejecutar el nodo
    executor = rclpy.executors.SingleThreadedExecutor()
    lifecycle_node = CancelLifeCycleNav()
    
    try:
        # Configurar el nodo
        if not lifecycle_node.configure().success:
            raise RuntimeError("Fallo en configuración del nodo de parada")
        
        # Activar el nodo
        if not lifecycle_node.activate().success:
            raise RuntimeError("Fallo en activación del nodo de parada")
        
        # Mantener el nodo activo
        executor.add_node(lifecycle_node)
        executor.spin()
        
    except Exception as e:
        lifecycle_node.get_logger().error(f"Error: {str(e)}")
    finally:
        # Limpieza ordenada
        executor.shutdown()
        lifecycle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()