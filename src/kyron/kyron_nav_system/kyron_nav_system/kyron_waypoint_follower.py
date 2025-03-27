import rclpy  # Importa la biblioteca principal de ROS 2 en Python
from rclpy.node import Node  # Permite crear nodos en ROS 2
from rclpy.action import ActionClient  # Permite comunicarse con acciones en ROS 2
from geometry_msgs.msg import PoseStamped  # Mensaje para definir una posición y orientación en el espacio
from nav2_msgs.action import FollowWaypoints  # Mensaje de acción para seguir una lista de waypoints

class WaypointFollower(Node):
    """Nodo de ROS 2 que envía una lista de waypoints al robot para que los siga."""
    
    def __init__(self):
        """Inicializa el nodo y crea el cliente de acción para seguir waypoints."""
        super().__init__('waypoint_follower')  # Nombre del nodo
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')  # Cliente para la acción de navegación

    def create_waypoints(self):
        """Crea una lista de waypoints como mensajes PoseStamped."""

        waypoints = []  # Lista donde se almacenarán los waypoints
        
        # Coordenadas de los waypoints (x, y, orientación)
        coords = [
            (0.5, 0.5, 0.0),
            (1.0, 1.0, 0.0),
            (1.5, 0.5, 0.0)
        ]

        # Crear los mensajes PoseStamped para cada waypoint
        for x, y, theta in coords:
            pose = PoseStamped()
            pose.header.frame_id = "map"  # Se define el marco de referencia del mapa
            pose.header.stamp = self.get_clock().now().to_msg()  # Marca de tiempo actual
            pose.pose.position.x = x  # Asigna la posición en X
            pose.pose.position.y = y  # Asigna la posición en Y
            pose.pose.orientation.w = 1.0  # Define la orientación (sin rotación)
            waypoints.append(pose)  # Agrega el waypoint a la lista
        
        return waypoints  # Devuelve la lista de waypoints

    def send_goal(self, waypoints):
        """Envía la lista de waypoints al servidor de la acción FollowWaypoints."""
        
        goal_msg = FollowWaypoints.Goal()  # Crea el mensaje de la acción
        goal_msg.poses = waypoints  # Asigna los waypoints a la meta

        # Espera a que el servidor de la acción esté disponible
        self.client.wait_for_server()
        self.get_logger().info("Enviando waypoints al servidor...")

        # Envía la meta de forma asíncrona
        send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)  # Maneja la respuesta del servidor
        
        


    def goal_response_callback(self, future):
        """Maneja la respuesta del servidor cuando se envían los waypoints."""
        
        goal_handle = future.result()  # Obtiene la respuesta del servidor
        
        if not goal_handle.accepted:
            self.get_logger().error("La solicitud de waypoints fue rechazada.")
            return  # Sale si la solicitud es rechazada
        
        self.get_logger().info("Waypoints aceptados, esperando resultado...")

        # Espera el resultado final de la acción de forma asíncrona
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Muestra el progreso de los waypoints mientras el robot se mueve."""
        
        self.get_logger().info(f"Waypoint actual: {feedback_msg.feedback.current_waypoint}")

    def result_callback(self, future):
        """Muestra el resultado final una vez que el robot completa todos los waypoints."""
        
        result = future.result().result  # Obtiene el resultado de la acción
        self.get_logger().info(f"Waypoints completados con código: {result}")

def main(args=None):
    """Función principal que inicializa el nodo y envía los waypoints."""
    
    rclpy.init(args=args)  # Inicializa ROS 2
    node = WaypointFollower()  # Crea una instancia del nodo
    
    waypoints = node.create_waypoints()  # Genera los waypoints
    node.send_goal(waypoints)  # Envía los waypoints al servidor
    
    rclpy.spin(node)  # Mantiene el nodo activo hasta que se detenga manualmente
    node.destroy_node()  # Destruye el nodo antes de cerrar ROS 2
    rclpy.shutdown()  # Apaga ROS 2 correctamente

if __name__ == '__main__':
    main()  # Llama a la función principal si el script se ejecuta directamente
