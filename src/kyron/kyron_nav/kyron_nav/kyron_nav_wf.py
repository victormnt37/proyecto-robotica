#Sigue una ruta de waypoints 
#Nota: no va con los waypoints sacados usand publish point pero si losque estan puestos.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('kyron_nav_wf')#nombfd cdl nodo
        
        #cliente de acción para comunicarse con el servidor de waypoints de Nav2
        
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        # self.declare_parameter('waypoints', [])
        
        self.send_waypoints()


        """
        Funcion que define los waypoints para que puedan ser utilizados
        """    
        
    def define_waypoints(self):
        
        waypoints = [
            (4.829135417938232, 1.2413616180419922),     # Waypoint 1
            (4.830772399902344, -5.883827209472656),    # Waypoint 2
            (3.6522417068481445, -21.372636795043945),  # Waypoint 3
            (-5.516279220581055, -24.34018898010254),   # Waypoint 4
            (-5.829804420471191, -14.206602096557617),  # Waypoint 5
            (-5.106757640838623, -1.8345870971679688)    # Waypoint 6
        ]
        
        
        # Lista para almacenar los mensajes de poses objetivo
        goal_poses = []
        
        for x, y in waypoints:
            goal_pose = PoseStamped() #cada coordenada en un mensaje PoseStamped
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0  # Orientación neutral
            goal_poses.append(goal_pose)
        
        return goal_poses
    
    

    def send_waypoints(self):
        """
        Envía la lista de waypoints al servidor de navegación para su ejecución.
        Configura los callbacks para manejar la respuesta, feedback y resultados.
        """
        
        self.client.wait_for_server()#esperamos al servidor
        
        goal_msg = FollowWaypoints.Goal()#creamos el goal
        goal_msg.poses = self.define_waypoints()#le anadimos los waypointd
        
        
        #enviamosel goal
        self.send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        
        

    def goal_response_callback(self, future):
        """
        Callback que maneja la respuesta del servidor al recibir el goal.
        
        Args:
            future: Objeto futuro que contiene el resultado de la petición
        """
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Gola rechazado!')
            return
        self.get_logger().info('Goal acceptado!')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
        
        

    def feedback_callback(self, feedback_msg):
        """
        Callback que recibe feedback periódico durante la ejecución.
        
        Args:
            feedback_msg: Mensaje de feedback con información del progreso
        """
        
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Ejecutando waypoint: {current_waypoint + 1}')
    
    
    def result_callback(self, future):
        """
        Callback que maneja el resultado final de la navegación.
        
        Args:
            future: Objeto futuro que contiene el resultado de la acción
        """
        
        result = future.result().result
        self.get_logger().info('Navegacion completada')


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
