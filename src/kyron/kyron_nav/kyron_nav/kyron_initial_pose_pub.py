# kyron_initial_pose_pub.py
# se suscribe al topic /initialpose para fijar la posición inicial del robot.
# será el encargado de publicar la localización inicial del robot en el mapa.

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class Publisher(Node):

    def __init__(self):
        """Inicializa el nodo y configura un temporizador para publicar la pose inicial.

        Crea un publicador en el tópico /initialpose y establece un temporizador que llama
        al método callback cada 0.5 segundos para publicar la pose.
        """
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.callback)

    def callback(self):
        """Publica un mensaje PoseWithCovarianceStamped con la posición inicial fija.

        El mensaje contiene una pose fija en el marco 'map' con coordenadas (x, y) y
        orientación w. Se publica en el tópico /initialpose.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.5846771001815796 
        msg.pose.pose.position.y = 15.311471939086914
        msg.pose.pose.orientation.w = 0.0038585662841796875 
        self.get_logger().info(f'Publishing  Initial Position  \n X= {msg.pose.pose.position.x } \n Y={msg.pose.pose.position.y} \n W = {msg.pose.pose.orientation.w} ')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    try:
        rclpy.spin_once(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()