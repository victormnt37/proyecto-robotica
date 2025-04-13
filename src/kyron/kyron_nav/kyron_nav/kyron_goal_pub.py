# introducción de coordenadas por terminal para que el robot se mueva hacia ellas
# ejemplo:
# ros2 run kyron_nav kyron_goal_pub 1.7 13.0 0.003

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse

class GoalPublisher(Node):
    """Nodo ROS 2 que publica una meta de navegación en el tópico /goal_pose."""
    def __init__(self, x, y, w):
        """Inicializa el nodo y publica la meta de navegación.

        Args:
            x (float): Coordenada X de la meta en el marco 'map'.
            y (float): Coordenada Y de la meta en el marco 'map'.
            w (float): Componente w del cuaternión de orientación (0 a 1).
        """
        super().__init__('goal_pub_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.publish_goal(x, y, w)

    def publish_goal(self, x, y, w):
        """Publica un mensaje PoseStamped con la meta de navegación especificada.

        Args:
            x (float): Coordenada X de la meta.
            y (float): Coordenada Y de la meta.
            w (float): Componente w del cuaternión de orientación.

        El mensaje se publica en el tópico /goal_pose con frame_id 'map' y el
        timestamp actual.
        """
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = w

        self.get_logger().info(f'Publicando destino:\n X={x} \n Y={y} \n W={w}')
        self.publisher_.publish(msg)

def main():
    parser = argparse.ArgumentParser(description="Publica una meta de navegación en /goal_pose")
    parser.add_argument("x", type=float, help="Coordenada X del destino")
    parser.add_argument("y", type=float, help="Coordenada Y del destino")
    parser.add_argument("w", type=float, help="Orientación W (0 a 1)")
    args = parser.parse_args()

    rclpy.init()
    
    goal_publisher = GoalPublisher(args.x, args.y, args.w)
    
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

