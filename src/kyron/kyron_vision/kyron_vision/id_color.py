import rclpy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ID_Color(Node):   

    def __init__(self):

        super().__init__('ID_Color')

        topic_img_sim='/camera/image_raw'
        topic_img_irl='/image'
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,topic_img_sim,self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    
    def detectar_color(self,img_bgr, color_bgr, tolerancia_h=10):
        """
        img_bgr: Imagen en formato BGR.
        color_bgr: Lista [B, G, R] del color a detectar.
        tolerancia_h: Cuánto se permite variar el matiz (H) en HSV.
        """
        color = np.uint8([[color_bgr]])
        color_hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)[0][0]

        lower = np.array([color_hsv[0] - tolerancia_h, 50, 50])
        upper = np.array([color_hsv[0] + tolerancia_h, 255, 255])

        hsv_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, lower, upper)

        result = cv2.bitwise_and(img_bgr, img_bgr, mask=mask)

        return mask, result

    def camera_callback(self, data):
        """
        Esta función procesa la imagen capturada por el robot.
        En este caso, detecta un color específico (por ejemplo, azul).
        """

        try:
            # Convertimos la imagen ROS a formato OpenCV (BGR)
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Detectar un color en la imagen (ej. azul: [255, 0, 0])
        mask, resultado = self.detectar_color(cv_image, [255, 0, 0])  # Azul puro

        # Mostrar la imagen original, la máscara, y el resultado
        cv2.imshow("Imagen capturada por el robot", cv_image)
        cv2.imshow("Mascara del color", mask)
        cv2.imshow("Color detectado", resultado)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = ID_Color()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
