import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Ros2OpenCVImageConverter(Node):   

    def __init__(self):

        topic_img_sim='/camera/image_raw'
        topic_img_irl='/image'

        super().__init__('Ros2OpenCVImageConverter')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,topic_img_sim,self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


            # Cargamos los clasificadores
            face_cascade = cv2.CascadeClassifier('src/kyron/kyron_vision/clasificadores/haarcascade_frontalface_default.xml')
            eye_cascade = cv2.CascadeClassifier('src/kyron/kyron_vision/clasificadores/haarcascade_eye.xml')

            # Detectamos caras
            faces = face_cascade.detectMultiScale(img_gray, 1.1, 5)

            # Para cada cara detectada, dibujamos un rectángulo
            for (x,y,w,h) in faces:
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
                roi_color = cv_image[y:y+h, x:x+w]
                

        except CvBridgeError as e:
            print(e)

        cv2.imshow("Imagen capturada por el robot", cv_image)
                
        cv2.waitKey(1)    

def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = Ros2OpenCVImageConverter()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
