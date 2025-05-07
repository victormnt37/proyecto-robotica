import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

 
class ID_Cuerpo(Node):
   
   

    def __init__(self):
        """ID_Cuerpo
            Identifica cuerpos y aplica un ROI sobre este. 
            Sobre el ROI identifica segun el color de la ropa si la persona es enfermera o paciente
        """  
        super().__init__('ID_Cuerpo')
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


            # Cargamos los clasificadores
            fullbody=cv2.CascadeClassifier('src/kyron/kyron_vision/clasificadores/haarcascade_fullbody.xml')

            # Detectamos cuerpos
            bodies = fullbody.detectMultiScale(img_gray, 1.1, 5)
            _,res = self.includes_x_color(cv_image)




            # Para cada cuerpo detectado, dibujamos un rectángulo
            for (x,y,w,h) in bodies:
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
                roi = cv_image[y:y+h, x:x+w]

                color,res = self.includes_x_color(roi)
                cv2.putText(roi, f"Color: {color}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)


                

        except CvBridgeError as e:
            print(e)

        cv2.imshow("Imagen capturada por el robot", cv_image)
        cv2.imshow("debug",res)

                
        cv2.waitKey(1) 


    def includes_x_color(self, img):
        """
        Detecta si hay color blanco en la imagen ROI.
        """

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        # Rango ajustado para blanco
        minimo = np.array([0, 0, 180])
        maximo = np.array([180, 50, 255])

        mask = cv2.inRange(hsv_img, minimo, maximo)
        result = cv2.bitwise_and(img, img, mask=mask)

        # Si hay muchos píxeles blancos, lo consideramos detectado
        if cv2.countNonZero(mask) > 550:
            return 'blanco' , result
        else:
            return 'ninguno'



def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = ID_Cuerpo()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
