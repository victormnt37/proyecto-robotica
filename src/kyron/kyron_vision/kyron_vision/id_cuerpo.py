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
            #_,res = self.includes_x_color(cv_image)




            # Para cada cuerpo detectado, dibujamos un rectángulo
            for (x,y,w,h) in bodies:
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
                roi = cv_image[y:y+h, x:x+w]

                cv2.putText(roi, f"{self.id_uniforme(roi)}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
   

        except CvBridgeError as e:
            print(e)

        cv2.imshow("Imagen capturada por el robot", cv_image)
        #cv2.imshow("debug",res)

                
        cv2.waitKey(1) 


    def id_uniforme(self,img):
        """identifica si una persona es doctora o enfermera basado en su uniforme

        Args:
            img (_type_): imagen a evaluar

        Returns:
            string que dice si es enfermera o medica o visitante
        """

        #Hashmap de colores con sus respetivos rangos:
        color_ranges = {'blanco': [np.array([0, 0, 180]), np.array([180, 50, 255])],
                        'celeste': [np.array([100, 50, 50]), np.array([120, 255, 255])],
                        'azul_oscuro':[np.array([100, 100, 50]), np.array([120, 255, 150])],
                        'verde_azulado':[np.array([55, 100, 80]), np.array([75, 255, 255])]
                        }


        
        #if self.includes_x_color(img,color_ranges['blanco']):
           #return "Doctor/Doctora"
        
        if self.includes_x_color(img,color_ranges['celeste']):
            return "Paciente"
        
        elif self.includes_x_color(img,color_ranges['azul_oscuro']):
            return "Enfermera/Enfermero"
        
        elif self.includes_x_color(img,color_ranges['verde_azulado']):
            return "Cirujano"
        
        #Si no encuentra ningun color
        return "??"
        

    def includes_x_color(self, img,rango_colores):
        """
        Devuelve true si hay suficientes pixeles en la imagen con el color que se busca

        Args:
        img= imagen que queremos evaluar

        rango_colores= rango de colores para que pueda encontrarlos.

        Returns:
        T/F
        """
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Rango ajustado para el color selecionado
        minimo, maximo = rango_colores

        mask = cv2.inRange(hsv_img, minimo, maximo)
        #result = cv2.bitwise_and(img, img, mask=mask)
        

        # Si hay muchos píxeles del color buscado , lo consideramos detectado
        if cv2.countNonZero(mask) > 800: #Cuenta los pixeles en la mascara aplicandole el rang.
            return True
        else:
            return False



def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = ID_Cuerpo()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Id_cuerpo ha cerrado!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
