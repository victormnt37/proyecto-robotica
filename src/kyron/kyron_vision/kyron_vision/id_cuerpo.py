import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from kyron_interface.msg import ConteoPersonas
 
class ID_Cuerpo(Node):

    def __init__(self):
        """ID_Cuerpo
            Identifica cuerpos y aplica un ROI sobre este. 
            Sobre el ROI identifica segun el color de la ropa y si lleva un badge
            si la persona es enfermera o paciente.

            Este luego los cuenta  y manda el conteo a un topic
        """  
        super().__init__('ID_Cuerpo') 

         # Declarar el parámetro con valor por defecto

        self.declare_parameter('modo', 'sim')  # puede ser 'sim' o 'irl'
        
        modo = self.get_parameter('modo').get_parameter_value().string_value
        

        # Elegir el topic en función del parámetro
        topic_img_sim = '/camera/image_raw'
        topic_img_irl = '/image'
        topic_seleccionado = topic_img_sim if modo == 'sim' else topic_img_irl

        self.get_logger().info(f"Usando el topic: {topic_seleccionado}")


        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,topic_seleccionado,self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.publisher_id_cuerpo= self.create_publisher(ConteoPersonas,"/vision/id_cuerpo",10)



    def camera_callback(self,data):
        """camera_calback

        Esta funcion aplica la lo queremos hacer con la imagen capturada por el robot, en este caso, identificar caras.

        """

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


            # Cargamos los clasificadores
            fullbody=cv2.CascadeClassifier('src/kyron/kyron_vision/clasificadores/haarcascade_fullbody.xml')

            # Detectamos cuerpos
            bodies = fullbody.detectMultiScale(img_gray, 1.1, 5)


            #Inicializamos nuestras variables    
            conteo_frame_actual= {
                "Internad@": 0,
                "Dr/Dra": 0,
                "Enfermer@": 0,
                "Cirujan@": 0,
                "Paciente":0,
            }

            # Para cada cuerpo detectado, dibujamos un rectángulo
            for (x,y,w,h) in bodies:
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
                roi = cv_image[y:y+h, x:x+w]
                persona_detectada=self.id_por_uniforme(roi)

                cv2.putText(roi, f"{persona_detectada}", (5, 15), cv2.FONT_HERSHEY_PLAIN, 0.8, (0,255,0), 1)
                

                if persona_detectada in conteo_frame_actual:
                    conteo_frame_actual[persona_detectada]+= 1

                self.publicar_conteo(conteo_frame_actual)


        except CvBridgeError as e:
            print(e)

        cv2.imshow("Identificadno personas por el cuerpo", cv_image)
                
        cv2.waitKey(1) 


    def id_por_uniforme(self,img):

        """identifica si una persona es doctora o enfermera basado en su uniforme

        Args:
            img (img): imagen a evaluar

        Returns:
            string que dice si es enfermera o medica o visitante
        """

        #Hashmap de colores con sus respetivos rangos:
        color = {
                    'blanco': self.get_color_range([255, 255, 255]),  # BGR Dr
                    'celeste': self.get_color_range([255, 191, 0]),    # BGR Internado
                    'azul_oscuro': self.get_color_range([177, 106, 64]),  # BGR Enfermer
                    'verde_azulado': self.get_color_range([178, 162, 42])  # BGR Cirujano
                }
        
        #booleano
        has_badge=self.has_badge(img)


        if self.includes_x_color(img,color['celeste']):
            return "Internad@"
        
        if self.includes_x_color(img,color['blanco'])and has_badge:
           return "Dr/Dra"
        
        elif self.includes_x_color(img,color['azul_oscuro']) and has_badge:
            return "Enfermer@"
        
        elif self.includes_x_color(img,color['verde_azulado']) and has_badge:
            return "Cirujan@"
        
        #Si no encuentra ningun color
        return "Paciente"

    def publicar_conteo(self,conteo):
        """publica el conteo actual de cada cuerpo detectado 
        (hay 5 doctores, hay 5 pacientes,etc)

        Args:
            conteo (dictionary{string:int}): diccionario con el tipo de persona y cuantas veces aparece.
        """
        #Creamos nuestro mensaje
        msg = ConteoPersonas()
        msg.doctores= conteo['Dr/Dra']
        msg.pacientes= conteo['Paciente']
        msg.internados=conteo['Internad@']
        msg.enfermeros=conteo['Enfermer@']
        msg.cirujanos=conteo['Cirujan@']

        self.publisher_id_cuerpo.publish(msg)

        self.get_logger().info(f'Num_doctores: {msg.doctores} | Num_pacientes: {msg.pacientes} | Num_internados: {msg.internados} | Num_enfermeros: {msg.enfermeros} | Num_cirujanos: {msg.cirujanos}')


    def includes_x_color(self, img,rango_colores):
        """
        Devuelve true si hay suficientes pixeles en la imagen con el color que se busca
        
        Args:
        img(img)= imagen que queremos evaluar

        rango_colores(tuple[float],[float])= rango de colores para que pueda encontrarlos.

        Returns:
        T/F
        """
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Rango ajustado para el color selecionado
        minimo, maximo = rango_colores

        mask = cv2.inRange(hsv_img, minimo, maximo)
        #result = cv2.bitwise_and(img, img, mask=mask)
        

        # Si hay muchos píxeles del color buscado , lo consideramos detectado
        if cv2.countNonZero(mask) > 900: #Cuenta los pixeles en la mascara aplicandole el rang.
            return True
        else:
            return False
    

    def has_badge(self, img, min_area=300, aspect_ratio_range=(0.4, 2.5)):
        """
        Detecta si existe un badge cuadrado o rectangular en la imagen (como un gafete).

        Args:
            img: Imagen a evaluar (BGR)
            min_area: Área mínima para considerar un contorno como badge
            aspect_ratio_range: Rango permitido para relación ancho/alto

        Returns:
            bool: True si se encuentra un badge válido
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))

        contornos, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contorno in contornos:
            if cv2.contourArea(contorno) < min_area:
                continue

            peri = cv2.arcLength(contorno, True)
            approx = cv2.approxPolyDP(contorno, 0.02 * peri, True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h

                if aspect_ratio_range[0] <= aspect_ratio <= aspect_ratio_range[1]:
                    hull = cv2.convexHull(contorno)
                    hull_area = cv2.contourArea(hull)
                    if hull_area > 0:
                        solidity = float(cv2.contourArea(contorno)) / hull_area
                        if solidity > 0.85:
                            return True

        return False


    
    def get_color_range(self,color_BGR):
        """Recibe un codigo de colores BGR y le saca su rango.

        Args:
            color_BGR (list): codigo bgr del color que queremos conseguir su color range.

        Returns:
            lowerlimit,upperlimit(list): rango de colores para su detection
        """

        c = np.uint8([[color_BGR]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue = hsvC[0][0][0]  # Get the hue value

        # Handle red hue wrap-around
        if hue >= 165:  # Upper limit for divided red hue
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([180, 255, 255], dtype=np.uint8)

        elif hue <= 15:  # Lower limit for divided red hue
            lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

        return lowerLimit, upperLimit



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
