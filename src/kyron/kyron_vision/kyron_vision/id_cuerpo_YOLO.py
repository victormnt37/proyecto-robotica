import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from kyron_interface.msg import ConteoPersonas
from ultralytics import YOLO
import os

class ID_Cuerpo_YOLO(Node):

    def __init__(self):
        """ID_Cuerpo
            Identifica personal medico y pacientes usando el modelo entrenado con yolo

            Este luego los cuenta  y manda el conteo a un topic
        """  
        super().__init__('ID_Cuerpo_YOLO') 

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
        
        #Cargamos el modelo
        #modelo_path = os.path.join(os.path.dirname(__file__), 'modelos', 'kyron_PD.pt')
        self.model = YOLO("src/kyron/kyron_ia/modelos/kyron_PD_model/kyron_PD_model.pt")


        #self.publisher_id_cuerpo= self.create_publisher(ConteoPersonas,"/vision/id_cuerpo",10)


    def camera_callback(self,data):
        """camera_calback

        Esta funcion aplica la lo queremos hacer con la imagen capturada por el robot, en este caso, identificar caras.

        """

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
            results= self.model(cv_image)

        except CvBridgeError as e:
            print(e)
    
        cv2.imshow("YOLO Detection", results[0].plot())
        cv2.waitKey(1)

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



def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = ID_Cuerpo_YOLO()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Id_cuerpo_YOLO ha cerrado!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
