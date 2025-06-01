import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from kyron_interface.msg import RiskDT
from ultralytics import YOLO
import os

class ID_riesgo(Node):

    def __init__(self):
        """ID_riesgo
            Identifica si hay algun riesgo en el hospital (agresiones y caidas) y manda un mensaje
            avisando de estas
        """  
        super().__init__('ID_Riesgo') 

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
        self.model = YOLO("src/kyron/kyron_ia/modelos/kyron_RD_model/risk_DT_model.pt")


        self.publisher_id_riesgo= self.create_publisher(RiskDT,"/vision/id_risk",10)


    def camera_callback(self,data):
        """camera_calback

        Esta funcion aplica la lo queremos hacer con la imagen capturada por el robot, en este caso, identificar caras.

        """

        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
            results= self.model(cv_image)

            violencia, caida = self.risk_detected(results)

            self.publicar_deteciones(violencia,caida)

        except CvBridgeError as e:
            print(e)
    
        cv2.imshow("Risk Detection", results[0].plot())
        cv2.waitKey(1)


    def risk_detected(self,results):
        """devuelve true si se detecta algun riesgo

        Args:
            results (list): lista de resultados devueltos por el modelo entrenado

        Returns:
            violencia,caida(boolean): booleanos.
        """
        result=results[0]
        violencia_detectada= False
        caida_detectada= False

        # Obtenemos las clases detectadas
        nombres_clases = result.names  # Diccionario {id: nombre}
        clases_detectadas = result.boxes.cls.tolist()  # Lista de ids (float)

        for clase_id in clases_detectadas:
                nombre_clase = nombres_clases[int(clase_id)]

                if nombre_clase == "violencie":
                    violencia_detectada=True
                if nombre_clase=="fall":
                     caida_detectada=True
        
        return violencia_detectada,caida_detectada

    def publicar_deteciones(self,vd,cd):
        """Publica el mensaje con el estado de violencia y caída."""
        msg = RiskDT()
        msg.violencia_detectada = vd
        msg.caida_detectada = cd

        self.publisher_id_riesgo.publish(msg)

        self.get_logger().info(
            f'Violencia detectada: {msg.violencia_detectada} | Caída detectada: {msg.caida_detectada}')



def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = ID_riesgo()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("id_violencia ha cerrado!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
