import rclpy
import cv2
import numpy as np
import face_recognition

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile


class ID_Cara(Node):  
    """
    ID_Cara

    Este nodo se encarga de identificar rostros en las fotos capturadas por el robot usando open cv y face recognition

    """ 

    def __init__(self):

        topic_img_sim='/camera/image_raw'
        topic_img_irl='/image'

        super().__init__('ID_Cara')
        
        self.caras_conocidas=self.cargar_caras()
        
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,topic_img_sim,self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
    def camera_callback(self,data):

        """camera_calback

        Esta funcion aplica la lo queremos hacer con la imagen capturada por el robot, en este caso, identificar caras.

        """

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
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(35,101,51),1)
                roi = cv_image[y:y+h, x:x+w]
                print(f"He detectado a: {self.reconocer_caras(roi)}")



            #cv2.putText(roi, f"{self.reconocer_caras(roi)}", (5, 15), cv2.FONT_HERSHEY_PLAIN, 0.4, (0,255,0), 1)
                

        except CvBridgeError as e:
            print(e)

        cv2.imshow("Imagen capturada por el robot", cv_image)
                
        cv2.waitKey(1)   
    
    
    def reconocer_caras(self,img):
        """Reconoce a la persona en la imagen. EN nuestro caso, dentro del ROI

        Args:
            img: imagen que queremos evaluar
        
        Returns:
            nombre de la persona
        """
        if not self.caras_conocidas:
            return 'Sin caras cargadas'
        
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        encodings = face_recognition.face_encodings(rgb_img)


        if not encodings:
            return "Sin rostro detectado"
        
        cara = encodings[0]

        for nombre, encoding in self.known_faces.items():
            match = face_recognition.compare_faces([encoding], cara)[0]
            if match:
                return nombre
            
        return "Desconocido"


    def cargar_caras(self):
        """
        Carga las caras conocidas y devuelve un diccionario: {nombre: encoding}

        Return:
            diccionario con nombre e encondings
        """
        #clave:nombre | valor: nombre de archvio
        caras_conocidas = {"Maria": "Maria.png", "Pedro": "Pedro.png","Pablo":"Pablo.jpeg"}
        diccionario_caras = {}

        for nombre, archivo in caras_conocidas.items():
            try:
                imagen = face_recognition.load_image_file(f'src/kyron/kyron_vision/caras_test/{archivo}')
                encodings = face_recognition.face_encodings(imagen)

                if encodings:  # Verifica que se haya encontrado al menos una cara
                    diccionario_caras[nombre] = encodings[0]
                else:
                    print(f"[AVISO] No se detectó ninguna cara en {archivo}")
            except Exception as e:
                print(f"[ERROR] No se pudo cargar {archivo}: {e}")

        return diccionario_caras


def main(args=None):

    rclpy.init(args=args)    
    img_converter_object = ID_Cara()    
       
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    
    cv2.destroyAllWindows() 
    

if __name__ == '__main__':
    main()
