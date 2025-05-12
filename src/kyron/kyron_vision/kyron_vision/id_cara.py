import rclpy
import cv2
import numpy as np
import face_recognition

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from kyron_interface.msg import PersonaIdentificada


class ID_Cara(Node):  
   
    def __init__(self):
        """
        ID_Cara

        Este nodo se encarga de identificar rostros en las fotos capturadas por el robot usando open cv y face recognition

        """ 
        super().__init__('ID_Cara')

        #=======================
        #Carga de caras y topics
        #=======================
        self.declare_parameter('modo', 'sim')  # puede ser 'sim' o 'irl'
        
        modo = self.get_parameter('modo').get_parameter_value().string_value

        # Elegir el topic en función del parámetro
        topic_img_sim = '/camera/image_raw'
        topic_img_irl = '/image'
        topic_seleccionado = topic_img_sim if modo == 'sim' else topic_img_irl

        self.get_logger().info(f"Usando el topic: {topic_seleccionado}")
        
        self.caras_conocidas=self.cargar_caras()
        
        #=======================
        #Suscriber a imagen
        #=======================
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image,topic_seleccionado,self.camera_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        #=======================
        #Publisher
        #=======================
        self.publisher_id_cara= self.create_publisher(PersonaIdentificada,"/vision/id_cara",10)

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
            # Detectamos caras
            faces = face_cascade.detectMultiScale(img_gray, 1.1, 5)

            for (x, y, w, h) in faces:
                pad = 0.2
                x0 = max(int(x - w*pad), 0)
                y0 = max(int(y - h*pad), 0)
                x1 = min(int(x + w*(1+pad)), cv_image.shape[1])
                y1 = min(int(y + h*(1+pad)), cv_image.shape[0])

                roi = cv_image[y0:y1, x0:x1]

                nombre = self.reconocer_caras(roi)

                #print(f'Nombre detectado: {nombre}')  # Verifica que no sea vacío
                self.publicar_persona_identificada(nombre)

                # Dibujar rectángulo y texto
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (35, 101, 51), 2)
                cv2.putText(cv_image, nombre, (x, max(y - 10, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)



        except CvBridgeError as e:
            print(e)

        cv2.imshow("Identificando Caras", cv_image)
                
        cv2.waitKey(1)   
    
    
    def reconocer_caras(self,img):
        """Reconoce a la persona en la imagen. EN nuestro caso, dentro del ROI

        Args:
            img: imagen que queremos evaluar
        
        Returns:
            nombre de la persona
        """
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # 1) Primero pregunto dónde ve rostros
        locations = face_recognition.face_locations(rgb)
        if not locations:
            return "Sin rostro detectado"

        # 2) Ahora sí, codifico usando esas ubicaciones
        encodings = face_recognition.face_encodings(rgb, locations)
        cara_encoding = encodings[0]

        # 3) Comparo con mis caras conocidas
        for nombre, known_encoding in self.caras_conocidas.items():
            match = face_recognition.compare_faces([known_encoding], cara_encoding, tolerance=0.6)[0]
            if match:
                return nombre

        return "Desconocido"
       

    def publicar_persona_identificada(self, nombre):  
        """publica el mensaje con la persona detectada

        Args:
            nombre (string): nombre de la persona detectada
        
        """
        msg = PersonaIdentificada()
        msg.nombre_persona = nombre

        self.publisher_id_cara.publish(msg)
        self.get_logger().info(f'Persona Identificada {nombre}')
        

    def cargar_caras(self):
        """
        Carga las caras conocidas y devuelve un diccionario: {nombre: encoding}

        Return:
            diccionario con nombre e encondings
        """
        #clave:nombre | valor: nombre de archvio

        caras_conocidas = {
            "Ozuna": "ozuna.jpg", 
            "Lamine": "LamineYamal.jpg",
            "Ariel": "Ariel.jpeg",
            "Juan": "Juan.jpeg",
            "Pau": "Pau.jpeg",
            "Victor": "Victor.jpeg",
            "Denys": "Denys.jpeg"
        }
        

        diccionario_caras = {}

        for nombre, archivo in caras_conocidas.items():
            try:
                imagen = face_recognition.load_image_file(f'src/kyron/kyron_vision/caras/{archivo}')
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
