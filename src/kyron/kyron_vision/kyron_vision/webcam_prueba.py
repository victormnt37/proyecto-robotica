import cv2
import numpy as np
import face_recognition

class FaceRecognizer:
    def __init__(self):
        """
        FaceRecognizer

        Este script identifica rostros en los frames capturados por una webcam usando OpenCV y face_recognition.
        """
        # Inicializar la captura de video desde la webcam (índice 0 por defecto)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: No se pudo abrir la webcam")
            raise Exception("Error al abrir la webcam")

        self.caras_conocidas = self.cargar_caras()

    def cargar_caras(self):
        """
        Carga las caras conocidas y devuelve un diccionario: {nombre: encoding}

        Return:
            diccionario con nombre e encodings
        """
        caras_conocidas = {
            "Ozuna": "ozuna.jpg", 
            "Lamine": "LamineYamal.jpg",
            "Ariel": "Alejandro.jpeg",
            "Juan": "Juan.jpeg",
            "Jorge": "Jorge.jpeg",
            "Pau": "Pau.jpeg",
            "Diego": "Armando.jpeg",
        }
        diccionario_caras = {}

        for nombre, archivo in caras_conocidas.items():
            try:
                imagen = face_recognition.load_image_file(f'pruebas/facerecognition/caras/{archivo}')
                encodings = face_recognition.face_encodings(imagen)

                if encodings:  # Verifica que se haya encontrado al menos una cara
                    diccionario_caras[nombre] = encodings[0]
                else:
                    print(f"[AVISO] No se detectó ninguna cara en {archivo}")
            except Exception as e:
                print(f"[ERROR] No se pudo cargar {archivo}: {e}")

        return diccionario_caras

    def reconocer_caras(self, img):
        """
        Reconoce a la persona en la imagen. En nuestro caso, dentro del ROI.

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

    def run(self):
        """
        Procesa los frames de la webcam y realiza la identificación de caras.
        """
        while True:
            try:
                # Leer un frame de la webcam
                ret, cv_image = self.cap.read()
                if not ret:
                    print("Error: No se pudo capturar la imagen de la webcam")
                    break

                img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

                # Cargamos los clasificadores
                face_cascade = cv2.CascadeClassifier('pruebas/facerecognition/clasificadores/haarcascade_frontalface_default.xml')
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

                    print(f'Nombre detectado: {nombre}')  # Verifica que no sea vacío

                    # Dibujar rectángulo y texto
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (35, 101, 51), 2)
                    cv2.putText(cv_image, nombre, (x, max(y - 10, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

                cv2.imshow("Identificando Caras", cv_image)
                
                # Salir si se presiona 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                print(f"Error procesando la imagen: {e}")

        # Liberar recursos
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    try:
        recognizer = FaceRecognizer()
        recognizer.run()
    except KeyboardInterrupt:
        print("Fin del programa!")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()