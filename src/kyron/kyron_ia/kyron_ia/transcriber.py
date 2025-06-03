import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class transcriber(Node):
    def __init__(self):
        super().__init__('transcriber')
        self.publisher_ = self.create_publisher(String, 'voz/transcripcion', 10)
        self.recognizer = sr.Recognizer()

        # Opcional: ajustar sensibilidad y tiempos
        self.recognizer.energy_threshold = 300
        self.recognizer.pause_threshold = 0.5

        # Ejecutar cada 10s (ajustable)
        self.timer = self.create_timer(10.0, self.transcribir_audio)
        self.get_logger().info("🎙️ Nodo de transcripción por voz iniciado.")

    def transcribir_audio(self):
        with sr.Microphone() as source:
            self.get_logger().info("🎤 Habla ahora (máx. 5s)...")
            try:
                audio = self.recognizer.listen(source, timeout=3, phrase_time_limit=5)
            except sr.WaitTimeoutError:
                self.get_logger().warn("⌛ No se detectó voz a tiempo.")
                return

        try:
            texto = self.recognizer.recognize_google(audio, language="es-ES")
            self.get_logger().info(f"📝 Texto detectado: {texto}")

            msg = String()
            msg.data = texto
            self.publisher_.publish(msg)
            self.get_logger().info("📤 Transcripción publicada en /voz/transcrita")

        except sr.UnknownValueError:
            self.get_logger().warn("❌ No se entendió el audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"🚫 Error con el servicio de Google: {e}")

def main(args=None):
    rclpy.init(args=args)
    nodo = transcriber()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.destroy_node()
        rclpy.shutdown()
        print("🛑 Nodo detenido por el usuario.")

if __name__ == '__main__':
    main()
