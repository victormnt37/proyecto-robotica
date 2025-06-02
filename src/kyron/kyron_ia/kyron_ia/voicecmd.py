import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import speech_recognition as sr

class VoiceCmd(Node):

    def __init__(self):
        super().__init__('voice_cmd')
        
        # Configuración del reconocedor de voz
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 300
        self.recognizer.pause_threshold = 0.5
        
        # Publicador para comandos de movimiento
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptor a la odometría
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        
        # Variables para control de movimiento
        self.current_position = None
        self.current_orientation = None
        self.target_distance = 0.0
        self.target_angle = 0.0
        self.moving = False
        self.rotating = False
        
        # Timer para reconocimiento de voz
        self.timer = self.create_timer(10.0, self.transcribir_audio)
        self.get_logger().info("🎙️ Nodo de comandos por voz iniciado.")

    def odom_callback(self, msg):
        """Actualiza la posición y orientación actual del robot"""
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convierte quaternion a ángulos de Euler (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z  # En radianes

    def transcribir_audio(self):
        """Captura y procesa comandos de voz"""
        with sr.Microphone() as source:
            self.get_logger().info("🎤 Di un comando (ej: 'adelante', 'derecha'...)")
            try:
                audio = self.recognizer.listen(source, timeout=3, phrase_time_limit=5)
            except sr.WaitTimeoutError:
                self.get_logger().warn("⌛ No se detectó voz a tiempo.")
                return

        try:
            texto = self.recognizer.recognize_google(audio, language="es-ES").lower()
            self.get_logger().info(f"📝 Comando detectado: {texto}")
            self.procesar_comando(texto)

        except sr.UnknownValueError:
            self.get_logger().warn("❌ No se entendió el audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"🚫 Error con el servicio de Google: {e}")

    def procesar_comando(self, comando):
        """Ejecuta acciones basadas en comandos de voz"""
        # Comandos de movimiento básico
        if any(palabra in comando for palabra in ["adelante", "avanza", "avanzar"]):
            self.mover_lineal(0.5)  # 0.5 metros adelante
            
        elif any(palabra in comando for palabra in ["atrás", "retrocede", "retroceder"]):
            self.mover_lineal(-0.5)  # 0.5 metros atrás
            
        elif any(palabra in comando for palabra in ["derecha", "gira derecha"]):
            self.girar_angulo(-math.pi/2)  # 90 grados derecha
            
        elif any(palabra in comando for palabra in ["izquierda", "gira izquierda"]):
            self.girar_angulo(math.pi/2)  # 90 grados izquierda
            
        elif "para" in comando or "detente" in comando:
            self.detener_movimiento()
            
        else:
            self.get_logger().warn(f"⚠️ Comando no reconocido: '{comando}'")

    def mover_lineal(self, distancia):
        """Mueve el robot una distancia lineal específica (en metros)"""
        if self.current_position is None:
            self.get_logger().warn("Odometría no disponible")
            return
            
        self.target_distance = distancia
        start_x = self.current_position.x
        start_y = self.current_position.y
        self.moving = True
        
        # Publicar comando de velocidad
        twist = Twist()
        twist.linear.x = 0.2 if distancia > 0 else -0.2  # Velocidad constante
        self.cmd_vel_pub.publish(twist)
        
        # Timer para verificar distancia recorrida
        self.move_timer = self.create_timer(0.1, lambda: self.check_distance(start_x, start_y))

    def check_distance(self, start_x, start_y):
        """Verifica si se ha alcanzado la distancia objetivo"""
        if not self.moving or self.current_position is None:
            return
            
        distancia_recorrida = math.sqrt(
            (self.current_position.x - start_x)**2 + 
            (self.current_position.y - start_y)**2
        )
        
        if abs(distancia_recorrida) >= abs(self.target_distance):
            self.detener_movimiento()
            self.move_timer.cancel()
            self.get_logger().info(f"✅ Movimiento completado: {self.target_distance:.2f}m")

    def girar_angulo(self, angulo):
        """Gira el robot un ángulo específico (en radianes)"""
        if self.current_orientation is None:
            self.get_logger().warn("Odometría no disponible")
            return
            
        self.target_angle = angulo
        start_angle = self.current_orientation
        self.rotating = True
        
        # Publicar comando de rotación
        twist = Twist()
        twist.angular.z = 0.5 if angulo > 0 else -0.5  # Velocidad angular constante
        self.cmd_vel_pub.publish(twist)
        
        # Timer para verificar ángulo girado
        self.rotate_timer = self.create_timer(0.1, lambda: self.check_angle(start_angle))

    def check_angle(self, start_angle):
        """Verifica si se ha alcanzado el ángulo objetivo"""
        if not self.rotating or self.current_orientation is None:
            return
            
        angulo_girado = self.current_orientation - start_angle
        # Normalizar el ángulo a [-π, π]
        angulo_girado = (angulo_girado + math.pi) % (2 * math.pi) - math.pi
        
        if abs(angulo_girado) >= abs(self.target_angle):
            self.detener_movimiento()
            self.rotate_timer.cancel()
            self.get_logger().info(f"✅ Rotación completada: {math.degrees(self.target_angle):.1f}°")

    def detener_movimiento(self):
        """Detiene todos los movimientos del robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.moving = False
        self.rotating = False

def main(args=None):
    rclpy.init(args=args)
    nodo = VoiceCmd()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.destroy_node()
        rclpy.shutdown()
        print("🛑 Nodo detenido por el usuario.")

if __name__ == '__main__':
    main()