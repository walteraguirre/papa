#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class EncoderRotationTester(Node):
    def __init__(self):
        super().__init__('test_giros_encoders')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.state = 'INIT'
        self.start_yaw = 0.0
        self.current_yaw = 0.0
        
        # Objetivo: Girar 90 grados (en radianes)
        self.target_angle_rad = math.radians(90.0) 
        
        # Bucle de control a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("=========================================")
        self.get_logger().info("Prueba de Encoders iniciada.")
        self.get_logger().info("El robot girará 90° a la Izquierda y luego regresará.")
        self.get_logger().info("Por favor, asegúrate de que tiene espacio libre.")
        self.get_logger().info("=========================================")

    def euler_from_quaternion(self, q):
        # Convertir cuaternión a Euler (Yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Mantener el ángulo entre -pi y pi
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg):
        self.current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        
        if self.state == 'INIT':
            self.start_yaw = self.current_yaw
            self.state = 'TURN_LEFT'
            self.get_logger().info(f"[Estado] Referencia inicial fijada: {math.degrees(self.start_yaw):.1f}°")
            self.get_logger().info("-> Rotando a la IZQUIERDA...")

    def control_loop(self):
        if self.state in ['INIT', 'PAUSE_1', 'DONE']:
            return

        msg = Twist()
        delta_yaw = self.normalize_angle(self.current_yaw - self.start_yaw)
        
        if self.state == 'TURN_LEFT':
            if delta_yaw < self.target_angle_rad:
                msg.angular.z = 0.4  # Velocidad de giro rad/s
            else:
                msg.angular.z = 0.0
                self.state = 'PAUSE_1'
                self.get_logger().info(f"Freno aplicado. La odometría marca: {math.degrees(delta_yaw):.1f}°")
                self.get_logger().info("Pausa de 2 segundos para evaluar inercia...")
                self.timer_pause = self.create_timer(2.0, self.start_turn_right)
                
        elif self.state == 'TURN_RIGHT':
            if delta_yaw > 0.0:
                msg.angular.z = -0.4
            else:
                msg.angular.z = 0.0
                self.state = 'DONE'
                self.get_logger().info(f"Freno aplicado. La odometría marca: {math.degrees(delta_yaw):.1f}°")
                self.get_logger().info("=========================================")
                self.get_logger().info("Prueba finalizada.")
                self.get_logger().info("Pregunta clave: ¿El robot regresó físicamente a la posición inicial exacta?")
                self.get_logger().info("=========================================")
                
        self.publisher.publish(msg)

    def start_turn_right(self):
        self.timer_pause.cancel()
        self.state = 'TURN_RIGHT'
        self.get_logger().info("-> Rotando a la DERECHA para volver a 0°...")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderRotationTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Parada de emergencia si el usuario presiona Ctrl+C
        pub = node.create_publisher(Twist, '/cmd_vel', 10)
        pub.publish(Twist()) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
