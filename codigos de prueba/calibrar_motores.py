#!/usr/bin/env python3
# antes de ejecutar dar permisos chmod +x calibrador_motores.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class MotorCalibrator(Node):
    def __init__(self):
        super().__init__('calibrador_motores')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Parámetros físicos
        self.WHEEL_SEPARATION = 0.42
        self.CURRENT_FACTOR = 1.2000  # Tu factor actual en el Arduino
        
        self.start_pose = None
        self.get_logger().info("=========================================")
        self.get_logger().info("Calibrador iniciado.")
        self.get_logger().info("Avanza el robot 2 metros en línea recta.")
        self.get_logger().info("=========================================")

    def euler_from_quaternion(self, q):
        # Conversión manual para evitar problemas de dependencias
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        if self.start_pose is None:
            self.start_pose = (current_x, current_y, current_yaw)
            return

        dx = current_x - self.start_pose[0]
        dy = current_y - self.start_pose[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # Ocultar cálculos hasta que se mueva al menos 10 cm
        if distance < 0.1:
            return

        delta_yaw = current_yaw - self.start_pose[2]
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw)) # Normalizar

        # Cinemática inversa: Cuánto avanzó físicamente cada rueda
        s_r = distance + (delta_yaw * self.WHEEL_SEPARATION) / 2.0
        s_l = distance - (delta_yaw * self.WHEEL_SEPARATION) / 2.0

        if s_r != 0:
            # Multiplicador ideal para compensar el desbalance
            correction_ratio = s_l / s_r
            new_factor = self.CURRENT_FACTOR * correction_ratio

            self.get_logger().info(f"Recorrido: {distance:.2f}m | Desvío: {math.degrees(delta_yaw):.1f}°", throttle_duration_sec=0.5)
            self.get_logger().info(f"Arcos -> Izq: {s_l:.3f}m | Der: {s_r:.3f}m", throttle_duration_sec=0.5)
            self.get_logger().info(f"NUEVO FACTOR PARA ARDUINO: {new_factor:.4f}\n", throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    node = MotorCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
