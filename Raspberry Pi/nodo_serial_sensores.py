import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import serial
import threading
import math
import time

def yaw_to_quaternion(yaw):
    # Importación corregida para evitar errores silenciosos en la red TF
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class RobotOdometryNode(Node):
    def __init__(self):
        super().__init__('nodo_serial_sensores')
        
        # Parámetros mecánicos (Ajustar constantes)
        self.TICKS_PER_METER = 1000.0  
        self.TICKS_PER_REV = 360.0 
        self.UMBRAL_OBSTACULO_CM = 30.0 
        
        # Publicadores ROS 2
        self.pub_obstaculo = self.create_publisher(Bool, 'robot/obstaculo_frontal', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Integración de variables de estado
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_enc1 = 0
        self.last_enc2 = 0
        self.last_time = time.time()
        
        # Proceso de calibración estocástica inicial
        self.calibrating = True
        self.gyro_samples = []
        self.gyro_bias = 0.0
        self.get_logger().info("Calibrando MPU6050... NO MUEVA EL ROBOT.")
        
        try:
            # Apuntar a /dev/ttyUSB0 (o /dev/arduino_base si usaste reglas Udev)
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        except Exception as e:
            self.get_logger().error(f"Falla crítica de hardware: {e}")
            return

        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()

    def read_serial_data(self):
        GYRO_DEADBAND_RAD = 0.026 # Zona muerta (~1.5 °/s)
        
        while rclpy.ok():
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    data = line.split(',')
                    if len(data) == 8:
                        current_time = time.time()
                        dt = current_time - self.last_time
                        self.last_time = current_time
                        
                        distL, distC, distR = float(data[0]), float(data[1]), float(data[2])
                        gyroZ_raw = float(data[5])
                        enc1, enc2 = int(data[6]), int(data[7])
                        
                        # --- 1. REMOCIÓN DEL SESGO (BIAS) ---
                        if self.calibrating:
                            self.gyro_samples.append(gyroZ_raw)
                            if len(self.gyro_samples) >= 100:
                                self.gyro_bias = sum(self.gyro_samples) / len(self.gyro_samples)
                                self.calibrating = False
                                self.get_logger().info(f"Calibración finalizada. Offset: {self.gyro_bias:.2f} °/s")
                            continue 
                        
                        # --- 2. FILTRO DE SENSIBILIDAD ---
                        gyroZ = gyroZ_raw - self.gyro_bias
                        gyro_z_rad = math.radians(gyroZ)
                        
                        if abs(gyro_z_rad) < GYRO_DEADBAND_RAD:
                            gyro_z_rad = 0.0
                            
                        self.yaw += gyro_z_rad * dt
                        
                        # --- 3. ODOMETRÍA LINEAL ---
                        d_enc1 = enc1 - self.last_enc1
                        d_enc2 = enc2 - self.last_enc2
                        self.last_enc1, self.last_enc2 = enc1, enc2
                        
                        d_distance = ((d_enc1 + d_enc2) / 2.0) / self.TICKS_PER_METER
                        self.x += d_distance * math.cos(self.yaw)
                        self.y += d_distance * math.sin(self.yaw)
                        
                        v_x = d_distance / dt if dt > 0 else 0.0
                        v_theta = gyro_z_rad
                        
                        ros_time = self.get_clock().now().to_msg()
                        
                        # --- 4. CONEXIÓN DEL ÁRBOL TF ---
                        t = TransformStamped()
                        t.header.stamp = ros_time
                        t.header.frame_id = 'odom'
                        t.child_frame_id = 'base_footprint'
                        t.transform.translation.x = self.x
                        t.transform.translation.y = self.y
                        t.transform.translation.z = 0.0
                        t.transform.rotation = yaw_to_quaternion(self.yaw)
                        self.tf_broadcaster.sendTransform(t)
                        
                        # --- 5. ESTADO VISUAL DE LAS RUEDAS ---
                        angle_left = (enc1 % self.TICKS_PER_REV) / self.TICKS_PER_REV * (2 * math.pi)
                        angle_right = (enc2 % self.TICKS_PER_REV) / self.TICKS_PER_REV * (2 * math.pi)
                        
                        joint_msg = JointState()
                        joint_msg.header.stamp = ros_time
                        # IMPORTANTE: Reemplazar por los nombres de la etiqueta <joint> en tu URDF
                        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint'] 
                        joint_msg.position = [angle_left, angle_right]
                        self.pub_joints.publish(joint_msg)
                        
                        # --- 6. TÓPICO DE ODOMETRÍA ESTÁNDAR ---
                        odom = Odometry()
                        odom.header.stamp = ros_time
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_footprint'
                        odom.pose.pose.position.x = self.x
                        odom.pose.pose.position.y = self.y
                        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
                        odom.twist.twist.linear.x = v_x
                        odom.twist.twist.angular.z = v_theta
                        self.pub_odom.publish(odom)
                        
                        # --- 7. ULTRASONIDOS (Lógica de interrupción) ---
                        hay_obstaculo = (distL < self.UMBRAL_OBSTACULO_CM) or \
                                        (distC < self.UMBRAL_OBSTACULO_CM) or \
                                        (distR < self.UMBRAL_OBSTACULO_CM)
                        msg_obs = Bool()
                        msg_obs.data = hay_obstaculo
                        self.pub_obstaculo.publish(msg_obs)
                        
            except Exception as e:
                # El registro de errores evitará bloqueos silenciosos en la red TF
                self.get_logger().error(f"Excepción en el procesamiento: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
