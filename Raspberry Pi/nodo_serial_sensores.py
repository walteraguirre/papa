import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import serial
import threading
import math
import time

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class RobotOdometryNode(Node):
    def __init__(self):
        super().__init__('nodo_serial_sensores')
        
        # Parámetros mecánicos
        self.TICKS_PER_REV = 36.0
        self.WHEEL_RADIUS = 0.08
        self.WHEEL_SEPARATION = 0.42
        self.UMBRAL_OBSTACULO_CM = 30.0
        
        # Publicadores ROS 2
        self.pub_obs_l = self.create_publisher(Bool, 'robot/obstaculo/izquierdo', 10)
        self.pub_obs_c = self.create_publisher(Bool, 'robot/obstaculo/centro', 10)
        self.pub_obs_r = self.create_publisher(Bool, 'robot/obstaculo/derecho', 10)
        
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        
        # Publicador del Escáner Virtual (Ultrasonidos)
        self.pub_sonar_scan = self.create_publisher(LaserScan, 'sonar_scan', 10) 
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_enc1 = 0
        self.last_enc2 = 0
        self.last_time = time.monotonic()
        
        # Calibración del Giroscopio
        self.calibrating = True
        self.gyro_samples = []
        self.gyro_bias = 0.0
        self.get_logger().info("Calibrando MPU6050... NO MUEVA EL ROBOT.")
        
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        except Exception as e:
            self.get_logger().error(f"Falla crítica de hardware: {e}")
            return

        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()

    def read_serial_data(self):
        GYRO_DEADBAND_RAD = 0.026
        
        while rclpy.ok():
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    data = line.split(',')
                    if len(data) == 8:
                        current_time = time.monotonic()
                        dt = current_time - self.last_time
                        self.last_time = current_time
                        
                        distL, distC, distR = float(data[0]), float(data[1]), float(data[2])
                        gyroZ_raw = float(data[5])
                        enc1, enc2 = int(data[6]), int(data[7])
                        
                        if self.calibrating:
                            self.gyro_samples.append(gyroZ_raw)
                            if len(self.gyro_samples) >= 100:
                                self.gyro_bias = sum(self.gyro_samples) / len(self.gyro_samples)
                                self.last_enc1, self.last_enc2 = enc1, enc2
                                self.last_time = time.monotonic()
                                self.calibrating = False
                                self.get_logger().info(f"Calibración finalizada. Offset: {self.gyro_bias:.2f} °/s")
                            continue
                                                
                        # Cinemática y Filtros
                        gyroZ = gyroZ_raw - self.gyro_bias
                        gyro_z_rad = math.radians(gyroZ)
                        if abs(gyro_z_rad) < GYRO_DEADBAND_RAD: 
                            gyro_z_rad = 0.0
                            
                        d_enc_left = enc1 - self.last_enc1
                        d_enc_right = enc2 - self.last_enc2
                        self.last_enc1, self.last_enc2 = enc1, enc2

                        dtheta_left = (2.0 * math.pi / self.TICKS_PER_REV) * d_enc_left
                        dtheta_right = (2.0 * math.pi / self.TICKS_PER_REV) * d_enc_right
                        ds_left = self.WHEEL_RADIUS * dtheta_left
                        ds_right = self.WHEEL_RADIUS * dtheta_right

                        ds = (ds_right + ds_left) / 2.0
                        dtheta_robot = (ds_right - ds_left) / self.WHEEL_SEPARATION
                        theta_mid = self.yaw + dtheta_robot / 2.0

                        self.x += ds * math.cos(theta_mid)
                        self.y += ds * math.sin(theta_mid)
                        self.yaw += dtheta_robot
                        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

                        if dt > 0.0:
                            v_left, v_right = ds_left / dt, ds_right / dt
                            v_x = (v_right + v_left) / 2.0
                            v_theta = (v_right - v_left) / self.WHEEL_SEPARATION
                        else:
                            v_x, v_theta = 0.0, 0.0
                        
                        ros_time = self.get_clock().now().to_msg()
                        
                        # TF base_footprint
                        t = TransformStamped()
                        t.header.stamp, t.header.frame_id, t.child_frame_id = ros_time, 'odom', 'base_footprint'
                        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.x, self.y, 0.0
                        t.transform.rotation = yaw_to_quaternion(self.yaw)
                        self.tf_broadcaster.sendTransform(t)
                        
                        # Joint States
                        angle_left = (enc1 % self.TICKS_PER_REV) / self.TICKS_PER_REV * (2 * math.pi)
                        angle_right = (enc2 % self.TICKS_PER_REV) / self.TICKS_PER_REV * (2 * math.pi)
                        joint_msg = JointState()
                        joint_msg.header.stamp = ros_time
                        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint'] 
                        joint_msg.position = [angle_left, angle_right]
                        self.pub_joints.publish(joint_msg)
                        
                        # Odometría
                        odom = Odometry()
                        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = ros_time, 'odom', 'base_footprint'
                        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
                        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
                        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v_x, v_theta
                        self.pub_odom.publish(odom)

                        # =========================================================
                        # CONFIGURACIÓN DEL LÁSER VIRTUAL DE ALTA DENSIDAD
                        # =========================================================
                        RADIO_ROBOT = 0.225
                        
                        # Filtro de ruido: Ignorar lecturas de 0 o rebotes fantasma menores a 2 cm
                        valid_L = 2.0 < distL < self.UMBRAL_OBSTACULO_CM
                        valid_C = 2.0 < distC < self.UMBRAL_OBSTACULO_CM
                        valid_R = 2.0 < distR < self.UMBRAL_OBSTACULO_CM

                        # Cálculo de distancia incluyendo el radio del robot
                        r_izquierda = (distL / 100.0) + RADIO_ROBOT if valid_L else float('inf')
                        r_centro    = (distC / 100.0) + RADIO_ROBOT if valid_C else float('inf')
                        r_derecha   = (distR / 100.0) + RADIO_ROBOT if valid_R else float('inf')

                        sonar_scan = LaserScan()
                        sonar_scan.header.stamp = ros_time
                        sonar_scan.header.frame_id = 'base_link'
                        
                        # Cono de ~70 grados subdividido en 45 rayos (15 por sensor)
                        sonar_scan.angle_min = -0.61  # -35 grados (Derecha)
                        sonar_scan.angle_max = 0.61   # +35 grados (Izquierda)
                        sonar_scan.angle_increment = (0.61 - (-0.61)) / 44.0 
                        
                        sonar_scan.range_min = 0.02
                        sonar_scan.range_max = 2.0  # Ampliado para admitir el radio del robot + distancia

                        # Rellenar arreglo: Derecha a Izquierda
                        sonar_scan.ranges = [r_derecha]*15 + [r_centro]*15 + [r_izquierda]*15
                        self.pub_sonar_scan.publish(sonar_scan)
                        
                        # =========================================================
                        # PUBLICADORES BOOLEANOS
                        # =========================================================
                        self.pub_obs_l.publish(Bool(data=valid_L))
                        self.pub_obs_c.publish(Bool(data=valid_C))
                        self.pub_obs_r.publish(Bool(data=valid_R))
                        
            except Exception as e:
                self.get_logger().error(f"Excepción en el procesamiento: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
