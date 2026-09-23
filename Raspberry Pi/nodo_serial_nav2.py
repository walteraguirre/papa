#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time
import glob
import sys

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        self.baudrate = 115200
        self.arduino = None
        self.conectar_arduino()

        self.WHEEL_SEPARATION = 0.42
        self.MAX_WHEEL_SPEED = 0.30
        self.MAX_PWM = 180
        self.MIN_PWM = 45 

        self.subscription_manual = self.create_subscription(
            String, '/comando_arduino', self.manual_callback, 10)

        self.subscription_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('Nodo iniciado. Escuchando /comando_arduino y /cmd_vel')

    def conectar_arduino(self):
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.close()
            except Exception:
                pass
        
        intentos = 0
        # Intentará buscar el puerto durante 5 segundos antes de rendirse
        while intentos < 5:
            puertos = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            for p in puertos:
                try:
                    self.arduino = serial.Serial(p, self.baudrate, timeout=1)
                    time.sleep(2) # Tiempo crítico para que el Arduino inicie
                    self.get_logger().info(f'Conectado al Arduino en {p}')
                    return
                except Exception:
                    pass
            
            self.get_logger().warning('Buscando puerto del Arduino...')
            time.sleep(1.0)
            intentos += 1
            
        self.get_logger().error('Hardware no encontrado. Abortando proceso para delegar a Systemd.')
        sys.exit(1) # Muerte súbita. Systemd tomará el control aquí.

    def manual_callback(self, msg):
        comando = msg.data
        self.enviar_serial(comando)

    def aplicar_inercia(self, pwm_bruto):
        if pwm_bruto == 0:
            return 0
            
        signo = 1 if pwm_bruto > 0 else -1
        pwm_abs = abs(pwm_bruto)
        
        if 0 < pwm_abs < self.MIN_PWM:
            pwm_abs = self.MIN_PWM
            
        return int(pwm_abs * signo)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        L = self.WHEEL_SEPARATION

        v_left = v - (w * L / 2.0)
        v_right = v + (w * L / 2.0)

        max_abs = max(abs(v_left), abs(v_right))
        if max_abs > self.MAX_WHEEL_SPEED:
            escala = self.MAX_WHEEL_SPEED / max_abs
            v_left *= escala
            v_right *= escala

        pwm_left = int((v_left / self.MAX_WHEEL_SPEED) * self.MAX_PWM)
        pwm_right = int((v_right / self.MAX_WHEEL_SPEED) * self.MAX_PWM)

        pwm_left = max(-self.MAX_PWM, min(self.MAX_PWM, pwm_left))
        pwm_right = max(-self.MAX_PWM, min(self.MAX_PWM, pwm_right))

        pwm_left = self.aplicar_inercia(pwm_left)
        pwm_right = self.aplicar_inercia(pwm_right)

        comando = f"VEL,{pwm_left},{pwm_right}"
        self.enviar_serial(comando)

    def enviar_serial(self, comando):
        if self.arduino is None or not self.arduino.is_open:
            self.get_logger().error('Conexión perdida. Reiniciando nodo...')
            sys.exit(1)

        try:
            self.arduino.write((comando + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Fallo de escritura en USB: {e}. Forzando reinicio del nodo...')
            sys.exit(1)

    def destroy_node(self):
        if self.arduino is not None and self.arduino.is_open:
            try:
                self.arduino.write(b'VEL,0,0\n')
                time.sleep(0.1)
                self.arduino.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo = MotorSerialNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
