
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        
        # Configuracion del puerto serie (Ajustar si tu Arduino usa ttyUSB0)
        self.puerto = '/dev/ttyACM0'
        self.baudrate = 9600
        
        try:
            self.arduino = serial.Serial(self.puerto, self.baudrate, timeout=1)
            time.sleep(2) # Esperar a que el Arduino reinicie tras la conexion
            self.get_logger().info(f'Conectado al Arduino en {self.puerto}')
        except Exception as e:
            self.get_logger().error(f'Error abriendo puerto serie: {e}')
            return

        # Suscribirse al topic donde la PC enviara los comandos
        self.subscription = self.create_subscription(
            String,
            '/comando_arduino',
            self.listener_callback,
            10)
        
        self.get_logger().info('Nodo iniciado. Escuchando en el topic /comando_arduino...')

    def listener_callback(self, msg):
        comando = msg.data
        self.get_logger().info(f'Comando recibido de la red: "{comando}"')
        
        # Enviar al Arduino con salto de linea
        try:
            self.arduino.write((comando + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Error enviando a Arduino: {e}')

def main(args=None):
    rclpy.init(args=args)
    nodo = MotorSerialNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
