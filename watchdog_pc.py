import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class PCNetworkWatchdog(Node):
    def __init__(self):
        super().__init__('pc_network_watchdog')
        
        self.timeout_sec = 6.0
        
        # Publicador del latido hacia el robot a 1 Hz
        self.ping_pub = self.create_publisher(Bool, '/ping_red', 10)
        self.ping_timer = self.create_timer(1.0, self.publish_ping)
        
        # Suscriptor para monitorear pasivamente si el robot sigue enviando datos
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.listener_callback, 10)
        
        self.last_msg_time = time.time()
        self.check_timer = self.create_timer(1.0, self.check_connection)
        
        self.get_logger().info('Watchdog PC iniciado. Emitiendo /ping_red y vigilando /odom del robot.')

    def publish_ping(self):
        msg = Bool()
        msg.data = True
        self.ping_pub.publish(msg)

    def listener_callback(self, msg):
        self.last_msg_time = time.time()

    def check_connection(self):
        if (time.time() - self.last_msg_time) > self.timeout_sec:
            # Solo muestra una advertencia en pantalla, no reinicia la red de la PC
            self.get_logger().warn('Alerta: Datos del robot perdidos por roaming. Esperando a que el robot reinicie su propia red...')
            
            # Pausa de 10 segundos antes de volver a advertir para no saturar la terminal
            self.last_msg_time = time.time() + 10.0 

def main(args=None):
    rclpy.init(args=args)
    node = PCNetworkWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
