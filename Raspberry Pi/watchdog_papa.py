import rclpy
from rclpy.node import Node
import subprocess
import time
from std_msgs.msg import Bool

class RobotNetworkWatchdog(Node):
    def __init__(self):
        super().__init__('robot_network_watchdog')
        
        self.interface_name = 'wlxc03a55f123f0' # Asegurar que coincide con el 'ip a' de la Mini PC
        self.timeout_sec = 6.0
        
        # Suscriptor exclusivo al latido de la PC
        self.subscription = self.create_subscription(
            Bool, '/ping_red', self.listener_callback, 10)
        
        self.last_msg_time = time.time()
        self.check_timer = self.create_timer(1.0, self.check_connection)
        
        self.get_logger().info(f'Watchdog Robot iniciado en {self.interface_name}. Vigilando latido /ping_red')

    def listener_callback(self, msg):
        self.last_msg_time = time.time()

    def check_connection(self):
        if (time.time() - self.last_msg_time) > self.timeout_sec:
            self.get_logger().error('Latido de la computadora perdido. Reiniciando Wi-Fi en el robot...')
            
            subprocess.run(["sudo", "ip", "link", "set", self.interface_name, "down"])
            time.sleep(2)
            subprocess.run(["sudo", "ip", "link", "set", self.interface_name, "up"])
            
            self.get_logger().info('Re-autenticación completada. Dando tiempo de enlace...')
            self.last_msg_time = time.time() + 15.0 

def main(args=None):
    rclpy.init(args=args)
    node = RobotNetworkWatchdog()
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
