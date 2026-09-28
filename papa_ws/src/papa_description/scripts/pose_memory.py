#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import os

class PoseMemoryNode(Node):
    def __init__(self):
        super().__init__('pose_memory_node')
        
        self.file_path = os.path.expanduser('~/papa/last_pose.json')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        self.latest_pose = None
        self.startup_timer = self.create_timer(4.0, self.load_and_publish_pose)
        self.save_timer = self.create_timer(5.0, self.save_pose)
        
        self.get_logger().info('Nodo de Memoria de Pose iniciado en la PC Cerebro.')

    def pose_callback(self, msg):
        self.latest_pose = msg

    def save_pose(self):
        if self.latest_pose is None:
            return
        
        p = self.latest_pose.pose.pose
        data = {
            'x': p.position.x,
            'y': p.position.y,
            'z': p.position.z,
            'qx': p.orientation.x,
            'qy': p.orientation.y,
            'qz': p.orientation.z,
            'qw': p.orientation.w
        }
        try:
            with open(self.file_path, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            self.get_logger().error(f'Error guardando pose: {e}')

    def load_and_publish_pose(self):
        self.startup_timer.cancel()
        
        if not os.path.exists(self.file_path):
            self.get_logger().warn('No hay pose guardada. Usa 2D Pose Estimate en RViz2 por única vez.')
            return
        
        try:
            with open(self.file_path, 'r') as f:
                data = json.load(f)
                
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.pose.pose.position.x = data['x']
            msg.pose.pose.position.y = data['y']
            msg.pose.pose.position.z = data['z']
            msg.pose.pose.orientation.x = data['qx']
            msg.pose.pose.orientation.y = data['qy']
            msg.pose.pose.orientation.z = data['qz']
            msg.pose.pose.orientation.w = data['qw']
            
            msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
                                   
            self.pose_pub.publish(msg)
            self.get_logger().info(f'¡Memoria recuperada! Robot en X={data["x"]:.2f}, Y={data["y"]:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error leyendo pose: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseMemoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#dar permisos de ejecución chmod +x ~/papa/papa_ws/src/papa_description/scripts/pose_memory.py
