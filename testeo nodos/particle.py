#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)


class ParticleCloudToPoseArray(Node):

    def __init__(self):
        super().__init__('particle_cloud_to_pose_array')

        # QoS compatible con /particle_cloud
        particle_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_callback,
            particle_qos
        )

        self.publisher = self.create_publisher(
            PoseArray,
            '/particle_cloud_rviz',
            10
        )

        self.get_logger().info(
            'Convirtiendo /particle_cloud -> /particle_cloud_rviz'
        )

    def particle_callback(self, msg):

        pose_array = PoseArray()
        pose_array.header = msg.header

        for particle in msg.particles:
            pose_array.poses.append(particle.pose)

        self.publisher.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)

    node = ParticleCloudToPoseArray()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
