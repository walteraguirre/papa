#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

import math
import time


class AutoLocalization(Node):

    def __init__(self):
        super().__init__('auto_localization')

        # ============================================================
        # PARÁMETROS
        # ============================================================

        # Umbrales obtenidos experimentalmente
        self.SIGMA_X_MAX = 0.25          # [m]
        self.SIGMA_Y_MAX = 0.35          # [m]
        self.SIGMA_YAW_MAX = 13.0        # [deg]

        # Tiempo que AMCL debe permanecer dentro de los umbrales
        self.STABLE_TIME = 5.0           # [s]

        # Velocidad angular durante la búsqueda
        self.ANGULAR_SPEED = 0.20        # [rad/s]

        # ============================================================
        # ESTADO
        # ============================================================

        self.localized = False
        self.stable_since = None

        self.last_sigma_x = None
        self.last_sigma_y = None
        self.last_sigma_yaw = None

        # ============================================================
        # SUBSCRIBER AMCL
        # ============================================================

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # ============================================================
        # PUBLISHER CMD_VEL
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Control a 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '========================================\n'
            ' AUTO LOCALIZATION - PAPA\n'
            '========================================\n'
            f' sigma_x   < {self.SIGMA_X_MAX:.2f} m\n'
            f' sigma_y   < {self.SIGMA_Y_MAX:.2f} m\n'
            f' sigma_yaw < {self.SIGMA_YAW_MAX:.1f} deg\n'
            f' estabilidad = {self.STABLE_TIME:.1f} s\n'
            '========================================'
        )


    # ================================================================
    # AMCL
    # ================================================================

    def amcl_callback(self, msg):

        cov = msg.pose.covariance

        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]

        self.last_sigma_x = math.sqrt(max(var_x, 0.0))
        self.last_sigma_y = math.sqrt(max(var_y, 0.0))
        self.last_sigma_yaw = math.degrees(
            math.sqrt(max(var_yaw, 0.0))
        )

        # ------------------------------------------------------------
        # ¿Estamos dentro de los umbrales?
        # ------------------------------------------------------------

        good = (
            self.last_sigma_x < self.SIGMA_X_MAX
            and
            self.last_sigma_y < self.SIGMA_Y_MAX
            and
            self.last_sigma_yaw < self.SIGMA_YAW_MAX
        )

        if good:

            if self.stable_since is None:

                self.stable_since = time.monotonic()

                self.get_logger().info(
                    'Covarianza dentro de los umbrales. '
                    'Verificando estabilidad...'
                )

            stable_time = time.monotonic() - self.stable_since

            # --------------------------------------------------------
            # LOCALIZACIÓN CONFIRMADA
            # --------------------------------------------------------

            if (
                stable_time >= self.STABLE_TIME
                and not self.localized
            ):

                self.localized = True

                self.stop_robot()

                self.get_logger().info(
                    '\n'
                    '========================================\n'
                    '       PAPA LOCALIZADO\n'
                    '========================================\n'
                    f'sigma_x   = {self.last_sigma_x:.3f} m\n'
                    f'sigma_y   = {self.last_sigma_y:.3f} m\n'
                    f'sigma_yaw = {self.last_sigma_yaw:.2f} deg\n'
                    '========================================'
                )

        else:

            # Si una variable vuelve a superar el umbral,
            # reiniciamos el contador.

            if self.stable_since is not None:

                self.get_logger().warn(
                    'La incertidumbre volvió a aumentar. '
                    'Reiniciando contador.'
                )

            self.stable_since = None


    # ================================================================
    # CONTROL
    # ================================================================

    def control_loop(self):

        # Todavía no recibimos AMCL
        if self.last_sigma_x is None:
            return

        # ------------------------------------------------------------
        # Si ya está localizado -> STOP
        # ------------------------------------------------------------

        if self.localized:

            self.stop_robot()
            return

        # ------------------------------------------------------------
        # Mostrar estado
        # ------------------------------------------------------------

        self.get_logger().info(
            f'σx={self.last_sigma_x:.3f} m | '
            f'σy={self.last_sigma_y:.3f} m | '
            f'σyaw={self.last_sigma_yaw:.2f} deg'
        )

        # ------------------------------------------------------------
        # Movimiento de búsqueda
        # ------------------------------------------------------------

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = self.ANGULAR_SPEED

        self.cmd_pub.publish(cmd)


    # ================================================================
    # STOP
    # ================================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = AutoLocalization()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Autolocalización cancelada.'
        )

        node.stop_robot()

    finally:

        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
