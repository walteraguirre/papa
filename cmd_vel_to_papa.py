#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ============================================================
# PARÁMETROS DEL PAPA
# ============================================================

WHEEL_SEPARATION = 0.42       # [m]

# Tiene que coincidir con papa_simulator.py
MAX_WHEEL_SPEED = 0.50        # [m/s]

MAX_PWM = 100.0


class CmdVelToPapa(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_papa')

        # --------------------------------------------------------
        # Suscripción a Nav2
        # --------------------------------------------------------

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # --------------------------------------------------------
        # Comando que entiende PAPA
        # --------------------------------------------------------

        self.command_pub = self.create_publisher(
            String,
            '/comando_arduino',
            10
        )

        self.get_logger().info(
            '============================================'
        )
        self.get_logger().info(
            'CMD_VEL -> PAPA iniciado'
        )
        self.get_logger().info(
            f'Wheel separation = {WHEEL_SEPARATION:.3f} m'
        )
        self.get_logger().info(
            f'Max wheel speed  = {MAX_WHEEL_SPEED:.3f} m/s'
        )
        self.get_logger().info(
            'Escuchando /cmd_vel'
        )
        self.get_logger().info(
            'Publicando /comando_arduino'
        )
        self.get_logger().info(
            '============================================'
        )

    def cmd_vel_callback(self, msg):

        # ========================================================
        # VELOCIDAD DEL ROBOT
        # ========================================================

        v = msg.linear.x
        omega = msg.angular.z

        # ========================================================
        # CINEMÁTICA DIFERENCIAL
        # ========================================================

        v_left = (
            v
            - omega * WHEEL_SEPARATION / 2.0
        )

        v_right = (
            v
            + omega * WHEEL_SEPARATION / 2.0
        )

        # ========================================================
        # SATURACIÓN
        #
        # Conservamos la relación entre las ruedas.
        # ========================================================

        max_speed = max(
            abs(v_left),
            abs(v_right)
        )

        if max_speed > MAX_WHEEL_SPEED:

            scale = MAX_WHEEL_SPEED / max_speed

            v_left *= scale
            v_right *= scale

        # ========================================================
        # VELOCIDAD -> PWM
        # ========================================================

        pwm_left = (
            v_left
            / MAX_WHEEL_SPEED
            * MAX_PWM
        )

        pwm_right = (
            v_right
            / MAX_WHEEL_SPEED
            * MAX_PWM
        )

        # ========================================================
        # COMANDO
        # ========================================================

        command = String()

        command.data = (
            f'VEL,'
            f'{pwm_left:.2f},'
            f'{pwm_right:.2f}'
        )

        self.command_pub.publish(command)

        self.get_logger().info(
            f'v={v:+.3f} m/s  '
            f'w={omega:+.3f} rad/s  ->  '
            f'L={pwm_left:+.1f}  '
            f'R={pwm_right:+.1f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CmdVelToPapa()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Frenar al salir
        stop = String()
        stop.data = 'VEL,0,0'

        node.command_pub.publish(stop)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()

