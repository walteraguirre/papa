#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import PoseWithCovarianceStamped

# ============================================================
# PARÁMETROS MECÁNICOS DEL PAPA
# ============================================================

WHEEL_RADIUS = 0.08       # [m]
WHEEL_SEPARATION = 0.42   # [m]

# Velocidad lineal de UNA rueda cuando |PWM| = 100.
#
# ESTE VALOR ES PROVISORIO.
# Después lo podemos calibrar con el robot real.
MAX_WHEEL_SPEED = 0.50    # [m/s]

MAX_PWM = 100.0


# ============================================================
# FUNCIONES AUXILIARES
# ============================================================

def yaw_to_quaternion(yaw):

    q = Quaternion()

    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)

    return q


# ============================================================
# SIMULADOR
# ============================================================

class PapaSimulator(Node):

    def __init__(self):

        super().__init__('papa_simulator')

        # --------------------------------------------------------
        # SUSCRIPCIÓN A LOS COMANDOS DEL ROBOT
        # --------------------------------------------------------

        self.subscription = self.create_subscription(
            String,
            '/comando_arduino',
            self.comando_callback,
            10
        )

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        # --------------------------------------------------------
        # PUBLICADORES
        # --------------------------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # --------------------------------------------------------
        # TF
        # --------------------------------------------------------

        self.tf_broadcaster = TransformBroadcaster(self)

        # --------------------------------------------------------
        # ESTADO REAL SIMULADO DEL ROBOT
        # --------------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocidad lineal de cada rueda
        self.v_left = 0.0
        self.v_right = 0.0

        # Posición angular acumulada de ruedas
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # --------------------------------------------------------
        # CONTROL DE TIEMPO
        # --------------------------------------------------------

        self.last_time = self.get_clock().now()

        # 50 Hz
        self.timer_period = 0.02

        self.timer = self.create_timer(
            self.timer_period,
            self.update
        )

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            'PAPA SIMULATOR INICIADO'
        )

        self.get_logger().info(
            f'Wheel radius     = {WHEEL_RADIUS:.3f} m'
        )

        self.get_logger().info(
            f'Wheel separation = {WHEEL_SEPARATION:.3f} m'
        )

        self.get_logger().info(
            f'PWM 100          = {MAX_WHEEL_SPEED:.3f} m/s'
        )

        self.get_logger().info(
            'Escuchando /comando_arduino'
        )

        self.get_logger().info(
            'Publicando /odom + /joint_states + TF'
        )

        self.get_logger().info(
            '============================================'
        )

    def initial_pose_callback(self, msg):

        # Posición seleccionada en RViz
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Quaternion -> yaw
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.theta = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        # Frenar al reposicionar
        self.v_left = 0.0
        self.v_right = 0.0

        self.get_logger().info(
            f'Pose inicial cambiada: '
            f'x={self.x:.2f}, '
            f'y={self.y:.2f}, '
            f'yaw={self.theta:.2f} rad'
        )

    # ============================================================
    # COMANDOS
    # ============================================================

    def comando_callback(self, msg):

        """
        Espera mensajes:

        VEL,left,right

        Ejemplos:

        VEL,50,50
        VEL,-50,-50
        VEL,-60,60
        VEL,60,-60
        """

        try:

            partes = msg.data.strip().split(',')

            if len(partes) != 3:
                return

            if partes[0] != 'VEL':
                return

            pwm_left = float(partes[1])
            pwm_right = float(partes[2])

            # Limitar PWM
            pwm_left = max(
                -MAX_PWM,
                min(MAX_PWM, pwm_left)
            )

            pwm_right = max(
                -MAX_PWM,
                min(MAX_PWM, pwm_right)
            )

            # ----------------------------------------------------
            # PWM -> VELOCIDAD LINEAL DE RUEDA
            # ----------------------------------------------------

            self.v_left = (
                pwm_left / MAX_PWM
            ) * MAX_WHEEL_SPEED

            self.v_right = (
                pwm_right / MAX_PWM
            ) * MAX_WHEEL_SPEED

            self.get_logger().info(
                f'PWM L={pwm_left:6.1f} '
                f'R={pwm_right:6.1f}  ->  '
                f'vL={self.v_left:.3f} '
                f'vR={self.v_right:.3f} m/s'
            )

        except Exception as e:

            self.get_logger().warn(
                f'Comando inválido: {msg.data} ({e})'
            )

    # ============================================================
    # SIMULACIÓN
    # ============================================================

    def update(self):

        now = self.get_clock().now()

        dt = (
            now - self.last_time
        ).nanoseconds / 1e9

        self.last_time = now

        # Protección ante pausas extrañas
        if dt <= 0.0 or dt > 0.5:
            return

        # --------------------------------------------------------
        # CINEMÁTICA DIFERENCIAL
        # --------------------------------------------------------

        v = (
            self.v_right + self.v_left
        ) / 2.0

        omega = (
            self.v_right - self.v_left
        ) / WHEEL_SEPARATION

        # --------------------------------------------------------
        # INTEGRACIÓN DE POSE
        # --------------------------------------------------------

        self.x += (
            v
            * math.cos(self.theta)
            * dt
        )

        self.y += (
            v
            * math.sin(self.theta)
            * dt
        )

        self.theta += omega * dt

        # Mantener theta entre -pi y pi
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # --------------------------------------------------------
        # POSICIÓN ANGULAR DE LAS RUEDAS
        # --------------------------------------------------------

        omega_left_wheel = (
            self.v_left / WHEEL_RADIUS
        )

        omega_right_wheel = (
            self.v_right / WHEEL_RADIUS
        )

        self.left_wheel_pos += (
            omega_left_wheel * dt
        )

        self.right_wheel_pos += (
            omega_right_wheel * dt
        )

        # --------------------------------------------------------
        # PUBLICAR
        # --------------------------------------------------------

        self.publicar_odom(
            now,
            v,
            omega
        )

        self.publicar_tf(now)

        self.publicar_joint_states(
            now,
            omega_left_wheel,
            omega_right_wheel
        )

    # ============================================================
    # ODOMETRÍA
    # ============================================================

    def publicar_odom(self, now, v, omega):

        msg = Odometry()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'

        msg.child_frame_id = 'base_footprint'

        # Pose
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation = (
            yaw_to_quaternion(self.theta)
        )

        # Velocidades
        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = omega

        self.odom_pub.publish(msg)

    # ============================================================
    # TF ODOM -> BASE_FOOTPRINT
    # ============================================================

    def publicar_tf(self, now):

        t = TransformStamped()

        t.header.stamp = now.to_msg()

        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = yaw_to_quaternion(self.theta)

        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    # ============================================================
    # JOINT STATES
    # ============================================================

    def publicar_joint_states(
        self,
        now,
        omega_left,
        omega_right
    ):

        msg = JointState()

        msg.header.stamp = now.to_msg()

        # IMPORTANTE:
        # Cambiar estos nombres si tu URDF utiliza otros.
        msg.name = [
            'left_wheel_joint',
            'right_wheel_joint'
        ]

        msg.position = [
            self.left_wheel_pos,
            self.right_wheel_pos
        ]

        msg.velocity = [
            omega_left,
            omega_right
        ]

        self.joint_pub.publish(msg)


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = PapaSimulator()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()