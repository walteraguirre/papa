import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import serial
import time


class MotorSerialNode(Node):

    def __init__(self):
        super().__init__('motor_serial_node')

        # ============================================================
        # SERIAL
        # ============================================================

        self.puerto = '/dev/ttyACM0'
        self.baudrate = 115200

        try:
            self.arduino = serial.Serial(
                self.puerto,
                self.baudrate,
                timeout=1
            )

            # Espera a que Arduino reinicie
            time.sleep(2)

            self.get_logger().info(
                f'Conectado al Arduino en {self.puerto} a {self.baudrate} baud'
            )

        except Exception as e:
            self.get_logger().error(
                f'Error abriendo puerto serie: {e}'
            )
            self.arduino = None


        # ============================================================
        # PARÁMETROS DEL ROBOT
        # ============================================================

        # Separación entre ruedas [m]
        self.WHEEL_SEPARATION = 0.42

        # Aproximación inicial:
        # velocidad lineal de rueda asociada al PWM máximo configurado
        self.MAX_WHEEL_SPEED = 0.30   # [m/s]

        # PWM máximo que queremos enviar al Arduino
        self.MAX_PWM = 180


        # ============================================================
        # TOPIC MANUAL
        # ============================================================

        self.subscription_manual = self.create_subscription(
            String,
            '/comando_arduino',
            self.manual_callback,
            10
        )


        # ============================================================
        # TOPIC NAV2
        # ============================================================

        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )


        self.get_logger().info(
            'Nodo iniciado.'
        )

        self.get_logger().info(
            'Escuchando /comando_arduino y /cmd_vel'
        )


    # ================================================================
    # CONTROL MANUAL
    # ================================================================

    def manual_callback(self, msg):

        comando = msg.data

        self.get_logger().info(
            f'Manual -> "{comando}"'
        )

        self.enviar_serial(comando)


    # ================================================================
    # CONTROL NAV2
    # ================================================================

    def cmd_vel_callback(self, msg):

        # Velocidad lineal del robot
        v = msg.linear.x

        # Velocidad angular del robot
        w = msg.angular.z

        L = self.WHEEL_SEPARATION


        # ============================================================
        # CINEMÁTICA DIFERENCIAL
        # ============================================================

        v_left = v - (w * L / 2.0)
        v_right = v + (w * L / 2.0)


        # ============================================================
        # LIMITAR VELOCIDADES
        # ============================================================

        max_abs = max(
            abs(v_left),
            abs(v_right)
        )

        if max_abs > self.MAX_WHEEL_SPEED:

            escala = self.MAX_WHEEL_SPEED / max_abs

            v_left *= escala
            v_right *= escala


        # ============================================================
        # CONVERTIR m/s -> PWM
        # ============================================================

        pwm_left = int(
            (v_left / self.MAX_WHEEL_SPEED)
            * self.MAX_PWM
        )

        pwm_right = int(
            (v_right / self.MAX_WHEEL_SPEED)
            * self.MAX_PWM
        )


        # Seguridad extra
        pwm_left = max(
            -self.MAX_PWM,
            min(self.MAX_PWM, pwm_left)
        )

        pwm_right = max(
            -self.MAX_PWM,
            min(self.MAX_PWM, pwm_right)
        )


        # ============================================================
        # ARMAR COMANDO SERIAL
        # ============================================================

        comando = f"VEL,{pwm_left},{pwm_right}"


        self.get_logger().info(
            f'cmd_vel: '
            f'v={v:.3f} m/s | '
            f'w={w:.3f} rad/s -> '
            f'L={pwm_left} | '
            f'R={pwm_right}'
        )


        self.enviar_serial(comando)


    # ================================================================
    # ENVIAR AL ARDUINO
    # ================================================================

    def enviar_serial(self, comando):

        if self.arduino is None:
            self.get_logger().error(
                'Arduino no conectado'
            )
            return

        try:

            self.arduino.write(
                (comando + '\n').encode('utf-8')
            )

        except Exception as e:

            self.get_logger().error(
                f'Error enviando al Arduino: {e}'
            )


    # ================================================================
    # CERRAR SERIAL
    # ================================================================

    def destroy_node(self):

        if self.arduino is not None:

            try:
                self.arduino.write(
                    b'VEL,0,0\n'
                )

                time.sleep(0.1)

                self.arduino.close()

            except Exception:
                pass

        super().destroy_node()


# ====================================================================
# MAIN
# ====================================================================

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
