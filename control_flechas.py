import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select
import time

FLECHA_ARRIBA = '\x1b[A'
FLECHA_ABAJO = '\x1b[B'
FLECHA_IZQUIERDA = '\x1b[D'
FLECHA_DERECHA = '\x1b[C'


class TeleopFlechas(Node):

    def __init__(self):
        super().__init__('teleop_flechas_directo')

        self.publisher = self.create_publisher(
            String,
            '/comando_arduino',
            10
        )

        # ============================================================
        # PARÁMETROS DE MOVIMIENTO
        # ============================================================

        self.max_speed = 100
        self.step = 1
        self.min_pwm = 45

        # ============================================================
        # VARIABLES DE ESTADO
        # ============================================================

        self.target_l = 0
        self.target_r = 0

        self.curr_l = 0
        self.curr_r = 0

        self.ultimo_tiempo_tecla = time.time()

        # Indica si el teleop está actualmente controlando el robot
        self.teleop_activo = False

        self.get_logger().info(
            'Control progresivo por flechas INICIADO.'
        )

        self.get_logger().info(
            'MANTÉN PRESIONADA una flecha para controlar el robot.'
        )

        self.get_logger().info(
            'Al soltar se envía VEL,0,0 UNA SOLA VEZ.'
        )

        self.get_logger().info(
            'Luego el nodo queda en silencio.'
        )

        self.get_logger().info(
            'Presiona CTRL+C para salir.'
        )

    # ============================================================
    # LECTURA DEL TECLADO
    # ============================================================

    def leer_tecla(self, timeout=0.1):

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        ch = None

        try:

            tty.setraw(sys.stdin.fileno())

            r, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                timeout
            )

            if r:

                ch = sys.stdin.read(1)

                if ch == '\x1b':

                    r2, _, _ = select.select(
                        [sys.stdin],
                        [],
                        [],
                        0.05
                    )

                    if r2:
                        ch += sys.stdin.read(2)

        finally:

            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old_settings
            )

        return ch

    # ============================================================
    # RAMPA DE VELOCIDAD
    # ============================================================

    def acercar_velocidad(self, actual, objetivo):

        # Salto inicial para superar la banda muerta
        if actual == 0 and objetivo > 0:
            return self.min_pwm

        if actual == 0 and objetivo < 0:
            return -self.min_pwm

        # Aceleración / desaceleración
        if actual < objetivo:
            return min(
                actual + self.step,
                objetivo
            )

        elif actual > objetivo:
            return max(
                actual - self.step,
                objetivo
            )

        return actual

    # ============================================================
    # PUBLICACIÓN
    # ============================================================

    def enviar_velocidades(self):

        comando = f"VEL,{self.curr_l},{self.curr_r}"

        msg = String()
        msg.data = comando

        self.publisher.publish(msg)

        print(
            f'\rComando publicado -> {comando}          ',
            end='',
            flush=True
        )

    # ============================================================
    # LOOP PRINCIPAL
    # ============================================================

    def ejecutar(self):

        while rclpy.ok():

            tecla = self.leer_tecla(timeout=0.1)

            tiempo_actual = time.time()

            # ----------------------------------------------------
            # CTRL+C
            # ----------------------------------------------------

            if tecla == '\x03':

                self.curr_l = 0
                self.curr_r = 0

                self.enviar_velocidades()

                break

            # ----------------------------------------------------
            # DETECCIÓN DE FLECHAS
            # ----------------------------------------------------

            tecla_movimiento = False

            if tecla == FLECHA_ARRIBA:

                self.target_l = self.max_speed
                self.target_r = self.max_speed

                tecla_movimiento = True

            elif tecla == FLECHA_ABAJO:

                self.target_l = -self.max_speed
                self.target_r = -self.max_speed

                tecla_movimiento = True

            elif tecla == FLECHA_IZQUIERDA:

                self.target_l = -self.max_speed
                self.target_r = self.max_speed

                tecla_movimiento = True

            elif tecla == FLECHA_DERECHA:

                self.target_l = self.max_speed
                self.target_r = -self.max_speed

                tecla_movimiento = True

            # ----------------------------------------------------
            # SI SE PRESIONÓ UNA FLECHA
            # ----------------------------------------------------

            if tecla_movimiento:

                self.ultimo_tiempo_tecla = tiempo_actual

                self.teleop_activo = True

                # Rampa de velocidad
                self.curr_l = self.acercar_velocidad(
                    self.curr_l,
                    self.target_l
                )

                self.curr_r = self.acercar_velocidad(
                    self.curr_r,
                    self.target_r
                )

                self.enviar_velocidades()

            # ----------------------------------------------------
            # DETECTAR QUE SE SOLTÓ LA FLECHA
            # ----------------------------------------------------

            elif (
                self.teleop_activo
                and
                tiempo_actual - self.ultimo_tiempo_tecla > 0.2
            ):

                # Frenado inmediato
                self.target_l = 0
                self.target_r = 0

                self.curr_l = 0
                self.curr_r = 0

                # IMPORTANTE:
                # se publica UNA SOLA VEZ
                self.enviar_velocidades()

                # A partir de acá queda en silencio
                self.teleop_activo = False

                print(
                    '\nTeleop inactivo -> esperando teclado...',
                    flush=True
                )

            # ----------------------------------------------------
            # SI NO HAY TELEOP ACTIVO
            # ----------------------------------------------------
            #
            # NO PUBLICAMOS NADA.
            #
            # Nav2 / navegación autónoma puede controlar el robot.
            # ----------------------------------------------------


def main(args=None):

    rclpy.init(args=args)

    nodo = TeleopFlechas()

    try:

        nodo.ejecutar()

    except KeyboardInterrupt:

        nodo.curr_l = 0
        nodo.curr_r = 0

        nodo.enviar_velocidades()

    finally:

        print("\nApagando control...")

        nodo.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
