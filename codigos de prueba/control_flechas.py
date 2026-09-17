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
        self.publisher = self.create_publisher(String, '/comando_arduino', 10)
        
        # Parámetros de movimiento (Aumentamos max_speed para tener mayor rango útil)
        self.max_speed = 80
        self.step = 2  # Tasa de aceleración por ciclo
        self.min_pwm = 45 # Salto inicial para vencer la inercia estática (Deadband)
        
        # Variables de estado
        self.target_l = 0
        self.target_r = 0
        self.curr_l = 0
        self.curr_r = 0
        self.ultimo_tiempo_tecla = time.time()
        
        self.get_logger().info('Control progresivo por flechas INICIADO.')
        self.get_logger().info('MANTÉN PRESIONADA una flecha para acelerar. SUELTA para frenar.')
        self.get_logger().info('Presiona CTRL+C para salir.')

    def leer_tecla(self, timeout=0.1):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        ch = None
        try:
            tty.setraw(sys.stdin.fileno())
            r, w, e = select.select([sys.stdin], [], [], timeout)
            if r:
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r2:
                        ch += sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def acercar_velocidad(self, actual, objetivo):
        # 1. Salto instantáneo de la banda muerta al arrancar
        if actual == 0 and objetivo > 0:
            return self.min_pwm
        if actual == 0 and objetivo < 0:
            return -self.min_pwm
            
        # 2. Frenado instantáneo si suelta el acelerador dentro de la banda muerta
        if 0 < actual < self.min_pwm and objetivo == 0:
            return 0
        if 0 > actual > -self.min_pwm and objetivo == 0:
            return 0

        # 3. Aceleración/Deceleración progresiva normal
        if actual < objetivo:
            return min(actual + self.step, objetivo)
        elif actual > objetivo:
            return max(actual - self.step, objetivo)
            
        return actual

    def enviar_velocidades(self):
        comando = f"VEL,{self.curr_l},{self.curr_r}"
        msg = String()
        msg.data = comando
        self.publisher.publish(msg)
        print(f'\rComando publicado -> {comando}                    ', end='', flush=True)

    def ejecutar(self):
        while rclpy.ok():
            tecla = self.leer_tecla(timeout=0.1)
            tiempo_actual = time.time()
            
            if tecla == '\x03':
                self.curr_l, self.curr_r = 0, 0
                self.enviar_velocidades()
                break

            # Asignar velocidad objetivo según la flecha
            if tecla == FLECHA_ARRIBA:
                self.target_l, self.target_r = self.max_speed, self.max_speed
                self.ultimo_tiempo_tecla = tiempo_actual
            elif tecla == FLECHA_ABAJO:
                self.target_l, self.target_r = -self.max_speed, -self.max_speed
                self.ultimo_tiempo_tecla = tiempo_actual
            elif tecla == FLECHA_IZQUIERDA:
                self.target_l, self.target_r = -self.max_speed, self.max_speed
                self.ultimo_tiempo_tecla = tiempo_actual
            elif tecla == FLECHA_DERECHA:
                self.target_l, self.target_r = self.max_speed, -self.max_speed
                self.ultimo_tiempo_tecla = tiempo_actual

            # Lógica de timeout y aceleración
            if tiempo_actual - self.ultimo_tiempo_tecla > 0.2:
                # Frenado instantáneo
                self.target_l, self.target_r = 0, 0
                self.curr_l, self.curr_r = 0, 0
            else:
                # Aceleración progresiva usando la nueva lógica con min_pwm
                self.curr_l = self.acercar_velocidad(self.curr_l, self.target_l)
                self.curr_r = self.acercar_velocidad(self.curr_r, self.target_r)

            self.enviar_velocidades()

def main(args=None):
    rclpy.init(args=args)
    nodo = TeleopFlechas()
    try:
        nodo.ejecutar()
    except Exception:
        pass
    finally:
        print("\n\rApagando control...")
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
