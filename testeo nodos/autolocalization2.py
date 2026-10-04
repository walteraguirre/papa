#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty

from tf2_ros import Buffer, TransformListener, TransformException


class AutoLocalizationV3(Node):

    def __init__(self):
        super().__init__('auto_localization_v3')

        # ============================================================
        # 1. UMBRALES AMCL
        # ============================================================

        self.SIGMA_X_MAX = 0.25
        self.SIGMA_Y_MAX = 0.35
        self.SIGMA_YAW_MAX = 13.0

        # ============================================================
        # 2. LIDAR <-> MAP
        # ============================================================

        self.OCCUPIED_THRESHOLD = 65

        self.MATCH_RADIUS = 0.15
        self.SCAN_STEP = 2

        self.MATCH_WINDOW = 40
        self.MATCH_AVG_MIN = 60.0

        # ============================================================
        # 3. CRITERIO TEMPORAL
        # ============================================================

        self.STABLE_TIME = 1.0

        # ============================================================
        # 4. MOVIMIENTO
        #
        # Estado 0 = avanzar
        # Estado 1 = girar
        # ============================================================

        self.LINEAR_SPEED = 0.10
        self.ANGULAR_SPEED = 0.40

        # Cuando encuentra obstáculo gira durante 5 segundos
        self.ROTATE_TIME = 1.5

        self.search_phase = 0
        self.phase_start = time.monotonic()

        # ============================================================
        # 5. SONAR
        # ============================================================

        self.SONAR_STOP_DISTANCE = 0.50
        self.SONAR_TIMEOUT = 2.0

        self.sonar_distance = float('inf')
        self.sonar_obstacle = False
        self.last_sonar_time = None

        # ============================================================
        # 6. TIMEOUT
        # ============================================================

        self.SEARCH_TIMEOUT = 120.0

        # ============================================================
        # 7. RECUPERACION GLOBAL V3
        # ============================================================

        self.BAD_HYPOTHESIS_TIME = 3.0
        self.GLOBAL_RESET_COOLDOWN = 15.0
        self.MAX_GLOBAL_RESETS = 3

        self.bad_hypothesis_since = None
        self.last_global_reset = None

        self.global_reset_count = 0
        self.global_reset_pending = False

        # Reset global obligatorio al iniciar
        self.initial_global_reset_done = False

        # ============================================================
        # 8. ESTADO
        # ============================================================

        self.map_msg = None

        self.sigma_x = None
        self.sigma_y = None
        self.sigma_yaw = None

        self.current_match = None

        self.match_history = deque(
            maxlen=self.MATCH_WINDOW
        )

        self.stable_since = None

        self.localized = False
        self.timed_out = False

        self.start_time = time.monotonic()
        self.last_log_time = 0.0

        # ============================================================
        # 9. TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # 10. QoS
        # ============================================================

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ============================================================
        # 11. SUBSCRIPCIONES
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )

        self.sonar_sub = self.create_subscription(
            LaserScan,
            '/sonar_scan',
            self.sonar_callback,
            sensor_qos
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # ============================================================
        # 12. CMD_VEL
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ============================================================
        # 13. GLOBAL LOCALIZATION
        # ============================================================

        self.global_localization_client = self.create_client(
            Empty,
            '/reinitialize_global_localization'
        )

        # ============================================================
        # 14. LOOP
        # ============================================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '\n'
            '========================================\n'
            ' AUTO LOCALIZATION V3 - PAPA\n'
            '========================================\n'
            f' sigma_x   < {self.SIGMA_X_MAX:.2f} m\n'
            f' sigma_y   < {self.SIGMA_Y_MAX:.2f} m\n'
            f' sigma_yaw < {self.SIGMA_YAW_MAX:.1f} deg\n'
            '----------------------------------------\n'
            f' MATCH AVG >= {self.MATCH_AVG_MIN:.1f} %\n'
            f' ventana MATCH = {self.MATCH_WINDOW}\n'
            '----------------------------------------\n'
            f' avance = {self.LINEAR_SPEED:.2f} m/s\n'
            f' giro   = {self.ANGULAR_SPEED:.2f} rad/s\n'
            f' tiempo giro = {self.ROTATE_TIME:.1f} s\n'
            f' sonar STOP = {self.SONAR_STOP_DISTANCE:.2f} m\n'
            '----------------------------------------\n'
            ' GLOBAL RESET AUTOMATICO AL INICIO\n'
            f' BAD HYP = {self.BAD_HYPOTHESIS_TIME:.1f} s\n'
            f' cooldown = {self.GLOBAL_RESET_COOLDOWN:.1f} s\n'
            f' max resets = {self.MAX_GLOBAL_RESETS}\n'
            '========================================'
        )

    # ================================================================
    # MAP
    # ================================================================

    def map_callback(self, msg):

        first_map = self.map_msg is None
        self.map_msg = msg

        if first_map:

            self.get_logger().info(
                f'Mapa recibido: '
                f'{msg.info.width} x {msg.info.height} | '
                f'{msg.info.resolution:.3f} m/celda'
            )

    # ================================================================
    # AMCL
    # ================================================================

    def amcl_callback(self, msg):

        cov = msg.pose.covariance

        self.sigma_x = math.sqrt(
            max(cov[0], 0.0)
        )

        self.sigma_y = math.sqrt(
            max(cov[7], 0.0)
        )

        self.sigma_yaw = math.degrees(
            math.sqrt(
                max(cov[35], 0.0)
            )
        )

    # ================================================================
    # SONAR
    # ================================================================

    def sonar_callback(self, msg):

        self.last_sonar_time = time.monotonic()

        valid_ranges = [
            r
            for r in msg.ranges
            if (
                math.isfinite(r)
                and r >= msg.range_min
                and r <= msg.range_max
            )
        ]

        if not valid_ranges:

            self.sonar_distance = float('inf')
            self.sonar_obstacle = False

            return

        self.sonar_distance = min(valid_ranges)

        self.sonar_obstacle = (
            self.sonar_distance
            < self.SONAR_STOP_DISTANCE
        )

    def sonar_alive(self):

        if self.last_sonar_time is None:
            return False

        return (
            time.monotonic()
            - self.last_sonar_time
            < self.SONAR_TIMEOUT
        )

    # ================================================================
    # MAP MATCH
    # ================================================================

    def occupied_near(
        self,
        mx,
        my,
        radius_cells
    ):

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        data = self.map_msg.data

        for dy in range(
            -radius_cells,
            radius_cells + 1
        ):

            for dx in range(
                -radius_cells,
                radius_cells + 1
            ):

                if (
                    dx * dx + dy * dy
                    > radius_cells * radius_cells
                ):
                    continue

                x = mx + dx
                y = my + dy

                if x < 0 or x >= width:
                    continue

                if y < 0 or y >= height:
                    continue

                index = y * width + x

                if (
                    data[index]
                    >= self.OCCUPIED_THRESHOLD
                ):
                    return True

        return False

    # ================================================================
    # LIDAR
    # ================================================================

    def scan_callback(self, scan):

        if self.map_msg is None:
            return

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                scan.header.frame_id,
                rclpy.time.Time()
            )

        except TransformException as ex:

            self.get_logger().warn(
                f'No pude obtener TF map <- '
                f'{scan.header.frame_id}: {ex}',
                throttle_duration_sec=2.0
            )

            return

        # ------------------------------------------------------------
        # Transform
        # ------------------------------------------------------------

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        q = transform.transform.rotation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        yaw_tf = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        cos_yaw = math.cos(yaw_tf)
        sin_yaw = math.sin(yaw_tf)

        # ------------------------------------------------------------
        # Mapa
        # ------------------------------------------------------------

        resolution = (
            self.map_msg.info.resolution
        )

        origin_x = (
            self.map_msg.info.origin.position.x
        )

        origin_y = (
            self.map_msg.info.origin.position.y
        )

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        radius_cells = max(
            1,
            int(
                math.ceil(
                    self.MATCH_RADIUS
                    / resolution
                )
            )
        )

        # ------------------------------------------------------------
        # MATCH
        # ------------------------------------------------------------

        valid_points = 0
        matched_points = 0

        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):

            if i % self.SCAN_STEP != 0:

                angle += scan.angle_increment
                continue

            if (
                not math.isfinite(r)
                or r < scan.range_min
                or r > scan.range_max
            ):

                angle += scan.angle_increment
                continue

            # Punto LiDAR
            x_laser = (
                r * math.cos(angle)
            )

            y_laser = (
                r * math.sin(angle)
            )

            # LiDAR -> MAP
            x_map = (
                tx
                + cos_yaw * x_laser
                - sin_yaw * y_laser
            )

            y_map = (
                ty
                + sin_yaw * x_laser
                + cos_yaw * y_laser
            )

            # MAP -> celda
            mx = int(
                math.floor(
                    (x_map - origin_x)
                    / resolution
                )
            )

            my = int(
                math.floor(
                    (y_map - origin_y)
                    / resolution
                )
            )

            if (
                mx < 0
                or mx >= width
                or my < 0
                or my >= height
            ):

                angle += scan.angle_increment
                continue

            valid_points += 1

            if self.occupied_near(
                mx,
                my,
                radius_cells
            ):

                matched_points += 1

            angle += scan.angle_increment

        if valid_points == 0:
            return

        self.current_match = (
            100.0
            * matched_points
            / valid_points
        )

        self.match_history.append(
            self.current_match
        )

    # ================================================================
    # MATCH AVG
    # ================================================================

    def get_match_average(self):

        if (
            len(self.match_history)
            < self.MATCH_WINDOW
        ):
            return None

        return (
            sum(self.match_history)
            / len(self.match_history)
        )

    # ================================================================
    # COVARIANZA
    # ================================================================

    def covariance_good(self):

        if (
            self.sigma_x is None
            or self.sigma_y is None
            or self.sigma_yaw is None
        ):
            return False

        return (
            self.sigma_x < self.SIGMA_X_MAX
            and self.sigma_y < self.SIGMA_Y_MAX
            and self.sigma_yaw < self.SIGMA_YAW_MAX
        )

    # ================================================================
    # MOVIMIENTO
    #
    # 0 = AVANZAR
    # 1 = GIRAR
    # ================================================================

    def search_motion(self):

        now = time.monotonic()

        phase_elapsed = (
            now - self.phase_start
        )

        cmd = Twist()

        # ============================================================
        # FASE 0 - AVANZAR
        # ============================================================

        if self.search_phase == 0:

            # Sin sonar -> no avanzar
            if not self.sonar_alive():

                self.stop_robot()

                self.get_logger().warn(
                    'SIN DATOS DE /sonar_scan -> '
                    'avance bloqueado',
                    throttle_duration_sec=1.0
                )

                return

            # Obstáculo
            if self.sonar_obstacle:

                self.stop_robot()

                self.get_logger().warn(
                    f'OBSTACULO A '
                    f'{self.sonar_distance:.2f} m -> '
                    f'GIRAR {self.ROTATE_TIME:.1f} s'
                )

                self.search_phase = 1
                self.phase_start = now

                return

            # Libre -> avanzar
            cmd.linear.x = self.LINEAR_SPEED
            cmd.angular.z = 0.0

        # ============================================================
        # FASE 1 - GIRAR
        # ============================================================

        elif self.search_phase == 1:

            cmd.linear.x = 0.0
            cmd.angular.z = self.ANGULAR_SPEED

            if (
                phase_elapsed
                >= self.ROTATE_TIME
            ):

                self.stop_robot()

                self.search_phase = 0
                self.phase_start = now

                self.get_logger().info(
                    'RUTINA: giro terminado -> avanzar'
                )

                return

        self.cmd_pub.publish(cmd)

    # ================================================================
    # GLOBAL LOCALIZATION
    # ================================================================

    def trigger_global_localization(
        self,
        reason='recovery'
    ):

        if self.global_reset_pending:
            return False

        if (
            not
            self.global_localization_client.service_is_ready()
        ):

            self.get_logger().warn(
                'Servicio /reinitialize_global_localization '
                'no disponible.'
            )

            return False

        self.stop_robot()

        if reason == 'initial':

            self.get_logger().warn(
                '\n'
                '========================================\n'
                '     GLOBAL LOCALIZATION INICIAL\n'
                '========================================\n'
                'Generando hipotesis por todo el mapa...\n'
                '========================================'
            )

        else:

            self.get_logger().warn(
                '\n'
                '========================================\n'
                '    HIPOTESIS AMCL INCONSISTENTE\n'
                '========================================\n'
                'Covarianza buena pero MATCH bajo.\n'
                'Generando nuevas hipotesis globales...\n'
                '========================================'
            )

        self.global_reset_pending = True

        request = Empty.Request()

        future = (
            self.global_localization_client.call_async(
                request
            )
        )

        future.add_done_callback(
            self.global_localization_done
        )

        return True

    # ================================================================
    # RESET INICIAL
    # ================================================================

    def try_initial_global_localization(self):

        if self.initial_global_reset_done:
            return

        if (
            not
            self.global_localization_client.service_is_ready()
        ):

            self.get_logger().warn(
                'Esperando servicio '
                '/reinitialize_global_localization...',
                throttle_duration_sec=2.0
            )

            return

        # Lo marcamos solamente cuando efectivamente
        # vamos a hacer la llamada.
        self.initial_global_reset_done = True

        success = self.trigger_global_localization(
            reason='initial'
        )

        # Si por algún motivo no pudo arrancar,
        # permitimos reintentar.
        if not success:
            self.initial_global_reset_done = False

    # ================================================================
    # GLOBAL LOCALIZATION TERMINADA
    # ================================================================

    def global_localization_done(
        self,
        future
    ):

        self.global_reset_pending = False

        try:

            future.result()

        except Exception as ex:

            self.get_logger().error(
                f'Error ejecutando global localization: '
                f'{ex}'
            )

            return

        now = time.monotonic()

        self.global_reset_count += 1
        self.last_global_reset = now

        # Limpiar evidencia de la hipótesis anterior
        self.match_history.clear()
        self.current_match = None

        self.stable_since = None
        self.bad_hypothesis_since = None

        # Después del reset arrancamos avanzando
        self.search_phase = 0
        self.phase_start = now

        self.get_logger().warn(
            '\n'
            '========================================\n'
            '     GLOBAL LOCALIZATION EJECUTADA\n'
            '========================================\n'
            f'Reset {self.global_reset_count}/'
            f'{self.MAX_GLOBAL_RESETS}\n'
            'Particulas redistribuidas por el mapa.\n'
            f'Cooldown: '
            f'{self.GLOBAL_RESET_COOLDOWN:.0f} s\n'
            'Movimiento: AVANZAR\n'
            '========================================'
        )

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        now = time.monotonic()

        # ------------------------------------------------------------
        # Ya localizado
        # ------------------------------------------------------------

        if self.localized:

            self.stop_robot()
            return

        # ============================================================
        # RESET GLOBAL OBLIGATORIO AL ARRANCAR
        # ============================================================

        if not self.initial_global_reset_done:

            self.stop_robot()

            self.try_initial_global_localization()

            return

        # ------------------------------------------------------------
        # Esperando respuesta del reset
        # ------------------------------------------------------------

        if self.global_reset_pending:

            self.stop_robot()
            return

        # ------------------------------------------------------------
        # Timeout
        # ------------------------------------------------------------

        elapsed = (
            now - self.start_time
        )

        if (
            elapsed
            >= self.SEARCH_TIMEOUT
        ):

            if not self.timed_out:

                self.timed_out = True

                self.stop_robot()

                self.get_logger().error(
                    '\n'
                    '========================================\n'
                    ' TIMEOUT AUTOLOCALIZACION V3\n'
                    '========================================\n'
                    'No se encontro localizacion valida.\n'
                    f'Resets realizados: '
                    f'{self.global_reset_count}\n'
                    'PAPA detenido.\n'
                    '========================================'
                )

            return

        # ------------------------------------------------------------
        # Esperar datos
        # ------------------------------------------------------------

        if (
            self.map_msg is None
            or self.sigma_x is None
            or self.current_match is None
        ):
            # Aunque todavía no haya datos suficientes para evaluar
            # la localización, PAPA debe seguir explorando.
            self.search_motion()
            return

        # ------------------------------------------------------------
        # Evaluación
        # ------------------------------------------------------------

        cov_good = (
            self.covariance_good()
        )

        match_avg = (
            self.get_match_average()
        )

        match_good = (
            match_avg is not None
            and match_avg >= self.MATCH_AVG_MIN
        )

        candidate = (
            cov_good
            and match_good
        )

        # ============================================================
        # COOLDOWN
        # ============================================================

        in_global_cooldown = (
            self.last_global_reset is not None
            and (
                now
                - self.last_global_reset
                < self.GLOBAL_RESET_COOLDOWN
            )
        )

        # ============================================================
        # HIPOTESIS INCORRECTA
        # ============================================================

        if (
            cov_good
            and match_avg is not None
            and not match_good
            and not in_global_cooldown
        ):

            if (
                self.bad_hypothesis_since
                is None
            ):

                self.bad_hypothesis_since = now

                self.get_logger().warn(
                    '\n'
                    '>>> POSIBLE HIPOTESIS INCORRECTA <<<\n'
                    'COV=OK pero MATCH=NO.\n'
                    'Iniciando temporizador V3...'
                )

            bad_time = (
                now
                - self.bad_hypothesis_since
            )

            if (
                bad_time
                >= self.BAD_HYPOTHESIS_TIME
            ):

                if (
                    self.global_reset_count
                    < self.MAX_GLOBAL_RESETS
                ):

                    self.get_logger().warn(
                        f'Hipotesis inconsistente durante '
                        f'{bad_time:.1f} s -> GLOBAL RESET'
                    )

                    if self.trigger_global_localization(
                        reason='recovery'
                    ):
                        return

                else:

                    self.bad_hypothesis_since = None

                    self.get_logger().warn(
                        'MAX_GLOBAL_RESETS alcanzado.',
                        throttle_duration_sec=5.0
                    )

        else:

            if not in_global_cooldown:

                self.bad_hypothesis_since = None

        # ============================================================
        # LOCALIZACION CORRECTA
        # ============================================================

        if candidate:

            if self.stable_since is None:

                self.stable_since = now

                self.get_logger().info(
                    '\n'
                    '>>> CANDIDATO A LOCALIZACION <<<\n'
                    'COV=OK + MATCH=OK.\n'
                    'Verificando estabilidad...'
                )

            stable_time = (
                now
                - self.stable_since
            )

            if (
                stable_time
                >= self.STABLE_TIME
            ):

                self.localized = True

                self.stop_robot()

                self.get_logger().info(
                    '\n'
                    '========================================\n'
                    '       PAPA LOCALIZADO - V3\n'
                    '========================================\n'
                    f'sigma_x   = '
                    f'{self.sigma_x:.3f} m\n'
                    f'sigma_y   = '
                    f'{self.sigma_y:.3f} m\n'
                    f'sigma_yaw = '
                    f'{self.sigma_yaw:.2f} deg\n'
                    f'MATCH     = '
                    f'{self.current_match:.1f} %\n'
                    f'MATCH AVG = '
                    f'{match_avg:.1f} %\n'
                    f'RESETS    = '
                    f'{self.global_reset_count}\n'
                    '========================================'
                )

                return

        else:

            if (
                self.stable_since
                is not None
            ):

                self.get_logger().warn(
                    'Candidato rechazado: '
                    'se perdio COV o MATCH.'
                )

            self.stable_since = None

        # ============================================================
        # MOVIMIENTO
        # ============================================================

        self.search_motion()

        # ============================================================
        # LOG
        # ============================================================

        if (
            now - self.last_log_time
            >= 1.0
        ):

            self.last_log_time = now

            # MATCH
            if match_avg is None:

                match_text = (
                    f'llenando ventana '
                    f'({len(self.match_history)}/'
                    f'{self.MATCH_WINDOW})'
                )

            else:

                match_text = (
                    f'{match_avg:.1f}%'
                )

            # COV
            cov_status = (
                'OK'
                if cov_good
                else 'NO'
            )

            # MATCH
            match_status = (
                'OK'
                if match_good
                else 'NO'
            )

            # SONAR
            if not self.sonar_alive():

                sonar_text = 'SIN DATOS'

            elif math.isfinite(
                self.sonar_distance
            ):

                sonar_text = (
                    f'{self.sonar_distance:.2f} m'
                )

            else:

                sonar_text = 'LIBRE'

            # BAD HYP
            if (
                self.bad_hypothesis_since
                is not None
            ):

                bad_elapsed = (
                    now
                    - self.bad_hypothesis_since
                )

                bad_text = (
                    f'{bad_elapsed:.1f}/'
                    f'{self.BAD_HYPOTHESIS_TIME:.1f} s'
                )

            else:

                bad_text = '-'

            # COOLDOWN
            if in_global_cooldown:

                remaining = max(
                    0.0,
                    self.GLOBAL_RESET_COOLDOWN
                    - (
                        now
                        - self.last_global_reset
                    )
                )

                cooldown_text = (
                    f'{remaining:.1f} s'
                )

            else:

                cooldown_text = '-'

            # MOVIMIENTO
            if self.search_phase == 0:
                movement_text = 'AVANZANDO'
            else:
                movement_text = 'GIRANDO'

            self.get_logger().info(
                '\n'
                f'σx={self.sigma_x:.3f} m | '
                f'σy={self.sigma_y:.3f} m | '
                f'σyaw={self.sigma_yaw:.2f} deg\n'
                f'MATCH={self.current_match:.1f}% | '
                f'AVG={match_text}\n'
                f'COV={cov_status} | '
                f'MATCH={match_status}\n'
                f'SONAR={sonar_text}\n'
                f'MOV={movement_text}\n'
                f'BAD HYP={bad_text} | '
                f'RESETS={self.global_reset_count}/'
                f'{self.MAX_GLOBAL_RESETS}\n'
                f'GLOBAL COOLDOWN={cooldown_text}'
            )

    # ================================================================
    # STOP
    # ================================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = AutoLocalizationV3()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Autolocalizacion V3 cancelada.'
        )

        node.stop_robot()

    finally:

        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
