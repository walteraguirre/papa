#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


from tf2_ros import Buffer, TransformListener, TransformException

import math


class LidarMapScore(Node):

    def __init__(self):
        super().__init__('lidar_map_score')

        # ============================================================
        # PARÁMETROS DE LA PRUEBA
        # ============================================================

        # Una celda se considera obstáculo a partir de este valor.
        self.OCCUPIED_THRESHOLD = 65

        # Permitimos cierta distancia entre el impacto del LiDAR
        # y una celda ocupada.
        self.MATCH_RADIUS = 0.15       # metros

        # Para no procesar necesariamente todos los rayos.
        # 1 = todos, 2 = uno de cada dos, etc.
        self.SCAN_STEP = 2

        # ============================================================
        # DATOS
        # ============================================================

        self.map_msg = None

        self.sigma_x = None
        self.sigma_y = None
        self.sigma_yaw = None

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # SUSCRIPCIONES
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # QoS compatible con sensores / LaserScan
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.get_logger().info(
            '========================================\n'
            ' LIDAR <-> MAP SCORE - PAPA\n'
            '========================================\n'
            'Esperando /map, /scan y /amcl_pose...\n'
            'Este nodo NO controla el robot.\n'
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
                f'resolucion = {msg.info.resolution:.3f} m/celda'
            )

    # ================================================================
    # AMCL
    # ================================================================

    def amcl_callback(self, msg):

        cov = msg.pose.covariance

        var_x = max(cov[0], 0.0)
        var_y = max(cov[7], 0.0)
        var_yaw = max(cov[35], 0.0)

        self.sigma_x = math.sqrt(var_x)
        self.sigma_y = math.sqrt(var_y)
        self.sigma_yaw = math.degrees(
            math.sqrt(var_yaw)
        )

    # ================================================================
    # BUSCAR OBSTÁCULO CERCA DE UNA CELDA
    # ================================================================

    def occupied_near(self, mx, my, radius_cells):

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        data = self.map_msg.data

        for dy in range(-radius_cells, radius_cells + 1):

            for dx in range(-radius_cells, radius_cells + 1):

                # Radio circular, no cuadrado
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue

                x = mx + dx
                y = my + dy

                if x < 0 or x >= width:
                    continue

                if y < 0 or y >= height:
                    continue

                index = y * width + x

                if data[index] >= self.OCCUPIED_THRESHOLD:
                    return True

        return False

    # ================================================================
    # SCAN
    # ================================================================

    def scan_callback(self, scan):

        if self.map_msg is None:
            return

        # ------------------------------------------------------------
        # Obtener transformación:
        #
        # scan.header.frame_id ---> map
        #
        # Normalmente será:
        #
        # laser_frame -> base_link -> odom -> map
        #
        # ------------------------------------------------------------

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                scan.header.frame_id,
                rclpy.time.Time()
            )

        except TransformException as ex:

            self.get_logger().warn(
                f'No pude obtener TF map <- {scan.header.frame_id}: {ex}',
                throttle_duration_sec=2.0
            )

            return

        # ------------------------------------------------------------
        # Transformación 2D
        # ------------------------------------------------------------

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        q = transform.transform.rotation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw_tf = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        cos_yaw = math.cos(yaw_tf)
        sin_yaw = math.sin(yaw_tf)

        # ------------------------------------------------------------
        # Datos del mapa
        # ------------------------------------------------------------

        resolution = self.map_msg.info.resolution

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        radius_cells = max(
            1,
            int(math.ceil(
                self.MATCH_RADIUS / resolution
            ))
        )

        # ------------------------------------------------------------
        # Comparación scan <-> mapa
        # ------------------------------------------------------------

        valid_points = 0
        matched_points = 0

        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):

            if i % self.SCAN_STEP != 0:
                angle += scan.angle_increment
                continue

            # Rayos inválidos
            if (
                not math.isfinite(r)
                or r < scan.range_min
                or r > scan.range_max
            ):
                angle += scan.angle_increment
                continue

            # --------------------------------------------------------
            # Punto en coordenadas del LiDAR
            # --------------------------------------------------------

            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)

            # --------------------------------------------------------
            # Transformarlo al frame MAP
            # --------------------------------------------------------

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

            # --------------------------------------------------------
            # Coordenadas mundo -> celda
            # --------------------------------------------------------

            mx = int(
                math.floor(
                    (x_map - origin_x) / resolution
                )
            )

            my = int(
                math.floor(
                    (y_map - origin_y) / resolution
                )
            )

            # Punto fuera del mapa
            if (
                mx < 0 or mx >= width
                or
                my < 0 or my >= height
            ):
                angle += scan.angle_increment
                continue

            valid_points += 1

            # --------------------------------------------------------
            # ¿Hay una pared cerca?
            # --------------------------------------------------------

            if self.occupied_near(
                mx,
                my,
                radius_cells
            ):
                matched_points += 1

            angle += scan.angle_increment

        # ------------------------------------------------------------
        # SCORE
        # ------------------------------------------------------------

        if valid_points == 0:
            return

        score = (
            matched_points /
            valid_points
        ) * 100.0

        # ------------------------------------------------------------
        # SALIDA
        # ------------------------------------------------------------

        print()
        print('========================================')
        print('        LIDAR <-> MAP SCORE')
        print('========================================')

        if self.sigma_x is not None:

            print(
                f'sigma_x   = {self.sigma_x:.3f} m'
            )

            print(
                f'sigma_y   = {self.sigma_y:.3f} m'
            )

            print(
                f'sigma_yaw = {self.sigma_yaw:.2f} deg'
            )

            print('----------------------------------------')

        print(
            f'Puntos validos : {valid_points}'
        )

        print(
            f'Coincidencias  : {matched_points}'
        )

        print(
            f'MATCH          : {score:.1f} %'
        )

        print('========================================')


def main(args=None):

    rclpy.init(args=args)

    node = LidarMapScore()

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
