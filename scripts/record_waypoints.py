#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


MAPS_DIR = Path.home() / "papa" / "Mappeo"


def available_maps():
    return sorted(
        map_path
        for map_path in MAPS_DIR.glob("*.yaml")
        if not map_path.stem.endswith("_obs")
        and not map_path.stem.endswith("_waypoints")
        and map_path.with_suffix(".pgm").exists()
    )


def select_map():
    map_files = available_maps()
    if not map_files:
        raise RuntimeError(
            f"No se encontraron mapas válidos (YAML + PGM) en {MAPS_DIR}."
        )

    print("\nMapas disponibles:")
    for index, map_path in enumerate(map_files, start=1):
        print(f"  [{index}] {map_path.stem}")

    while True:
        try:
            selection = int(input("\nSelecciona el número del mapa: "))
            if 1 <= selection <= len(map_files):
                return map_files[selection - 1]
        except ValueError:
            pass
        print("Selección no válida. Introduce uno de los números mostrados.")


class WaypointRecorder(Node):
    def __init__(self, map_path):
        super().__init__("waypoint_recorder")
        self.subscription = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10,
        )
        self.waypoints = {}
        self.output_file = map_path.with_name(f"{map_path.stem}_waypoints.yaml")
        self.get_logger().info(
            f"Guardando waypoints de '{map_path.stem}' en {self.output_file}"
        )

        if self.output_file.exists():
            with self.output_file.open("r", encoding="utf-8") as file:
                data = yaml.safe_load(file) or {}
            self.waypoints = data.get("waypoints", {})
            self.get_logger().info(
                f"Se cargaron {len(self.waypoints)} waypoints existentes."
            )

    def goal_callback(self, msg):
        while True:
            waypoint_name = input("\nNombre para el nuevo waypoint: ").strip()
            if not waypoint_name:
                print("El nombre no puede estar vacío.")
                continue
            if waypoint_name in self.waypoints:
                print(f"Ya existe un waypoint llamado '{waypoint_name}'.")
                continue
            break

        orientation = msg.pose.orientation
        siny_cosp = 2 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1 - 2 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.waypoints[waypoint_name] = {
            "pose": [
                round(msg.pose.position.x, 6),
                round(msg.pose.position.y, 6),
                0,
            ],
            "orientation": [
                round(orientation.x, 6),
                round(orientation.y, 6),
                round(orientation.z, 6),
                round(orientation.w, 6),
            ],
            "x": round(msg.pose.position.x, 2),
            "y": round(msg.pose.position.y, 2),
            "yaw": round(yaw, 2),
        }

        with self.output_file.open("w", encoding="utf-8") as file:
            yaml.safe_dump(
                {"waypoints": self.waypoints},
                file,
                allow_unicode=True,
                sort_keys=False,
            )

        self.get_logger().info(
            f"Guardado '{waypoint_name}' en {self.output_file}"
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        map_path = select_map()
    except RuntimeError as error:
        print(f"Error: {error}")
        rclpy.shutdown()
        return

    recorder = WaypointRecorder(map_path)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
