from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    mapa_path = os.path.join(home_dir, "papa", "Mappeo", "paseo_colon_1er.yaml")
    obs_mapa_path = os.path.splitext(mapa_path)[0] + "_obs.yaml"

    # Servidor de mapas solo para visualización
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": mapa_path}]
    )

    extra_map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="extra_map_server",
        output="screen",
        parameters=[{
            "yaml_filename": obs_mapa_path,
            "topic_name": "extra_map"
        }]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "extra_map_server"]
        }]
    )

    return LaunchDescription([
        map_server_node,
        extra_map_server_node,
        lifecycle_manager
    ])
