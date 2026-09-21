from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = "papa_description"
    default_model_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "papa.urdf.xacro"
    )
    
    mapa_path = "/home/vboxuser/Mappeo/mi_primer_mapa.yaml"
    nav2_params_path = "/home/vboxuser/Mappeo/nav2_params.yaml"

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Ruta absoluta al archivo URDF/Xacro"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("model")]),
            value_type=str
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": mapa_path}]
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "scan_topic": "scan",
            "base_frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "global_frame_id": "map",
            "set_initial_pose": True,
            "initial_pose.x": 0.0,
            "initial_pose.y": 0.0,
            "initial_pose.yaw": 0.0
        }]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapper",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
            "bond_timeout": 4.0, 
            "attempt_respawn_reconnection": True
        }]
    )

    # NUEVO: Módulo de Navegación Autónomo
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_path
        }.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        model_arg,
        robot_state_publisher_node,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        nav2_navigation,
        pose_memory_node,  #recuperamos posición
        rviz_node
    ])
