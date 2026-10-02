from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_name = "papa_description"

    package_share = get_package_share_directory(package_name)
    home_dir = os.path.expanduser("~")

    # ============================================================
    # RUTAS
    # ============================================================

    default_model_path = os.path.join(
        package_share,
        "urdf",
        "papa.urdf.xacro"
    )

    mapa_path = os.path.join(
        home_dir,
        "papa",
        "Mappeo",
        "paseo_colon_1er.yaml"
    )

    nav2_params_path = os.path.join(
        home_dir,
        "papa",
        "Mappeo",
        "nav2_params_sim.yaml"
    )

    # ============================================================
    # URDF
    # ============================================================

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Ruta absoluta al URDF/Xacro"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                LaunchConfiguration("model")
            ]),
            value_type=str
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": False}
        ]
    )

    # ============================================================
    # MAP SERVER
    # ============================================================

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": mapa_path,
                "use_sim_time": False
            }
        ]
    )

    # ============================================================
    # LIFECYCLE DEL MAP SERVER
    # ============================================================

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_sim_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"]
            }
        ]
    )

    # ============================================================
    # LOCALIZACION PERFECTA SIMULADA
    #
    # map -> odom
    #
    # papa_simulator.py publica:
    #
    # odom -> base_footprint
    # ============================================================

    map_to_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_sim",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom"
        ]
    )

    # ============================================================
    # NAV2
    #
    # IMPORTANTE:
    # navigation_launch.py NO lanza AMCL ni map_server.
    # ============================================================

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": nav2_params_path,
            "autostart": "True"
        }.items()
    )

    # ============================================================
    # RVIZ
    # ============================================================

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_sim",
        output="screen"
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([

        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION",
            "rmw_cyclonedds_cpp"
        ),

        model_arg,

        robot_state_publisher_node,

        map_server_node,
        map_lifecycle_manager,

        map_to_odom_node,

        nav2_navigation,

        rviz_node
    ])
