import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory("papa_description")
    urdf_path = os.path.join(pkg_dir, "urdf", "papa.urdf.xacro")
    robot_desc = {"robot_description": ParameterValue(Command(["xacro ", urdf_path]), value_type=str)}

    # TUS PARÁMETROS CUSTOMIZADOS PARA EL ROBOT "PAPA"
    custom_slam_params = {
        'use_sim_time': False, 
        'base_frame': 'base_footprint', 
        'scan_topic': 'scan',
        
        # Tolerancia extra para la latencia entre la Raspberry Pi y la PC
        'transform_timeout': 2.0,
        
        # Ajustes de movimiento y optimización de mapeo
        'minimum_travel_distance': 0.3,
        'minimum_travel_heading': 0.5,
        'correlation_search_space_dimension': 0.3,
        'correlation_search_space_resolution': 0.01,
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty': 1.5,
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher", 
            executable="robot_state_publisher", 
            output="screen", 
            parameters=[robot_desc]
        ),
        Node(
            package="joint_state_publisher", 
            executable="joint_state_publisher", 
            output="screen"
        ),
        
        # EL CEREBRO DE SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_async.yaml'),
                custom_slam_params  # <--- Inyectamos toda tu configuración aquí
            ]
        ),
        
        # EL DESPERTADOR
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['slam_toolbox']
            }]
        ),
        
        Node(
            package="rviz2", 
            executable="rviz2", 
            output="screen"
        )
    ])
