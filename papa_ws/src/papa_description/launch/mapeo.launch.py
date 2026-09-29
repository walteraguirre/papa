import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "papa_description"
    urdf_path = os.path.join(get_package_share_directory(package_name), "urdf", "papa.urdf.xacro")
    
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", urdf_path]), value_type=str)}

    # DICCIONARIO COMPLETO DE SLAM TOOLBOX
    slam_params = {
        # 1. Configuración de Tópicos y Frames
        'use_sim_time': False,
        'odom_frame': 'odom',
        'base_frame': 'base_link',  # El chasis real de tu URDF
        'map_frame': 'map',
        'scan_topic': '/scan',
        'mode': 'mapping',
        
        # 2. Motores Matemáticos (OBLIGATORIOS para que no se congele)
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_region_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        
        # 3. Ajustes de Mapeo y Red
        'max_laser_range': 10.0,
        'resolution': 0.05,
        'transform_timeout': 2.0,  # Tolerancia para la Raspberry Pi
        
        # 4. Penalizaciones y Optimizaciones
        'minimum_travel_distance': 0.3,
        'minimum_travel_heading': 0.5,
        'correlation_search_space_dimension': 0.3,
        'correlation_search_space_resolution': 0.01,
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty': 1.5,
        
        # LIFELONG MAPPING:
        # Para continuar un mapa, descomenta las siguientes líneas y pon la ruta:
        # 'map_file_name': '/home/perez_010/papa/Mappeo/mapa_maestro',
        # 'map_start_pose': [0.0, 0.0, 0.0]
    }

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        Node(
            package="robot_state_publisher", 
            executable="robot_state_publisher", 
            output="screen", 
            parameters=[robot_description]
        ),
        Node(
            package="joint_state_publisher", 
            executable="joint_state_publisher", 
            name="joint_state_publisher", 
            output="screen"
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]  # Inyectamos el diccionario directamente
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        )
    ])
