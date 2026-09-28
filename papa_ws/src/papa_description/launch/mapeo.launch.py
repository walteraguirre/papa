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
    
    # ==========================================================
    # CONFIGURACIÓN DE MAPAS ACTUALIZABLES (LIFELONG MAPPING)
    # ==========================================================
    # Para empezar un mapa de cero, déjalo vacío: ""
    # Para continuar un mapa, escribe la ruta SIN la extensión .posegraph
    # Ejemplo: "/home/perez_010/papa/Mappeo/mapa_maestro"
    mapa_a_continuar = "/home/perez_010/papa/Mappeo/paseo_colon_pb" 
    
    robot_description = {"robot_description": ParameterValue(Command(["xacro ", urdf_path]), value_type=str)}

    # 1. Definimos los parámetros base como un diccionario de Python
    slam_params = {
        'use_sim_time': False,
        'odom_frame': 'odom',
        'base_frame': 'base_footprint',
        'map_frame': 'map',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'max_laser_range': 10.0,
        'resolution': 0.05,
        'transform_timeout': 0.5,
        'minimum_travel_distance': 0.3,
        'minimum_travel_heading': 0.5,
        'correlation_search_space_dimension': 0.3,
        'correlation_search_space_resolution': 0.01,
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty': 1.5
    }

    # 2. Lógica de inyección: Si el usuario escribió una ruta, se agregan los parámetros de carga
    if mapa_a_continuar.strip():
        slam_params['map_file_name'] = mapa_a_continuar
        slam_params['map_start_pose'] = [0.0, 0.0, 0.0]  # Asume que arrancas desde el origen del mapa anterior

    # 3. Opcional: Cargar automáticamente tu diseño de RViz2 (paneles y colores)
    rviz_config_path = os.path.join(get_package_share_directory(package_name), "rviz", "mapeo.rviz")
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

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
            parameters=[slam_params]  # Pasamos el diccionario modificado
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=rviz_args       # Carga el diseño si existe
        )
    ])
