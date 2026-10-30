# Proyecto PAPA

Robot autónomo basado en ROS 2 Jazzy, diseñado para tareas de mapeo y navegación en interiores utilizando un LiDAR YDLIDAR X4.

## Descripción
Este proyecto integra hardware robótico (controlado mediante Arduino/ESP32) con una infraestructura de software en ROS 2 sobre Ubuntu 24.04 LTS, permitiendo la navegación autónoma y la creación de mapas del entorno.

## Especificaciones Técnicas
- **OS:** Ubuntu 24.04 LTS
- **Middleware:** ROS 2 Jazzy
- **Sensor:** YDLIDAR X4 (topic: `/scan`)
- **Navegación:** Nav2 Stack
- **Frameworks:** ROS 2, YDLidar-SDK

## Estructura del Workspace
- `~/papa_ws`: Workspace principal del robot.
- `~/ydlidar_ws`: Workspace dedicado al driver del LiDAR.
- `Mappeo/`: Configuraciones de navegación (Nav2, AMCL, Costmaps).
- `Arduino/`: Firmware para el microcontrolador (Papa Leonardo).

## Guía de Configuración
Para una instalación desde cero, consulte el archivo `papa_ros2_guia.txt` en la raíz del repositorio.

## Módulos Principales
- **Percepción:** Driver YDLIDAR X4 configurado para ROS 2.
- **Navegación:** Configuración de Nav2 para planificación de rutas, evitación de obstáculos (lidar + sonar) y localización (AMCL).
- **Control:** Scripts de interfaz para teleoperación y control del robot.
