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

## Control web desde el celular

`control_web.py` publica los mismos comandos que `control_flechas.py`:
`VEL,izquierda,derecha` en `/comando_arduino`. Mantener presionado un botón
mantiene el movimiento; al soltarlo se publica `VEL,0,0`.

En la Raspberry, el archivo puede quedar directamente en `/home/papa`, sin
pertenecer a un paquete ROS:

```bash
chmod +x ~/control_web.py
source /opt/ros/jazzy/setup.bash
python3 ~/control_web.py
```

Desde el celular abrir `http://192.168.1.130:8080`. Para instalarlo como servicio,
copiar `systemd/papa-control-web@.service` a `/etc/systemd/system/`. El servicio
ejecuta `/home/papa/control_web.py` directamente y no necesita `papa_description`.
Para el usuario mostrado en la Raspberry:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now papa-control-web@papa.service
```

También se puede usar el unit simple `control-web.service`, que está preparado
específicamente para el usuario `papa`:

```bash
sudo cp control-web.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now control-web.service
```

Si la interfaz tiene otra dirección, iniciar el nodo con
`PAPA_WEB_HOST=<ip> PAPA_WEB_PORT=8080`.
