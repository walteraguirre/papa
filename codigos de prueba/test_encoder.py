#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import threading
import time
import math

class OdomRotationDiagnostic(Node):
    def __init__(self):
        super().__init__('odom_rotation_diagnostic')
        self.sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            10
        )
        self.yaw = 0.0
        self.msg_count = 0

    def odom_callback(self, msg):
        self.msg_count += 1
        
        # Extraer Yaw (Rotación Z) desde el Cuaternión
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

# Función para evitar errores si el ángulo pasa de 180 a -180
def get_angle_diff(start, end):
    diff = end - start
    while diff > 180.0:
        diff -= 360.0
    while diff < -180.0:
        diff += 360.0
    return diff

def run_diagnostics(node):
    print("\n==================================================")
    print(" [DIAGNÓSTICO] ROTACIÓN Y ENCODERS - PAPA")
    print("==================================================\n")
    
    print("Esperando conexión con el tópico /odom...")
    timeout = 10
    start_wait = time.time()
    
    while node.msg_count == 0:
        if time.time() - start_wait > timeout:
            print("[ERROR CRÍTICO]: No hay datos en /odom.")
            print("Presiona Ctrl+C para salir.")
            return
        time.sleep(0.1)
        
    print(f"[OK]: Conexión establecida.\n")

    # ==========================================
    # PRUEBA 1: GIRO DERECHA
    # ==========================================
    print("--------------------------------------------------")
    input("-> PRUEBA 1: Gira el robot hacia la DERECHA (aprox 90 grados) y presiona ENTER...")
    
    start_yaw = node.yaw
    
    input("... Usa tu teleop para girar a la derecha. Presiona ENTER cuando termines ...")
    
    delta_yaw = get_angle_diff(start_yaw, node.yaw)
    
    print(f"\nResultados Prueba 1 (Giro a la Derecha):")
    print(f" - Rotación medida: {delta_yaw:.1f} grados")
    
    if delta_yaw < -30.0:
        print("   [OK] CORRECTO: En ROS, girar a la derecha es negativo.")
    elif delta_yaw > 30.0:
        print("   [FALLA GRAVE]: Giraste a la derecha, pero el sistema cree que giraste a la IZQUIERDA.")
        print("      Solución: Los cables de dirección de tu encoder derecho o izquierdo están invertidos.")
    else:
        print("   [FALLA GRAVE]: No se detectó rotación suficiente en el encoder.")

    # ==========================================
    # PRUEBA 2: GIRO IZQUIERDA
    # ==========================================
    print("\n--------------------------------------------------")
    input("-> PRUEBA 2: Gira el robot hacia la IZQUIERDA (aprox 90 grados) y presiona ENTER...")
    
    start_yaw = node.yaw
    
    input("... Usa tu teleop para girar a la izquierda. Presiona ENTER cuando termines ...")
    
    delta_yaw = get_angle_diff(start_yaw, node.yaw)
    
    print(f"\nResultados Prueba 2 (Giro a la Izquierda):")
    print(f" - Rotación medida: {delta_yaw:.1f} grados")
    
    if delta_yaw > 30.0:
        print("   [OK] CORRECTO: En ROS, girar a la izquierda es positivo.")
    elif delta_yaw < -30.0:
        print("   [FALLA GRAVE]: Giraste a la izquierda, pero el sistema cree que giraste a la DERECHA.")
    else:
        print("   [FALLA GRAVE]: No se detectó rotación suficiente.")

    print("\n==================================================")
    print(" [FIN] Diagnóstico finalizado. Presiona Ctrl+C para salir.")
    print("==================================================\n")

def main(args=None):
    rclpy.init(args=args)
    node = OdomRotationDiagnostic()
    
    cli_thread = threading.Thread(target=run_diagnostics, args=(node,))
    cli_thread.daemon = True
    cli_thread.start()
    
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
