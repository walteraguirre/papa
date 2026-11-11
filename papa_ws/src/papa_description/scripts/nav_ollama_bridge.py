import rclpy
from rclpy.node import Node
import json
import math
import re
import os
import urllib.request
import urllib.error
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import yaml

MAPS_DIR = Path.home() / "papa" / "Mappeo"
WAYPOINTS_FILE = MAPS_DIR / "paseo_colon_1er_waypoints.yaml"
CONTEXT_FILE = MAPS_DIR / "paseo_colon_1er_context.txt"

class NavOllamaBridge(Node):
    def __init__(self):
        super().__init__('nav_ollama_bridge')
        
        # Cargar waypoints
        with open(WAYPOINTS_FILE, 'r', encoding='utf-8') as f:
            waypoints_data = yaml.safe_load(f) or {}
            self.waypoints = waypoints_data.get("waypoints", {})
            
        # Cargar historias / anécdotas del edificio si existe el archivo
        historias_texto = ""
        if CONTEXT_FILE.exists():
            with open(CONTEXT_FILE, 'r', encoding='utf-8') as f:
                historias_texto = f.read()

        # Contexto del robot
        self.contexto = {
            "nombre": "PAPA-01",
            "rol": "Robot asistente de exploración",
            "estado": "IDLE"
        }
            
        # Publicador para Nav2
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Historial de conversación con prompt del sistema enriquecido
        self.historial = [
            {"role": "system", "content": f"""
            Eres {self.contexto['nombre']}, un robot asistente en el Edificio Paseo Colón (FIUBA). 
            Tu personalidad es profesional, servicial y conocedora de la historia del edificio.
            
            Información del entorno e historias del edificio:
            {historias_texto}
            
            - Tu trabajo: Moverte entre laboratorios y zonas de estudio.
            - Waypoints/destinos disponibles EXACTOS: {list(self.waypoints.keys())}.
            
            IMPORTANTE - Reglas de respuesta:
            DEBES responder SIEMPRE únicamente en formato JSON estricto sin bloques Markdown extra.
            
            Caso A) Si el usuario pide ir o moverse a un lugar:
            {{"accion": "MOVER", "destino": "nombre_exacto_del_wp", "mensaje": "Respuesta hablada confirmando el movimiento"}}
            
            Caso B) Si es solo conversación o preguntas sobre el edificio/historia:
            {{"accion": "HABLAR", "mensaje": "Tu respuesta al usuario"}}
            """}
        ]
        
        self.get_logger().info(f"{self.contexto['nombre']} listo para chatear.")

    def enviar_objetivo(self, nombre_objetivo):
        if nombre_objetivo not in self.waypoints:
            self.get_logger().warn(f"Objetivo '{nombre_objetivo}' no encontrado en waypoints.")
            return

        wp = self.waypoints[nombre_objetivo]
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        if "x" in wp and "y" in wp and "yaw" in wp:
            x = wp["x"]
            y = wp["y"]
            yaw = wp["yaw"]
        else:
            x, y, _ = wp["pose"]
            orientation = wp["orientation"]
            yaw = math.atan2(
                2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]),
                1 - 2 * (orientation[1] ** 2 + orientation[2] ** 2),
            )
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        
        # Convertir yaw a cuaternión
        q = quaternion_from_euler(0, 0, float(yaw))
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        self.nav_pub.publish(goal)
        # Procesar eventos ROS2 para asegurar el envío del mensaje
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.01)
        self.get_logger().info(
            f"Objetivo publicado en /goal_pose: {nombre_objetivo} -> ({x}, {y})"
        )

    def procesar_comando(self, comando_usuario):
        self.historial.append({"role": "user", "content": comando_usuario})
        
        data = {
            "model": "llama3.1:8b",
            "messages": self.historial,
            "stream": False
        }
        
        req = urllib.request.Request(
            'http://localhost:11434/api/chat',
            data=json.dumps(data).encode('utf-8'),
            headers={'Content-Type': 'application/json'}
        )
        
        try:
            with urllib.request.urlopen(req) as response:
                result = json.loads(response.read().decode('utf-8'))
                raw_respuesta = result['message']['content'].strip()
                self.historial.append({"role": "assistant", "content": raw_respuesta})
                
                # Extraer JSON de la respuesta (incluso si está envuelto en ```json ... ``` o texto extra)
                match = re.search(r'\{.*\}', raw_respuesta, re.DOTALL)
                if match:
                    json_str = match.group(0)
                    try:
                        respuesta = json.loads(json_str)
                        mensaje = respuesta.get('mensaje', '')
                        
                        if respuesta.get('accion') == 'MOVER':
                            destino = respuesta.get('destino')
                            if mensaje:
                                print(f"Robot: {mensaje}")
                            else:
                                print(f"Robot: Entendido, me muevo hacia '{destino}'.")
                            self.enviar_objetivo(destino)
                        else:
                            print(f"Robot: {mensaje if mensaje else raw_respuesta}")
                    except json.JSONDecodeError:
                        print(f"Robot: {raw_respuesta}")
                else:
                    print(f"Robot: {raw_respuesta}")
                    
        except Exception as e:
            self.get_logger().error(f"Error conectando a Ollama: {e}")

def main(args=None):
    rclpy.init(args=args)
    bridge = NavOllamaBridge()
    
    print(f"\n--- Chat iniciado con {bridge.contexto['nombre']} --- (Escribe 'exit' para salir)\n")
    try:
        while rclpy.ok():
            comando = input("> ")
            if comando.lower() == 'exit':
                break
            if comando.strip():
                bridge.procesar_comando(comando)
                rclpy.spin_once(bridge, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
        
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
