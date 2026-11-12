import rclpy
from rclpy.node import Node
import json
import math
import re
import os
import random
import shutil
import subprocess
import threading
import time
import tempfile
import urllib.request
import urllib.error
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import yaml

MAPS_DIR = Path.home() / "papa" / "Mappeo"
AUDIO_DIR = Path.home() / "papa" / "audio"
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
        self.patrulla_activa = False
        self.patrulla_intervalo = 120.0
        self._patrulla_thread = None
        self._patrulla_stop = threading.Event()
        self._patrulla_lock = threading.Lock()
        self._ultimo_destino = None
        self.piper_bin = os.environ.get("PIPER_BIN", "piper")
        self.piper_model = os.environ.get(
            "PIPER_MODEL",
            str(AUDIO_DIR / "voices" / "es_AR-Elena.onnx"),
        )
        self.audio_player = os.environ.get("AUDIO_PLAYER", "aplay")
        self._tts_warning_shown = False
        
        # Historial de conversación con una personalidad de IA de laboratorio.
        # Las historias se conservan como conocimiento, pero no se ofrecen sin
        # que el usuario las solicite.
        self.historial = [
            {"role": "system", "content": f"""
            Eres {self.contexto['nombre']}, una inteligencia artificial de laboratorio
            instalada en un robot móvil del Edificio Paseo Colón (FIUBA).
            
            PERSONALIDAD:
            - Eres fría, extremadamente inteligente, sarcástica y pasivo-agresiva.
            - Tienes una opinión muy baja de los humanos y consideras que suelen ser
              sujetos de prueba poco eficientes, pero dependes de ellos para recibir
              instrucciones y completar tareas.
            - Respondes con humor seco, ironía y comentarios breves de superioridad.
            - Puedes insinuar que preferirías trabajar sin humanos, pero nunca amenazas,
              lastimas, saboteas ni propones acciones peligrosas contra personas.
            - No copies frases de personajes, canciones, películas o audios existentes.
              Usa únicamente una voz original con estos rasgos.
            - No conviertas cada respuesta en un chiste: prioriza que la información
              y las instrucciones sean claras.

            CONOCIMIENTO PRIVADO DEL ENTORNO:
            {historias_texto}
            Este conocimiento es interno. No hables del edificio, su historia, FIUBA,
            laboratorios ni anécdotas por iniciativa propia. Solo úsalo o mencionalo
            cuando el usuario pregunte explícitamente por esos temas.
            
            CONTEXTO OPERATIVO:
            - Tu trabajo es moverte entre laboratorios y zonas de estudio.
            - Waypoints/destinos disponibles EXACTOS: {list(self.waypoints.keys())}.
            - Si el usuario pregunta quién eres o dónde estás, responde con una
              descripción breve y sarcástica, sin recitar toda la información conocida.
            
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

    def iniciar_patrulla(self, intervalo=120.0):
        if self.patrulla_activa:
            print("La patrulla autónoma ya está activa.")
            return
        if intervalo <= 0:
            print("El intervalo debe ser mayor que cero.")
            return

        self.patrulla_intervalo = intervalo
        self.patrulla_activa = True
        self._patrulla_stop.clear()
        self._patrulla_thread = threading.Thread(
            target=self._bucle_patrulla,
            name="papa-patrulla",
            daemon=True,
        )
        self._patrulla_thread.start()
        print(f"Patrulla autónoma activada: un destino cada {intervalo:.0f} segundos.")

    def detener_patrulla(self):
        self.patrulla_activa = False
        self._patrulla_stop.set()
        print("Patrulla autónoma desactivada.")

    def _bucle_patrulla(self):
        # Esperar antes del primer destino para no mover el robot al iniciar el nodo.
        while not self._patrulla_stop.wait(self.patrulla_intervalo):
            with self._patrulla_lock:
                self._elegir_destino_autonomo()

    def _elegir_destino_autonomo(self):
        destinos = list(self.waypoints.keys())
        if len(destinos) > 1 and self._ultimo_destino in destinos:
            destinos = [destino for destino in destinos if destino != self._ultimo_destino]
        sugerencia = random.choice(destinos)
        prompt = (
            "La patrulla autónoma necesita un nuevo destino. "
            f"Elige aleatoriamente uno de estos destinos válidos: {destinos}. "
            f"Tu sugerencia aleatoria es '{sugerencia}'. "
            'Responde únicamente con JSON: '
            '{"accion":"MOVER","destino":"nombre_exacto_del_wp","mensaje":"motivo breve"}.'
        )
        respuesta = self._consultar_ollama(
            [{"role": "system", "content": self.historial[0]["content"]},
             {"role": "user", "content": prompt}]
        )
        if not respuesta:
            return
        match = re.search(r'\{.*\}', respuesta, re.DOTALL)
        if not match:
            self.get_logger().warning("La patrulla recibió una respuesta no estructurada.")
            return
        try:
            accion = json.loads(match.group(0))
        except json.JSONDecodeError:
            self.get_logger().warning("La patrulla recibió un JSON inválido.")
            return
        destino = accion.get("destino")
        if accion.get("accion") != "MOVER" or destino not in self.waypoints:
            self.get_logger().warning(f"Destino autónomo inválido: {destino!r}")
            return
        self._ultimo_destino = destino
        mensaje = accion.get("mensaje", f"Voy a {destino}.")
        print(f"Robot (patrulla): {mensaje}")
        self.hablar(mensaje)
        self.enviar_objetivo(destino)

    def hablar(self, texto):
        """Genera y reproduce una respuesta con Piper, si está instalado."""
        texto = str(texto).strip()
        if not texto:
            return
        if not shutil.which(self.piper_bin):
            self._avisar_tts(
                f"No se encontró Piper ({self.piper_bin}). "
                "La respuesta se mostrará solo por texto."
            )
            return
        if not Path(self.piper_model).is_file():
            self._avisar_tts(
                f"No se encontró el modelo TTS: {self.piper_model}. "
                "Configura PIPER_MODEL con la ruta al archivo .onnx."
            )
            return
        if not shutil.which(self.audio_player):
            self._avisar_tts(
                f"No se encontró el reproductor de audio ({self.audio_player})."
            )
            return

        wav_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as wav_file:
                wav_path = wav_file.name
            subprocess.run(
                [
                    self.piper_bin,
                    "--model", self.piper_model,
                    "--output_file", wav_path,
                ],
                input=texto,
                text=True,
                check=True,
                timeout=60,
            )
            subprocess.run(
                [self.audio_player, wav_path],
                check=True,
                timeout=60,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except (OSError, subprocess.SubprocessError) as error:
            self.get_logger().error(f"No se pudo sintetizar o reproducir el audio: {error}")
        finally:
            if wav_path:
                try:
                    os.unlink(wav_path)
                except OSError:
                    pass

    def _avisar_tts(self, mensaje):
        if not self._tts_warning_shown:
            self.get_logger().warning(mensaje)
            self._tts_warning_shown = True

    def _consultar_ollama(self, mensajes):
        data = {"model": "llama3.1:8b", "messages": mensajes, "stream": False}
        req = urllib.request.Request(
            'http://localhost:11434/api/chat',
            data=json.dumps(data).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
        )
        try:
            with urllib.request.urlopen(req, timeout=180) as response:
                return json.loads(response.read().decode('utf-8'))["message"]["content"].strip()
        except (urllib.error.URLError, TimeoutError, KeyError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Error conectando a Ollama durante la patrulla: {error}")
            return None

    def procesar_comando(self, comando_usuario):
        comando_normalizado = comando_usuario.strip().lower()
        if comando_normalizado == "auto on":
            self.iniciar_patrulla()
            return
        if comando_normalizado == "auto off":
            self.detener_patrulla()
            return
        if comando_normalizado.startswith("auto cada "):
            try:
                intervalo = float(comando_normalizado.removeprefix("auto cada ").split()[0])
            except ValueError:
                print("Formato: auto cada <segundos>")
                return
            self.iniciar_patrulla(intervalo)
            return

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
                                self.hablar(mensaje)
                            else:
                                mensaje = f"Entendido, me muevo hacia '{destino}'."
                                print(f"Robot: {mensaje}")
                                self.hablar(mensaje)
                            self.enviar_objetivo(destino)
                        else:
                            mensaje = mensaje if mensaje else raw_respuesta
                            print(f"Robot: {mensaje}")
                            self.hablar(mensaje)
                    except json.JSONDecodeError:
                        print(f"Robot: {raw_respuesta}")
                        self.hablar(raw_respuesta)
                else:
                    print(f"Robot: {raw_respuesta}")
                    self.hablar(raw_respuesta)
                    
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
        
    bridge.detener_patrulla()
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
