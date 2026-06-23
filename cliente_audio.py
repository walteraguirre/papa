import subprocess
import numpy as np
import socket
import sys

# --- CONFIGURACIÓN DEL SOCKET UDP ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def calcular_rms(data):
    # Convertimos los bytes crudos a números
    data_np = np.frombuffer(data, dtype=np.int16)
    if len(data_np) == 0: return 0
    return np.sqrt(np.mean(np.square(data_np, dtype=np.float64)))

print("Buscando el monitor de audio del sistema...")

try:
    # Le preguntamos directamente a PipeWire cuál es la salida actual (los parlantes)
    salida = subprocess.check_output(["pactl", "get-default-sink"], text=True).strip()
    # El monitor siempre se llama igual que la salida, pero terminado en ".monitor"
    monitor_name = salida + ".monitor"
    print(f"¡Monitor detectado!: {monitor_name}")
except Exception as e:
    print("No se pudo detectar el monitor automáticamente. Intentando captura general.")
    monitor_name = None

# Armamos el comando nativo de Linux (parec) para capturar el audio en bruto
cmd = ["parec", "--format=s16le", "--rate=44100", "--channels=2", "--latency-msec=10"]
if monitor_name:
    cmd.extend(["-d", monitor_name])

print("Iniciando captura de audio nativa con PipeWire...")

try:
    # Ejecutamos el comando en segundo plano y capturamos su salida
    proceso = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
except FileNotFoundError:
    print("Error: falta una herramienta del sistema.")
    print("Instálala en la terminal con: sudo apt-get install pulseaudio-utils")
    sys.exit(1)

# Parámetros de sensibilidad
PISO_RUIDO = 300
AUDIO_MAXIMO = 15000
# 4096 bytes = 1024 frames de audio estéreo a 16-bit
CHUNK_SIZE = 2048 

try:
    while True:
        # Leemos el audio directamente de la salida del comando de Linux
        raw_data = proceso.stdout.read(CHUNK_SIZE)
        if not raw_data:
            break
        
        rms = calcular_rms(raw_data)

        if rms < PISO_RUIDO: 
            rms = 0

        intensidad = (rms - PISO_RUIDO) / (AUDIO_MAXIMO - PISO_RUIDO)
        intensidad = max(0.0, min(1.0, intensidad))

        # Enviamos la intensidad por red interna al servidor de LEDs
        sock.sendto(str(intensidad).encode('utf-8'), (UDP_IP, UDP_PORT))

except KeyboardInterrupt:
    print("\nDeteniendo cliente de audio...")
finally:
    # Limpieza: matamos el proceso de captura y cerramos la conexión
    proceso.terminate()
    sock.close()
    print("Apagado correctamente.")
