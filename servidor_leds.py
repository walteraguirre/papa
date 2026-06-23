import time
import board
import neopixel
import socket

# --- AHORA USAMOS GPIO 10 (SPI) ---
PIN = board.D10
NUM_PIXELS = 16
MAX_BRIGHTNESS = 0.3
COLOR_YELLOW = (255, 255, 0)

# El resto del servidor se mantiene igual
pixels = neopixel.NeoPixel(PIN, NUM_PIXELS, brightness=0.1, auto_write=False)
pixels.fill(COLOR_YELLOW)
pixels.show()

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Servidor LED escuchando en el puerto {UDP_PORT} por el Pin GPIO 10...")

try:
    while True:
        data, addr = sock.recvfrom(1024) 
        intensidad = float(data.decode('utf-8'))
        
        brillo_final = intensidad * MAX_BRIGHTNESS
        pixels.brightness = float(brillo_final)
        pixels.show()
except KeyboardInterrupt:
    print("\nCerrando servidor LED...")
finally:
    pixels.fill((0, 0, 0))
    pixels.show()
    sock.close()
