import os
import random
import serial
import pygame
import time

# =========================
# 1. Configuración
# =========================

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200


#lista_voces = ["glados", 

AUDIO_FOLDERS = {
    "square": "/home/papa/Desktop/audios/brainrots/glados/chicos",
    "triangle": "/home/papa/Desktop/audios/brainrots/glados/permiso",
    "circle": "/home/papa/Desktop/audios/brainrots/gladossarcastico",
    "cross": "/home/papa/Desktop/audios/brainrots/gladosvuelta",
    "startup": "/home/papa/Desktop/audios/brainrots/glados/encendido",
}

VALID_EXTENSIONS = (".mp3", ".wav", ".ogg")
VOLUME = 0.9

POLL_INTERVAL = 10  # 🔥 cada cuánto consulta (100 ms)

# =========================
# 2. Inicializar pygame
# =========================
pygame.init()
pygame.mixer.init()
pygame.mixer.music.set_volume(VOLUME)

pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick conectado:", joystick.get_name())
print("Audio system initialized")

# =========================
# 3. Conexión serie
# =========================
try:
    mcu = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print("Connected to Microcontroller via USB!")
except serial.SerialException as e:
    print(f"Failed to connect: {e}")
    mcu = None

# =========================
# 4. Utilidades
# =========================
def send_serial(message):
    if mcu:
        try:
            mcu.write((message + "\n").encode("utf-8"))
            print(f"Sent: {message}")
        except Exception as e:
            print(f"Serial error: {e}")


def play_random_audio(folder_path):
    if not os.path.isdir(folder_path):
        return
    
    
    files = [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.lower().endswith(VALID_EXTENSIONS)
    ]

    if not files:
        return

    selected_file = random.choice(files)
    print(f"Playing: {selected_file}")

    pygame.mixer.music.stop()
    time.sleep(0.1)
    pygame.mixer.music.load(selected_file)
    pygame.mixer.music.play()


# =========================
# 5. Estado previo (anti-spam)
# =========================
prev_buttons = {}
prev_hat = (0, 0)

# =========================
# 6. Main (polling loop)
# =========================
if __name__ == "__main__":
    print("Reproduciendo audio de inicio...")
    play_random_audio(AUDIO_FOLDERS["startup"])

    print("Polling joystick...")

    while True:
        pygame.event.pump()  # 🔥 actualizar estados

        # =========================
        # BOTONES
        # =========================
        # Mapeo típico DS4 en pygame:
        # 0: X, 1: Circle, 2: Square, 3: Triangle

        buttons = {
            "x": joystick.get_button(0),
            "circle": joystick.get_button(1),
            "square": joystick.get_button(2),
            "triangle": joystick.get_button(3),
        }

        # Detectar cambios (flanco)
        for key, value in buttons.items():
            prev = prev_buttons.get(key, 0)

            if value == 1 and prev == 0:  # PRESSED
                print(f"{key.upper()} pressed")

                if key == "x":
                    play_random_audio(AUDIO_FOLDERS["cross"])
                    send_serial("BTN,CROSS,1")

                elif key == "square":
                    play_random_audio(AUDIO_FOLDERS["square"])

                elif key == "triangle":
                    play_random_audio(AUDIO_FOLDERS["triangle"])

                elif key == "circle":
                    play_random_audio(AUDIO_FOLDERS["circle"])

            if value == 0 and prev == 1:  # RELEASE
                if key == "x":
                    send_serial("BTN,CROSS,0")

            prev_buttons[key] = value

        # =========================
        # CRUCETA (HAT)
        # =========================
        hat = joystick.get_hat(0)

        if hat != prev_hat:
            print("HAT:", hat)

            # reset
            send_serial("DIR,UP,0")
            send_serial("DIR,DOWN,0")
            send_serial("DIR,LEFT,0")
            send_serial("DIR,RIGHT,0")

            if hat == (0, 1):
                send_serial("DIR,UP,1")

            elif hat == (0, -1):
                send_serial("DIR,DOWN,1")

            elif hat == (-1, 0):
                send_serial("DIR,LEFT,1")

            elif hat == (1, 0):
                send_serial("DIR,RIGHT,1")

            prev_hat = hat

        # =========================
        # DELAY (clave)
        # =========================
        time.sleep(POLL_INTERVAL)
