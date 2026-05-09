import os
import random
import serial
import pygame
from pyPS4Controller.controller import Controller

import time  # <- agregar arriba

# =========================
# 1. Configuración
# =========================

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
JOYSTICK_INTERFACE = "/dev/input/js0"

AUDIO_FOLDERS = {
    "square": "/home/papa/Desktop/audios/chicos",
    "triangle": "/home/papa/Desktop/audios/permiso",
    "circle": "/home/papa/Desktop/audios/sarcastico",
    "cross": "/home/papa/Desktop/audios/vuelta",
    "startup": "/home/papa/Desktop/audios/encendido",
}

VALID_EXTENSIONS = (".mp3", ".wav", ".ogg")
VOLUME = 1  


# =========================
# 2. Inicializar audio
# =========================
pygame.mixer.init()
pygame.mixer.music.set_volume(VOLUME)
print("Audio system initialized")


# =========================
# 3. Conexión serie
# =========================
try:
    mcu = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print("Connected to Microcontroller via USB!")
except serial.SerialException as e:
    print(f"Failed to connect to microcontroller: {e}")
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


def play_random_audio(folder_path, stop_current=True):
    if not os.path.isdir(folder_path):
        print(f"Folder does not exist: {folder_path}")
        return

    files = [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.lower().endswith(VALID_EXTENSIONS)
    ]

    if not files:
        print(f"No audio files found in: {folder_path}")
        return

    selected_file = random.choice(files)
    print(f"Playing: {selected_file}")

    try:
        if stop_current:
            pygame.mixer.music.stop()
            time.sleep(0.2)  # 🔥 delay clave (200 ms)

        pygame.mixer.music.load(selected_file)
        pygame.mixer.music.play()

    except Exception as e:
        print(f"Error playing audio: {e}")

# =========================
# 5. Clase del control DS4
# =========================
class DS4Router(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # -------- BOTONES DE AUDIO --------
    def on_x_press(self):
        print("X pressed")
        play_random_audio(AUDIO_FOLDERS["cross"])
        send_serial("BTN,CROSS,1")

    def on_x_release(self):
        send_serial("BTN,CROSS,0")

    def on_square_press(self):
        print("Square pressed")
        play_random_audio(AUDIO_FOLDERS["square"])

    def on_triangle_press(self):
        print("Triangle pressed")
        play_random_audio(AUDIO_FOLDERS["triangle"])

    def on_circle_press(self):
        print("Circle pressed")
        play_random_audio(AUDIO_FOLDERS["circle"])

    # -------- CRUCETA / DIRECCIONES --------
    def on_up_arrow_press(self):
        print("UP pressed")
        send_serial("DIR,UP,1")

    def on_up_down_arrow_release(self):
        print("UP/DOWN released")
        send_serial("DIR,UP,0")
        send_serial("DIR,DOWN,0")

    def on_down_arrow_press(self):
        print("DOWN pressed")
        send_serial("DIR,DOWN,1")

    def on_left_arrow_press(self):
        print("LEFT pressed")
        send_serial("DIR,LEFT,1")

    def on_left_right_arrow_release(self):
        print("LEFT/RIGHT released")
        send_serial("DIR,LEFT,0")
        send_serial("DIR,RIGHT,0")

    def on_right_arrow_press(self):
        print("RIGHT pressed")
        send_serial("DIR,RIGHT,1")

    # -------- STICK EJEMPLO --------
    def on_L3_up(self, value):
        command = f"JOY,L_UP,{abs(value)}"
        send_serial(command)

    def on_options_press(self):
        print("Stopping audio")
        pygame.mixer.music.stop()


# =========================
# 6. Main
# =========================
if __name__ == "__main__":
    print("Reproduciendo audio de inicio...")
    play_random_audio(AUDIO_FOLDERS["startup"])

    print("Waiting for DS4 input...")
    router = DS4Router(
        interface=JOYSTICK_INTERFACE,
        connecting_using_ds4drv=False
    )
    router.listen()