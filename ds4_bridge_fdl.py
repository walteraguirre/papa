import os
import random
import serial
import pygame
import time
from pyPS4Controller.controller import Controller

# =========================
# 1. Configuración
# =========================

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
JOYSTICK_INTERFACE = "/dev/input/js0"

VOICES_ROOT = "/home/papa/Desktop/audios/voces"

VALID_EXTENSIONS = (".mp3", ".wav", ".ogg")
VOLUME = 1.0 # Puedes bajarlo a 0.8 si los audios originales son muy fuertes

# =========================
# 2. Inicializar audio (CORREGIDO PARA EVITAR SATURACIÓN)
# =========================

# Forzamos parámetros estándar para evitar problemas de memoria/saturación en la Raspberry Pi
pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=4096)
pygame.mixer.init()
pygame.mixer.music.set_volume(VOLUME)
print("Audio system initialized with custom buffer/frequency")

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
# 4. Sistema de voces
# =========================

def load_characters():
    if not os.path.isdir(VOICES_ROOT):
        print(f"Voices folder does not exist: {VOICES_ROOT}")
        return []

    characters = [
        name for name in os.listdir(VOICES_ROOT)
        if os.path.isdir(os.path.join(VOICES_ROOT, name))
    ]

    characters.sort()
    return characters

CHARACTERS = load_characters()
current_character_index = 0

def get_current_character():
    if not CHARACTERS:
        return None
    return CHARACTERS[current_character_index]

def get_audio_folder(category):
    character = get_current_character()
    if character is None:
        print("No characters found")
        return None
    return os.path.join(VOICES_ROOT, character, category)

def next_character():
    global current_character_index

    if not CHARACTERS:
        print("No characters available")
        return

    current_character_index = (current_character_index + 1) % len(CHARACTERS)
    character = get_current_character()

    print(f"Character changed to: {character}")

    folder = get_audio_folder("encendido")
    play_random_audio(folder)

# =========================
# 5. Utilidades
# =========================

def send_serial(message):
    if mcu:
        try:
            mcu.write((message + "\n").encode("utf-8"))
            print(f"Sent: {message}")
        except Exception as e:
            print(f"Serial error: {e}")

def play_random_audio(folder_path, stop_current=True):
    if folder_path is None:
        return

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
            time.sleep(0.1) # Reduje un poco el delay para mayor respuesta

        pygame.mixer.music.load(selected_file)
        # CORRECCIÓN: Quitamos el loops=-1 para que no se repita infinitamente
        pygame.mixer.music.play() 

    except Exception as e:
        print(f"Error playing audio: {e}")

# =========================
# 6. Clase del control DS4
# =========================

class DS4Router(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # -------- CAMBIAR DE MODO (NUEVO L1) --------
    def on_L1_release(self):
        print("L1 released - Alternando modo automático/manual")
        send_serial("on_L1_release")

    # -------- CAMBIAR PERSONAJE --------
    def on_R1_press(self):
        print("R1 pressed")
        next_character()

    # -------- BOTONES DE AUDIO --------
    def on_x_press(self):
        print("X pressed")
        play_random_audio(get_audio_folder("vuelta"))
        send_serial("BTN,CROSS,1")

    def on_x_release(self):
        send_serial("BTN,CROSS,0")

    def on_square_press(self):
        print("Square pressed")
        play_random_audio(get_audio_folder("chicos"))

    def on_triangle_press(self):
        print("Triangle pressed")
        play_random_audio(get_audio_folder("permiso"))

    def on_circle_press(self):
        print("Circle pressed")
        play_random_audio(get_audio_folder("sarcastico"))

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


# =========================
# 7. Main
# =========================

if __name__ == "__main__":
    if CHARACTERS:
        print("Characters found:")
        for character in CHARACTERS:
            print(f"- {character}")

        print(f"Current character: {get_current_character()}")
        print("Reproduciendo audio de inicio...")
        play_random_audio(get_audio_folder("encendido"))
    else:
        print("No characters found. Check folder structure.")

    print("Waiting for DS4 input...")

    router = DS4Router(
        interface=JOYSTICK_INTERFACE,
        connecting_using_ds4drv=False
    )

    router.listen()