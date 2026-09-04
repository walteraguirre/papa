import serial
import math
import time

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


# ============================================================
# CONFIGURACIÓN
# ============================================================

SERIAL_PORT = "COM11"
BAUDRATE = 115200

TICKS_PER_REV = 36.0
WHEEL_RADIUS = 0.08
WHEEL_SEPARATION = 0.42


# ============================================================
# VARIABLES DE ODOMETRÍA
# ============================================================

x = 0.0
y = 0.0
yaw = 0.0

last_enc1 = None
last_enc2 = None

trayectoria_x = [0.0]
trayectoria_y = [0.0]


# ============================================================
# PUERTO SERIAL
# ============================================================

ser = serial.Serial(
    SERIAL_PORT,
    BAUDRATE,
    timeout=0
)

time.sleep(2)

print("Conectado a:", SERIAL_PORT)
print("Esperando datos...")


# ============================================================
# INTERFAZ PYQTGRAPH
# ============================================================

app = QtWidgets.QApplication([])

win = pg.GraphicsLayoutWidget(
    show=True,
    title="Odometría por encoders"
)

win.resize(800, 700)

plot = win.addPlot(
    title="Trayectoria estimada"
)

plot.setLabel("bottom", "X", units="m")
plot.setLabel("left", "Y", units="m")

plot.showGrid(x=True, y=True)

# Mantener misma escala en X e Y
plot.setAspectLocked(True)

curve = plot.plot(
    trayectoria_x,
    trayectoria_y,
    pen=pg.mkPen(width=2)
)

# Punto actual del robot
robot_point = plot.plot(
    [0],
    [0],
    pen=None,
    symbol="o",
    symbolSize=10
)


# ============================================================
# PROCESAMIENTO SERIAL
# ============================================================

def actualizar():

    global x, y, yaw
    global last_enc1, last_enc2

    # Leer todas las líneas disponibles
    while ser.in_waiting:

        try:

            line = ser.readline().decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not line:
                continue

            data = line.split(",")

            if len(data) != 8:
                continue

            # Trama:
            # distL,distC,distR,gyroX,gyroY,gyroZ,enc1,enc2

            enc1 = int(data[6])
            enc2 = int(data[7])

        except:
            continue


        # ====================================================
        # PRIMERA MEDICIÓN
        # ====================================================

        if last_enc1 is None:

            last_enc1 = enc1
            last_enc2 = enc2

            print("Referencia inicial:")
            print(f"ENC1 = {enc1}")
            print(f"ENC2 = {enc2}")

            continue


        # ====================================================
        # DIFERENCIA DE TICKS
        # ====================================================

        d_enc_left = enc1 - last_enc1
        d_enc_right = enc2 - last_enc2

        last_enc1 = enc1
        last_enc2 = enc2


        # ====================================================
        # TICKS -> ÁNGULO
        # ====================================================

        dtheta_left = (
            2.0 * math.pi / TICKS_PER_REV
        ) * d_enc_left

        dtheta_right = (
            2.0 * math.pi / TICKS_PER_REV
        ) * d_enc_right


        # ====================================================
        # ÁNGULO -> DISTANCIA
        # ====================================================

        ds_left = WHEEL_RADIUS * dtheta_left
        ds_right = WHEEL_RADIUS * dtheta_right


        # ====================================================
        # ODOMETRÍA DIFERENCIAL
        # ====================================================

        ds = (ds_right + ds_left) / 2.0

        dtheta_robot = (
            ds_right - ds_left
        ) / WHEEL_SEPARATION


        theta_mid = yaw + dtheta_robot / 2.0


        x += ds * math.cos(theta_mid)
        y += ds * math.sin(theta_mid)

        yaw += dtheta_robot


        # Normalización [-pi, pi]

        yaw = math.atan2(
            math.sin(yaw),
            math.cos(yaw)
        )


        # ====================================================
        # GUARDAR TRAYECTORIA
        # ====================================================

        trayectoria_x.append(x)
        trayectoria_y.append(y)


        # ====================================================
        # CONSOLA
        # ====================================================

        print(
            f"ENC1: {enc1:6d}   "
            f"ENC2: {enc2:6d}   |   "
            f"dL: {d_enc_left:3d}   "
            f"dR: {d_enc_right:3d}   |   "
            f"x: {x:+.3f} m   "
            f"y: {y:+.3f} m   "
            f"yaw: {math.degrees(yaw):+.1f}°"
        )


    # ========================================================
    # ACTUALIZAR GRÁFICO
    # ========================================================

    curve.setData(
        trayectoria_x,
        trayectoria_y
    )

    robot_point.setData(
        [x],
        [y]
    )


# ============================================================
# TIMER DE LA GUI
# ============================================================

timer = QtCore.QTimer()

timer.timeout.connect(actualizar)

# 50 ms = gráfico a aproximadamente 20 FPS
timer.start(50)


# ============================================================
# EJECUTAR
# ============================================================

try:

    app.exec_()

finally:

    ser.close()

    print("Puerto serial cerrado.")