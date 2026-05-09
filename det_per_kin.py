import cv2
import numpy as np
import freenect
from ultralytics import YOLO

# Modelo más liviano posible para Raspberry Pi 3
model = YOLO("yolov8n.pt")

def get_rgb_frame():
    """Captura frame RGB de la Kinect via libfreenect"""
    array, _ = freenect.sync_get_video()
    # Kinect devuelve RGB, OpenCV usa BGR
    frame_bgr = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return frame_bgr

def get_depth_frame():
    """Captura frame de profundidad (opcional, útil para filtrar detecciones)"""
    array, _ = freenect.sync_get_depth()
    # Normalizar para visualización (0-255)
    depth_normalized = cv2.normalize(array, None, 0, 255, cv2.NORM_MINMAX)
    return depth_normalized.astype(np.uint8)

print("Iniciando detección con Kinect...")
print("Presioná 'q' para salir | 'd' para toggle profundidad")

show_depth = False

while True:
    try:
        frame = get_rgb_frame()
    except Exception as e:
        print(f"Error al leer Kinect: {e}")
        break

    # Reducir resolución: Kinect nativa es 640x480, podés bajar más si va lento
    frame = cv2.resize(frame, (320, 240))

    # Inferencia YOLO
    results = model(frame, verbose=False, conf=0.5)

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            label = model.names[cls]

            if label == "person":
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    "Persona",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

    cv2.imshow("Deteccion de personas - Kinect", frame)

    # Mostrar profundidad si está activado
    if show_depth:
        depth = get_depth_frame()
        depth_resized = cv2.resize(depth, (320, 240))
        depth_colored = cv2.applyColorMap(depth_resized, cv2.COLORMAP_JET)
        cv2.imshow("Profundidad - Kinect", depth_colored)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("d"):
        show_depth = not show_depth
        if not show_depth:
            cv2.destroyWindow("Profundidad - Kinect")

# Liberar Kinect correctamente
freenect.sync_stop()
cv2.destroyAllWindows()
print("Programa finalizado")