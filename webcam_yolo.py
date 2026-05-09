import cv2
from ultralytics import YOLO

# Modelo liviano
model = YOLO("yolov8n.pt")

# Webcam de la notebook
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("No se pudo abrir la webcam")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("No se pudo leer frame")
        break

    # Bajar resolución para hacerlo más liviano
    frame = cv2.resize(frame, (640, 480))

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
                    0.7,
                    (0, 255, 0),
                    2
                )

    cv2.imshow("Deteccion de personas - Webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()