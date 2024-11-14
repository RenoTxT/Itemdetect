import RPi.GPIO as GPIO
import time
import cv2
from ultralytics import YOLO

# Menonaktifkan peringatan GPIO
GPIO.setwarnings(False)

# Setup GPIO
pin_laser = 22
pin_servo = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_laser, GPIO.OUT)
GPIO.setup(pin_servo, GPIO.OUT)

# Load model YOLOv8
model = YOLO('best.pt')

# Coba untuk membuka kamera dengan indeks 0 atau 1
cap = cv2.VideoCapture(0)  # Ubah ke 1 jika 0 tidak bekerja

if not cap.isOpened():
    print("Kamera tidak ditemukan atau tidak dapat dibuka.")
    exit()
    
print("Model classes:", model.names)
    

def deteksi(frame):
    results = model(frame)
    bird_detected = False

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = box.conf[0]
            label = model.names[class_id]

            if label == "Bird" and confidence > 0.5:
                bird_detected = True
                break

    if bird_detected:
        GPIO.output(pin_laser, GPIO.HIGH)
        GPIO.output(pin_servo, GPIO.HIGH)
        print("Ada burung tuh, burung ape? tembak je lah, hidup laser berhenti dulu servo")
    else:
        GPIO.output(pin_laser, GPIO.LOW)
        GPIO.output(pin_servo, GPIO.LOW)
        print("Burung tidak terdeteksi, laser dimatikan dan servo lanjut muter kanggg negkolkan")
        
    

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        deteksi(frame)  # Lakukan deteksi burung tanpa menampilkan frame

        # Program berjalan tanpa menampilkan gambar

except KeyboardInterrupt:
    pass

# Lepaskan video capture dan bersihkan GPIO
cap.release()
GPIO.cleanup()

