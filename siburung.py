import RPi.GPIO as GPIO
import time
import cv2
from ultralytics import YOLO

pin_servo1 = 27 # pin GPIO servo 1
pin_laser = 22 # pin GPIO laser

freq1 = 50 # frequensi servo 1

duty_cycle = 0 # memulai duty cycle di 0

# setting dulu disini
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_servo1, GPIO.OUT)
GPIO.setup(pin_laser, GPIO.OUT)

servo1_pwm = GPIO.PWM(pin_servo1, freq1)
servo1_pwm.start(duty_cycle)

def patroli(angle):
    duty = 2 + (angle / 18)
    GPIO.output(pin_servo1, True)
    servo1_pwm.ChangeDutyCycle(duty)
    time.sleep(0.02)
    GPIO.output(pin_servo1, False)
    servo1_pwm.ChangeDutyCycle(0)

def detect_bird(frame):
    global bird_detected  # Menandakan bahwa bird_detected adalah variabel global

    # Deteksi objek menggunakan YOLOv8
    results = model(frame)
    bird_detected = False  # Reset status bird_detected sebelum pengecekan

    # Cek apakah burung terdeteksi
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = box.conf[0]
            label = model.names[class_id]

            if label == "bird" and confidence > 0.5:
                bird_detected = True
                break

    # Update status laser berdasarkan deteksi burung
    if bird_detected:
        GPIO.output(pin_laser, GPIO.HIGH)
        print("Burung terdeteksi! Laser diaktifkan.")
    else:
        GPIO.output(pin_laser, GPIO.LOW)
    
# Load model YOLOv8
model = YOLO('best.pt')

# Menangkap kamera
cap = cv2.VideoCapture(0) # "0" = default camera

bird_detected = False

try:
    while True:
        for angle in range(0, 181, 10): 
            servo1_pwm(angle)

            ret, frame = cap.read()
            if not ret:
                break

            detect_bird(frame) # tembak si burung

        for angle in range(180, -1, -10): 
            servo1_pwm(angle)

            ret, frame = cap.read()
            if not ret:
                break

            detect_bird(frame) # tembak si burung

except KeyboardInterrupt:
    pass

# Bersihkan setelah selesai
cap.release()
servo1_pwm.stop()
GPIO.cleanup()