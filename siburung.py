import RPi.GPIO as GPIO
import time
import cv2
from ultralytics import YOLO

pin_servo1 = 27  # pin GPIO servo 1
pin_laser = 22   # pin GPIO laser

freq1 = 50        # frekuensi servo 1
duty_cycle = 0    # memulai duty cycle di 0

# Menonaktifkan peringatan GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup pin GPIO
GPIO.setup(pin_servo1, GPIO.OUT)
GPIO.setup(pin_laser, GPIO.OUT)

servo1_pwm = GPIO.PWM(pin_servo1, freq1)
servo1_pwm.start(duty_cycle)

# Fungsi untuk menggerakkan servo
def patroli(angle):
    duty = 2 + (angle / 18)  # Menghitung duty cycle untuk servo
    servo1_pwm.ChangeDutyCycle(duty)
    time.sleep(0.02)  # Waktu untuk menggerakkan servo
    servo1_pwm.ChangeDutyCycle(0)  # Matikan PWM

# Fungsi untuk mendeteksi burung
def detect_bird(frame):
    global bird_detected  # Variabel global untuk status deteksi burung

    # Deteksi objek menggunakan YOLOv8
    results = model(frame)
    bird_detected = False  # Reset status burung terdeteksi

    # Cek apakah burung terdeteksi
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = box.conf[0]
            label = model.names[class_id]

            if label == "bird" and confidence > 0.5:
                bird_detected = True
                break
        if bird_detected:
            tembak_laser()

def tembak_laser():
    # Aktifkan laser jika burung terdeteksi
    if bird_detected:
        GPIO.output(pin_laser, GPIO.HIGH)
        print("Burung terdeteksi! Laser diaktifkan.")
    else:
        GPIO.output(pin_laser, GPIO.LOW)

# Load model YOLOv8
model = YOLO('best.pt')

# Menangkap kamera
cap = cv2.VideoCapture(1)  # "0" untuk kamera default, coba ganti ke 1 jika perlu

bird_detected = False

try:
    while True:
        # Patroli horizontal dari 0 hingga 180 derajat
        for angle in range(0, 181, 10):  # Ubah langkah jika perlu
            patroli(angle)

            ret, frame = cap.read()
            if not ret:
                break  # Keluar jika kamera tidak dapat membaca frame
            
            detect_bird(frame)  # Deteksi burung pada setiap frame

        # Patroli horizontal dari 180 hingga 0 derajat
        for angle in range(180, -1, 10):  # Ubah langkah jika perlu
            patroli(angle)

            ret, frame = cap.read()
            if not ret:
                break  # Keluar jika kamera tidak dapat membaca frame

            #detect_bird(frame)  # Deteksi burung pada setiap frame

except KeyboardInterrupt:
    print("Program dihentikan")

finally:
    # Bersihkan setelah selesai
    cap.release()
    servo1_pwm.stop()
    GPIO.cleanup()

