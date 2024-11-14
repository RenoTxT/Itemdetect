import RPi.GPIO as GPIO
import time
import cv2
from ultralytics import YOLO

pin_laser = 22
pin_servo = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_laser, GPIO.OUT)
GPIO.setup(pin_servo, GPIO.OUT)

model = YOLO('best.pt')

cap = cv2.VideoCapture(0)

def deteksi(frame):
    results = model(frame)
    bird_detected = False

    for result in results:
        for box in results.boxes:
            class_id = int(box.cls[0])
            confidence = box.conf[0]
            label = model.names[class_id]

            if label == "bird" and confidence > 0.5:
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

        deteksi(frame)

        cv2.imshow('Camera', frame)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

except KeyboardInterrupt:
    pass

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()
