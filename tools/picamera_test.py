import os
import time
import cv2
from picamera2 import Picamera2

# Use Picamera2 to capture frames and display with OpenCV
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "XRGB8888"})
picam2.configure(config)
picam2.start()

headless = not bool(os.environ.get("DISPLAY"))

def show_img():
    for _ in range(10):
        frame = picam2.capture_array()
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.001)

def save_img():
    for _ in range(10):
        frame = picam2.capture_array()
        cv2.imwrite("test_frame.jpg", frame)
        print("Wrote test_frame.jpg")
        time.sleep(0.5)

try:
    if headless:
        save_img()
    else:
        try:
            show_img()
        except:
            save_img()
finally:
    picam2.stop()
    if not headless:
        cv2.destroyAllWindows()
