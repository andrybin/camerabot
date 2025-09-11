import os
import cv2, time, sys
from picamera2 import Picamera2

# Use Picamera2 to capture frames and display with OpenCV
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "XRGB8888"})
picam2.configure(config)
picam2.start()

headless = not bool(os.environ.get("DISPLAY"))

try:
    if headless:
        print("Running in headless mode: writing frames to test_frame.jpg (Ctrl+C to stop)")
        for _ in range(10):
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            cv2.imwrite("test_frame.jpg", frame_bgr)
            print("Wrote test_frame.jpg")
            time.sleep(0.5)
    else:
        for _ in range(10):
            frame = picam2.capture_array()
            # XRGB8888 -> OpenCV expects BGR; drop alpha and convert RGB to BGR
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            cv2.imshow("Frame", frame_bgr)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            # time.sleep(0.001)
finally:
    picam2.stop()
    if not headless:
        cv2.destroyAllWindows()
