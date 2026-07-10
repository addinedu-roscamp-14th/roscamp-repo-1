from picamera2 import Picamera2
import cv2
from pinky_lcd import LCD
from PIL import Image
import time

lcd = LCD()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

while True:
    frame = picam2.capture_array()
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rotate_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
    flipped_frame = cv2.flip(rotate_frame, 1)
    
    show_frame = Image.fromarray(flipped_frame)

    lcd.img_show(show_frame)

    cv2.waitKey(1)

picam2.close()
