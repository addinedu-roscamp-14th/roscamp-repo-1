from pinky_lcd import LCD
from PIL import Image
import time

lcd = LCD()

img = Image.open("example.jpg")

lcd.img_show(img)
time.sleep(3)

print("Play ended.")

lcd.close()
