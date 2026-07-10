from pinky_lcd import LCD
from PIL import Image
import time

lcd = LCD()

img_width, img_height = 320, 240
background_color = (0, 255, 0)
text_color = (0, 255, 0)

img = Image.new('RGB', (img_width, img_height), color=background_color)

lcd.img_show(img)

time.sleep(3)

lcd.close()
