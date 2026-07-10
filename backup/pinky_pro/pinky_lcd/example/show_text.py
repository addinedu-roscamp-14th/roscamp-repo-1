from pinky_lcd import LCD
from PIL import Image, ImageSequence, ImageDraw, ImageFont
import os, time

lcd = LCD()

#이미지 크기와 색 설정 
img_width, img_height = 320, 240
background_color = (0, 0, 0)
text_color = (255, 255, 255)
text = "hello\npinky!!"

img = Image.new('RGB', (img_width, img_height), color=background_color)
draw = ImageDraw.Draw(img)

font = ImageFont.load_default()

# 이미지 가운데 좌표 계산 후 출력
bbox = draw.textbbox((0, 0), text, font=font)
text_width = bbox[2] - bbox[0]
text_height = bbox[3] - bbox[1]
x = (img_width - text_width) // 2
y = (img_height - text_height) // 2

draw.text((x, y), text, fill=text_color, font=font)

lcd.img_show(img)
time.sleep(3)

lcd.close()
