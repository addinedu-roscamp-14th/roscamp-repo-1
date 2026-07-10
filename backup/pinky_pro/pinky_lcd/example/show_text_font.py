from pinky_lcd import LCD
from PIL import Image, ImageSequence, ImageDraw, ImageFont
import os, time

lcd = LCD()

# lcd 밝기 설정
lcd.set_backlight(50)

img_width, img_height = 320, 240
background_color = (255, 255, 255)
text_color = (0, 255, 0)

text = "주제\n뭐함"

img = Image.new('RGB', (img_width, img_height), color=background_color)
draw = ImageDraw.Draw(img)

try:
    font = ImageFont.truetype("MaruBuri-Bold.ttf", 60) # 글자크기 30으로 설정
except:
    font = ImageFont.load_default()

# 이미지 가운데 좌표 계산 후 출력
bbox = draw.textbbox((0, 0), text, font=font)
text_width = bbox[2] - bbox[0]
text_height = bbox[3] - bbox[1]
x = (img_width - text_width) // 2
y = (img_height - text_height) // 2

draw.text((x, y), text, fill=text_color, font=font)

lcd.img_show(img)
time.sleep(30)

lcd.close()
