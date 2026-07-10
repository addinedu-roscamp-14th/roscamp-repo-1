from pinky_lcd import LCD
from PIL import Image, ImageSequence, ImageDraw, ImageFont
import os, time

lcd = LCD()

img_width, img_height = 320, 240
background_color = (0, 0, 0)
text_color = (255, 255, 255)
text = ""

try:
    font = ImageFont.truetype("MaruBuri-Bold.ttf", 30) # 글자크기 30으로 설정
except:
    font = ImageFont.load_default()

# q 입력 시 종료
while text != 'q':
    text = input()
    
    img = Image.new('RGB', (img_width, img_height), color=background_color)
    draw = ImageDraw.Draw(img)

    bbox = draw.textbbox((0, 0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    x = (img_width - text_width) // 2
    y = (img_height - text_height) // 2

    draw.text((x, y), text, fill=text_color, font=font, align="center")

    lcd.img_show(img)

lcd.close()
