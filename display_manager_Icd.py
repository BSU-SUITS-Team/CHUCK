import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

class DisplayManagerLCD:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.display = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

        self.width = self.display.width
        self.height = self.display.height

        self.image = Image.new("1", (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)

        self.font = ImageFont.load_default()

    def update(self, page_index):
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        if page_index == 0:
            text = "Temp: 72\nHR: 80"
        elif page_index == 1:
            text = "CPU: 40%\nRAM: 60%"
        else:
            text = "No Data"

        self.draw.text((0, 0), f"Page {page_index}", font=self.font, fill=255)
        self.draw.text((0, 20), text, font=self.font, fill=255)

        self.display.image(self.image)
        self.display.show()