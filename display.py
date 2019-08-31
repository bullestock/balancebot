import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used

class Display:
    def __init__(self):
        # 128x32 display with hardware I2C:
        self.disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)
        # Initialize library.
        self.disp.begin()
        self.clear()
        # Load default font.
        self.font = ImageFont.load_default()
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))
        self.padding = -2
        self.lineheight = 8
        self.top = self.padding
        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)

    def clear(self):
        # Clear display.
        self.disp.clear()
        self.disp.display()
        
    def show(self, line, text):
        self.draw.rectangle((0, line*self.lineheight, self.width, (line + 1)*self.lineheight), outline=0, fill=0)
        self.draw.text((0, self.padding + line*self.lineheight), text, font=self.font, fill=255)
        self.disp.image(self.image)
        self.disp.display()
