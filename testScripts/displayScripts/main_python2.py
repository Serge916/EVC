import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
from time import sleep

# Initialize the display (set the correct I2C address if necessary)
disp = Adafruit_SSD1306.SSD1306_128_64(rst=None)

# Initialize the library
disp.begin()

# Clear the display
disp.clear()
disp.display()

# Create a blank image for drawing
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get a drawing object to draw on the image
draw = ImageDraw.Draw(image)

# Load a font (adjust path and size as needed)
# For a built-in font, you can use ImageFont.load_default()
font = ImageFont.load_default()

# Draw static text
draw.rectangle((0, 0, width, height), outline=0, fill=0)  # Clear the image
draw.text((10, 5), "Battery: ", font=font, fill=255)
draw.text((10, 15), "Connection: ", font=font, fill=255)
draw.text((10, 25), "ToF: ", font=font, fill=255)

# Display the image
disp.image(image)
disp.display()

# Refresh the display every 5 seconds
while True:
    sleep(5)
