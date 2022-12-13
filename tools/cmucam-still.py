import serial
import numpy as np
from PIL import Image

cmucam = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
if cmucam.inWaiting() > 0:
    cmucam.readall()

def find_shell():
    cmucam.read_until(expected=b':')

def take_picture():
    image = np.zeros((143,80,3), dtype=np.uint8)
    cmucam.write(b'DF\0')
    x = 0
    y = 0
    c = 0
    while True:
        b = cmucam.read()[0]
        if b == 1:
            x = 0
            y = 0
            c = 0
            print("Column: 0")
        elif b == 2:
            x = x + 1
            y = 0
            c = 0
            print("Column: ", x)
        elif b == 3:
            print("Done")
            find_shell()
            return np.flip(image, 0)
        else:
            if c == 3:
                c = 0
                y = y + 1
            image[y,x,c] = np.uint8(b)
            c = c + 1

# Start the camera
cmucam.write(b'\r')
find_shell()

# Enter raw mode, throw out ACK
cmucam.write(b'RM 7\r')
cmucam.read(1)

# Take the picture
frame = take_picture()
image = Image.fromarray(frame, 'RGB')
image.show()

# Exit raw mode
cmucam.write(b'RM\1\0')
find_shell()
