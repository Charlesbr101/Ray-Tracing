from sys import argv
from PIL import Image
import time

nm = str(int(time.time()))

im = Image.open(argv[1])
im.save(f"./renders/{argv[1].split('.')[0]}_{nm}.jpg")
