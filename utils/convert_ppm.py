from PIL import Image
import time

nm = str(int(time.time()))

im = Image.open("imagem.ppm")
im.save(f"./renders/{nm}.jpg")
