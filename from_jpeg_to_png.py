import cv2,os
import numpy as np
from PIL import Image
def desenhar(image, x1,y1,x2,y2, cor, espessura):
    cv2.rectangle(image, (x1,y1),(x2,y2), cor, espessura)

xc=250
yc=260
width=320
height=300

black_threshold = 50

i=0



for caminho in os.listdir("jpeg_images"):
    caminho = 'jpeg_images/'+caminho

    img = cv2.imread(caminho)

    img_rgba = cv2.cvtColor(img, cv2.COLOR_RGB2RGBA)

    img_rgba[np.all(img_rgba[:, :, :3] < black_threshold, axis=-1)] = [0, 0, 0, 0]

    nome='png_images/'+str(i)+'.png'
    cv2.imwrite(nome,img_rgba)

    i=i+1