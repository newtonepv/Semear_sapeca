import cv2
import numpy as np

capCamera = cv2.VideoCapture(0)

if not capCamera.isOpened():
    print("nao abriu")
    exit()

while True:

    suc, img = capCamera.read()

    if(suc):
        
        cv2.imshow('Camaera',img)
    
    if(cv2.waitKey(1)&0xFF==ord('q')):
        break

capCamera.release()
cv2.destroyAllWindows()