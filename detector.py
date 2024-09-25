import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Define color ranges for HSV
lower_range_az = np.array([69, 5, 0])
upper_range_az = np.array([156, 94, 77])

lower_range_am = np.array([0, 86, 0])
upper_range_am = np.array([28, 157, 175])

lower_range_vrd = np.array([0, 56, 0])
upper_range_vrd = np.array([112, 129, 50])

lower_range_vrm = np.array([26, 0, 101])
upper_range_vrm = np.array([255, 37, 255])

lower_range_br = np.array([81, 58, 17])
upper_range_br = np.array([103, 176, 149])

lower_range_lar = np.array([0, 0, 0])
upper_range_lar = np.array([15, 116, 255])

# Função para equalizar a imagem
def hisEqulColor(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    return img

# Função para aplicar erode e dilate
def process_mask(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)
    return mask

# Funções para cada cor com erode e dilate
def azul(frame):
    mask = cv2.inRange(frame, lower_range_az, upper_range_az)
    mask = process_mask(mask)
    _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = []
    for c in cnts:
        if cv2.contourArea(c) > 600:
            x, y, w, h = cv2.boundingRect(c)
            a.append(x)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 1)
    cv2.putText(frame, ("AZUL " + str(len(a))), (10, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

def verde(frame):
    mask = cv2.inRange(frame, lower_range_vrd, upper_range_vrd)
    mask = process_mask(mask)
    _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = []
    for c in cnts:
        if cv2.contourArea(c) > 600:
            x, y, w, h = cv2.boundingRect(c)
            a.append(x)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
    cv2.putText(frame, ("VERDE " + str(len(a))), (10, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

def vermelho(frame):
    mask = cv2.inRange(frame, lower_range_vrm, upper_range_vrm)
    mask = process_mask(mask)
    _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = []
    for c in cnts:
        if cv2.contourArea(c) > 600:
            x, y, w, h = cv2.boundingRect(c)
            a.append(x)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 1)
    cv2.putText(frame, ("VERMELHO " + str(len(a))), (10, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

def amarelo(frame):
    mask = cv2.inRange(frame, lower_range_am, upper_range_am)
    mask = process_mask(mask)
    _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = []
    for c in cnts:
        if cv2.contourArea(c) > 600:
            x, y, w, h = cv2.boundingRect(c)
            a.append(x)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
    cv2.putText(frame, ("AMARELO " + str(len(a))), (10, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

def branco(frame):
    mask = cv2.inRange(frame, lower_range_br, upper_range_br)
    mask = process_mask(mask)
    _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = []
    for c in cnts:
        if cv2.contourArea(c) > 600:
            x, y, w, h = cv2.boundingRect(c)
            a.append(x)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
    cv2.putText(frame, ("BRANCO " + str(len(a))), (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

def laranja(frame):
    mask = cv2.inRange(frame, lower_range_lar, upper_range_lar)
    mask = process_mask(mask)
    _, mask1 = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = []
    for c in cnts:
        if cv2.contourArea(c) > 600:
            x, y, w, h = cv2.boundingRect(c)
            a.append(x)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 128, 255), 2)
    cv2.putText(frame, ("LARANJA " + str(len(a))), (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    frame = hisEqulColor(frame)
    
    azul(frame)
    amarelo(frame)
    verde(frame)
    vermelho(frame)
    laranja(frame)
    branco(frame)
    
    cv2.imshow("FRAME", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()