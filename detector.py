import cv2 as cv
import numpy as np
import cv2 as cv

def get_limits(color):
    """
    Ajusta os limites HSV para a cor fornecida com base na imagem capturada.
    """
    c = np.uint8([[color]])
    hsvC = cv.cvtColor(c, cv.COLOR_BGR2HSV)
    h, s, v = hsvC[0][0]

    if color == (0, 255, 255):  # Amarelo
        lower_limit = np.array([h - 20, 100, 100], dtype=np.uint8)
        upper_limit = np.array([h + 20, 255, 255], dtype=np.uint8)
    elif color == (0, 0, 255):  # Vermelho
        lower_limit2 = np.array([160, 100, 100], dtype=np.uint8)
        upper_limit2 = np.array([180, 255, 255], dtype=np.uint8)
        return lower_limit2, upper_limit2
    elif color == (0, 165, 255):  # Laranja
        lower_limit = np.array([5, 100, 100], dtype=np.uint8)
        upper_limit = np.array([15, 255, 255], dtype=np.uint8)
    elif color == (0, 255, 0):  # Verde
        lower_limit = np.array([35, 50, 50], dtype=np.uint8)
        upper_limit = np.array([85, 255, 255], dtype=np.uint8)
    elif color == (255, 0, 0):  # Azul
        lower_limit = np.array([100, 100, 100], dtype=np.uint8)
        upper_limit = np.array([140, 255, 255], dtype=np.uint8)
    elif color == (255, 255, 255):  # Branco
        lower_limit = np.array([0,0,0], dtype=np.uint8)
        upper_limit = np.array([200,0,150], dtype=np.uint8)
    
    return lower_limit, upper_limit


def get_limits(color):
    """
    Ajusta os limites HSV para a cor fornecida com base na imagem capturada.
    """
    c = np.uint8([[color]])
    hsvC = cv.cvtColor(c, cv.COLOR_BGR2HSV)
    h, s, v = hsvC[0][0]

    if color == (0, 255, 255):  # Amarelo
        lower_limit = np.array([h - 20, 100, 100], dtype=np.uint8)
        upper_limit = np.array([h + 20, 255, 255], dtype=np.uint8)
    elif color == (0, 0, 255):  # Vermelho
        lower_limit2 = np.array([160, 100, 100], dtype=np.uint8)
        upper_limit2 = np.array([180, 255, 255], dtype=np.uint8)
        return lower_limit2, upper_limit2
    elif color == (0, 165, 255):  # Laranja
        lower_limit = np.array([5, 100, 100], dtype=np.uint8)
        upper_limit = np.array([15, 255, 255], dtype=np.uint8)
    elif color == (0, 255, 0):  # Verde
        lower_limit = np.array([35, 50, 50], dtype=np.uint8)
        upper_limit = np.array([85, 255, 255], dtype=np.uint8)
    elif color == (255, 0, 0):  # Azul
        lower_limit = np.array([100, 100, 100], dtype=np.uint8)
        upper_limit = np.array([140, 255, 255], dtype=np.uint8)
    elif color == (255, 255, 255):  # Branco
        lower_limit = np.array([0,0,0], dtype=np.uint8)
        upper_limit = np.array([200,0,150], dtype=np.uint8)
    
    return lower_limit, upper_limit

def process_color(color_name, color, hsv, frame):
    """
    Processa a detecção de uma cor específica e desenha as caixas delimitadoras.
    """
    lower_bound, upper_bound = get_limits(color)
    mask = cv.inRange(hsv, lower_bound, upper_bound)

    
    result = cv.bitwise_and(frame, frame, mask=mask)
    gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 100, 200)
    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        if cv.contourArea(contour) > 200:
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv.circle(frame, (x + w // 2, y + h // 2), 3, (0, 255, 0), -1)
            cv.putText(frame, color_name, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return result

def main():
    # Inicializa a captura de vídeo
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Erro ao abrir a câmera.")
        return
    
    colors = {
        'white': (255, 255, 255),
        'yellow': (0, 255, 255),
        'red': (0, 0, 255),
        'orange': (0, 165, 255),
        'green': (0, 255, 0),
        'blue': (255, 0, 0)
    }
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro ao capturar imagem.")
            break
        
        # Normalização do histograma para reduzir o impacto da iluminação
        lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
        l, a, b = cv.split(lab)
        l = cv.equalizeHist(l)
        lab = cv.merge((l, a, b))
        frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)

        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        combined_result = np.zeros_like(frame)
        
        for color_name, color in colors.items():
            result = process_color(color_name, color, hsv, frame)
            combined_result = cv.add(combined_result, result)
        combined_result = cv.GaussianBlur(combined_result, (3, 3), 1)
        cv.imshow('Combined Result', combined_result)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()


def main():
    # Inicializa a captura de vídeo
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Erro ao abrir a câmera.")
        return
    
    colors = {
        'white': (255, 255, 255),
        'yellow': (0, 255, 255),
        'red': (0, 0, 255),
        'orange': (0, 165, 255),
        'green': (0, 255, 0),
        'blue': (255, 0, 0)
    }
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro ao capturar imagem.")
            break
        
        # Normalização do histograma para reduzir o impacto da iluminação
        lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
        l, a, b = cv.split(lab)
        l = cv.equalizeHist(l)
        lab = cv.merge((l, a, b))
        frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)

        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        combined_result = np.zeros_like(frame)
        
        for color_name, color in colors.items():
            result = process_color(color_name, color, hsv, frame)
            combined_result = cv.add(combined_result, result)
        combined_result = cv.GaussianBlur(combined_result, (21, 21), 10)
        cv.imshow('Combined Result', combined_result)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
