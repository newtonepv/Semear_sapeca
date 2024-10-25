import cv2
import numpy as np
import subprocess
from collections import Counter

# Comando libcamera-vid para capturar vídeo e enviar para stdout como MJPEG
command = [
    "libcamera-vid",
    "-t", "0",  # Captura indefinida (0 milissegundos)
    "--inline",  # MJPEG embutido para leitura eficiente
    "--codec", "mjpeg",  # Formato MJPEG
    "--width", "640",  # Largura da imagem
    "--height", "480",  # Altura da imagem
    "--framerate", "30",  # FPS
    "-o", "-",  # Saída para stdout
]

# Inicializa o processo libcamera-vid
process = subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=10**8)

# Função para ler frames da câmera
def ler_camera():
    frame_data = b''
    while True:
        chunk = process.stdout.read(1024)
        if not chunk:
            break
        frame_data += chunk
        while b'\xff\xd9' in frame_data:
            end_index = frame_data.index(b'\xff\xd9') + 2
            jpeg_data = frame_data[:end_index]
            frame_data = frame_data[end_index:]
            yield jpeg_data

# Função vazia para usar nas trackbars
def nothing(x):
    pass

# Criação da janela para as trackbars
cv2.namedWindow("Trackbars")

# Trackbars para ajustar os limites HSV
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)  # Limite inferior Hue
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)  # Limite inferior Saturation
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)  # Limite inferior Value
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)  # Limite superior Hue
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)  # Limite superior Saturation
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)  # Limite superior Value

# Função para detectar cor com base na faixa HSV
def detect_color(frame, lower_range, upper_range, color_value):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Converter para HSV
    mask = cv2.inRange(hsv_frame, lower_range, upper_range)  # Criar a máscara com os valores ajustados

    # Encontrar contornos
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterar pelos contornos encontrados
    for c in cnts:
        if cv2.contourArea(c) > 300:  # Limite de área
            epsilon = 0.02 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)
            cv2.drawContours(frame, [approx], 0, color_value, 2)  # Desenhar contorno

# Função principal
while True:
    for jpeg_data in ler_camera():
        # Converte os dados binários para uma imagem usando numpy e OpenCV
        frame = np.frombuffer(jpeg_data, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        if frame is None:
            continue

        # Converter para HSV e equalizar o canal V
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_frame[:, :, 2] = cv2.equalizeHist(hsv_frame[:, :, 2])  # Equalizar o canal V
        frame = cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2BGR)  # Converter de volta para BGR

        # Obter os valores das trackbars
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        # Definir os limites inferior e superior com base nas trackbars
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])

        # Detectar a cor com base nas faixas ajustáveis
        detect_color(frame, lower_range, upper_range, (0, 255, 0))

        # Exibir a imagem processada
        cv2.imshow("FRAME", frame)
        cv2.imshow("MASK", cv2.inRange(hsv_frame, lower_range, upper_range))

        # Pressione 'q' para sair
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# Finaliza o processo e libera os recursos
process.terminate()
cv2.destroyAllWindows()
