import cv2
import numpy as np
import subprocess
from collections import Counter
import asyncio

async def loop_detector():
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

    # Definição das faixas de cores em HSV
    color_ranges = {
        "AZUL": (np.array([89, 145, 137]), np.array([124, 255, 255]), (255, 0, 0)),
        "AMARELO": (np.array([24, 150, 0]), np.array([50, 255, 255]), (0, 255, 255)),
        "VERDE": (np.array([55, 85, 93]), np.array([87, 255, 255]), (0, 255, 0)),
        "VERMELHO": (np.array([134, 203, 166]), np.array([179, 255, 255]), (0, 0, 255)),
        "BRANCO": (np.array([85, 0, 170]), np.array([179, 133, 255]), (255, 255, 255)),
        "LARANJA": (np.array([0, 197, 193]), np.array([27, 255, 255]), (0, 128, 255))
    }

    # Função para verificar se um contorno é um quadrado
    def is_square(approx):
        if len(approx) != 4:
            return False
        dists = [np.linalg.norm(approx[i] - approx[(i + 1) % 4]) for i in range(4)]
        return max(dists) - min(dists) < 20  # Tolerância para suavizar a detecção

    # Função para detectar cores
    def detect_color(frame):
        detected_colors = []
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Converter o frame para HSV

        for color_name, (lower, upper, color_value) in color_ranges.items():
            mask = cv2.inRange(hsv_frame, lower, upper)  # Criar máscara para a cor
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in cnts:
                if cv2.contourArea(c) > 300:  # Limite de área
                    epsilon = 0.04 * cv2.arcLength(c, True)  # Aumentar epsilon para suavizar
                    approx = cv2.approxPolyDP(c, epsilon, True)
                    if is_square(approx):  # Verificar se o contorno é um quadrado
                        cv2.drawContours(frame, [approx], 0, color_value, 2)  # Desenhar contorno
                        x, y = approx[0][0]
                        cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_value, 2)  # Colocar nome da cor
                        detected_colors.append(color_name)  # Adicionar cor detectada à lista

        return detected_colors  # Retornar cores detectadas

    cores = []  # Lista para armazenar cores detectadas nos frames

    # Função principal
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

        # Detectar cores e adicionar às cores detectadas
        cores_in_frame = detect_color(frame)

        # Atualizar a lista de cores detectadas, mantendo apenas os últimos 5 frames
        cores.append(cores_in_frame)
        if len(cores) > 5:
            cores.pop(0)

        # Determinar a cor mais comum nos últimos 5 frames
        flat_cores = [color for sublist in cores for color in sublist]  # Achatar a lista de listas
        if flat_cores:
            most_common_color = Counter(flat_cores).most_common(1)[0][0]  # Obter a cor mais comum
            print(f"Cor mais presente: {most_common_color}")  # Imprimir a cor mais presente

        # Mostrar o frame processado
        cv2.imshow("FRAME", frame)

        # Pressione 'q' para sair
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        await asyncio.sleep(0.01)
    # Finaliza o processo e libera os recursos
    process.terminate()
    cv2.destroyAllWindows()