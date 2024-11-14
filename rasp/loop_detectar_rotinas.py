import cv2
import numpy as np
import subprocess
from collections import Counter
import asyncio



async def loop_detector(setpoint):
    '''rodar = {0: True}  # Dicionário para controlar a execução do loop

    async def cooldown(rodar):
        rodar[0] = False  # Desativa o loop
        await asyncio.sleep(300000000000)  # Tempo de cooldown
        rodar[0] = True 


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

    # Função para detectar cores e áreas de quadrados
    def detect_squares(frame):
        area_sum = Counter()  # Contador para áreas detectadas
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Converter o frame para HSV

        for color_name, (lower, upper, color_value) in color_ranges.items():
            mask = cv2.inRange(hsv_frame, lower, upper)  # Criar máscara para a cor
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APif cv2.waitKey(1) & 0xFF == ord("q"):
                    breakPROX_SIMPLE)

            for c in cnts:
                area = cv2.contourArea(c)  # Calcular a área do contorno
                if area > 1000:  # Verificar se a área é maior que 2000
                    if area > 300:  # Limite de área mínima para contornos
                        epsilon = 0.02 * cv2.arcLength(c, True)
                        approx = cv2.approxPolyDP(c, epsilon, True)
                        if is_square(approx):  # Verificar se o contorno é um quadrado
                            area_sum[color_name] += area  # Adicionar área ao contador
                            cv2.drawContours(frame, [approx], 0, color_value, 2)  # Desenhar contorno
                            x, y = approx[0][0]
                            cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_value, 2)  # Colocar nome da cor

        return area_sum  # Retornar áreas detectadas

    # Função principal
    while True:
        for jpeg_data in ler_camera():
            if(rodar[0]):
                # Converte os dados binários para uma imagem usando numpy e OpenCV
                frame = np.frombuffer(jpeg_data, dtype=np.uint8)
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

                if frame is None:
                    continue

                # Converter para HSV e equalizar o canal V
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                hsv_frame[:, :, 2] = cv2.equalizeHist(hsv_frame[:, :, 2])  # Equalizar o canal V
                frame = cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2BGR)  # Converter de volta para BGR

                # Detectar quadrados e calcular as áreas
                area_sum = detect_squares(frame)

                # Determinar a cor com a maior área
                if area_sum:
                    max_area_color = area_sum.most_common(1)[0]  # Cor com maior área
                    """print(f"Cor com maior área: {max_area_color[0]} (Área: {max_area_color[1]:.2f})")
                    if(max_area_color[0] == 'AMARELO'):
                        setpoint[0] = 10
                    if(max_area_color[0] == 'VERMELHO'):
                        setpoint[0] = -10
                    else:
                        setpoint[0] = 0"""

                # Mostrar o frame processado
                #cv2.imshow("FRAME", frame)

                # Pressione 'q' para sair
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            
            await asyncio.sleep(0.001)
        await asyncio.sleep(0.001)

    # Finaliza o processo e libera os recursos
    process.terminate()
    cv2.destroyAllWindows()'''

