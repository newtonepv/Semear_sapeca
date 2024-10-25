import cv2
import subprocess
import numpy as np
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
def ler_camera():
    # Lê dados binários de um único quadro (MJPEG é segmentado por '\xff\xd8' e '\xff\xd9')
    frame_data = b''
    while True:
        chunk = process.stdout.read(1024)
        if not chunk:
            break
        frame_data += chunk
        if b'\xff\xd9' in chunk:  # Fim do JPEG
            break

    # Converte os dados binários para uma imagem usando numpy e OpenCV
    frame = np.frombuffer(frame_data, dtype=np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    return frame

    