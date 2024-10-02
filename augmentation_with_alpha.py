import cv2
import numpy as np

# Carregar a imagem JPEG e a imagem PNG com alpha
jpeg_image = cv2.imread('sample.jpeg')
png_image = cv2.imread('png_images/0.png', cv2.IMREAD_UNCHANGED)
png_image = cv2.resize(png_image,(50,50))

# Verifique se as imagens foram carregadas corretamente
if jpeg_image is None or png_image is None:
    print("Erro ao carregar as imagens.")
    exit()

# Redimensionar a PNG se necessário (opcional)
png_height, png_width = png_image.shape[:2]
jpeg_height, jpeg_width = jpeg_image.shape[:2]

if png_width > jpeg_width or png_height > jpeg_height:
    png_image = cv2.resize(png_image, (jpeg_width, jpeg_height))

# Definir a posição para sobreposição (por exemplo, canto superior esquerdo)
x_offset = 0
y_offset = 0

# Verifique os limites da imagem
if x_offset + png_width > jpeg_width or y_offset + png_height > jpeg_height:
    print("A imagem PNG não cabe na imagem JPEG nesta posição.")
    exit()

# Obter o canal alpha da imagem PNG
alpha_channel = png_image[:, :, 3] / 255.0  # normaliza para [0, 1]

# Combinar as imagens
for c in range(0, 3):  # Para os canais RGB
    jpeg_image[y_offset:y_offset+png_height, x_offset:x_offset+png_width, c] = \
        jpeg_image[y_offset:y_offset+png_height, x_offset:x_offset+png_width, c] * (1 - alpha_channel) + \
        png_image[:, :, c] * alpha_channel

# Salvar a imagem resultante
cv2.imwrite('0.png', jpeg_image)

# Exibir a imagem resultante (opcional)
cv2.imshow('Imagem Resultante', jpeg_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
