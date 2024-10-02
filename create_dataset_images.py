import cv2, random
import numpy as np

# Carregar as imagens
png_image = cv2.imread("png_images/0.png", cv2.IMREAD_UNCHANGED)
jpeg_image = cv2.resize(cv2.imread("sample.jpg"),(640,640))
while(True):
    target_width = target_width = random.randint(60, 500)
    target_height = target_width

    pos_x, pos_y = random.randint(0,639-target_width), random.randint(0,639-target_height)


    # Redimensionar a imagem PNG
    png_resized = cv2.resize(png_image, (target_width, target_height))

    # Separar os canais do PNG
    overlay_bgr = png_resized[:, :, :3]
    overlay_alpha = png_resized[:, :, 3] / 255.0

    # Criar a máscara
    alpha_mask = np.stack((overlay_alpha,) * 3, axis=-1)

    # Recortar a área do fundo
    fundo_recorte = jpeg_image[pos_y:pos_y + target_height, pos_x:pos_x + target_width]

    # Combinar as imagens usando operações vetorizadas
    fundo_recorte = (fundo_recorte * (1 - alpha_mask) + overlay_bgr * alpha_mask).astype(np.uint8)

    # Colocar a área combinada de volta na imagem de fundo
    jpeg_image[pos_y:pos_y + target_height, pos_x:pos_x + target_width] = fundo_recorte

    # Mostrar e salvar a imagem final
    cv2.imshow("Imagem Resultante", jpeg_image)
    cv2.imwrite("resultado.jpg", jpeg_image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
