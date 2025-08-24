import cv2
import numpy as np
import matplotlib.pyplot as plt

# Carrega imagem do mapa (obstáculos em preto = 0)
img = cv2.imread('../../../../Imagens/Captura de tela de 2025-06-29 17-15-41.png', cv2.IMREAD_GRAYSCALE)

# Define resolução do mapa e origem (se souber)
resolution = 0.05  # metros por pixel
origin_x = -10.0   # origem em metros
origin_y = -10.0

# Limiares: obstáculos são pixels < 50
_, binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)

# Encontra contornos (obstáculos)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

print(f"Detectados {len(contours)} obstáculos")

for i, cnt in enumerate(contours):
    M = cv2.moments(cnt)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        world_x = origin_x + cx * resolution
        world_y = origin_y + cy * resolution
        print(f"Obstáculo {i+1}: ({world_x:.2f}, {world_y:.2f})")

# Visualização
plt.imshow(binary, cmap='gray')
plt.title("Mapa de obstáculos")
plt.show()
