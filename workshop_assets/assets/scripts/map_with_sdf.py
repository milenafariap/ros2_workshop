import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np


# Caminho para seu arquivo .sdf
sdf_path = "../world/forest.sdf"

# Lê o arquivo SDF
tree = ET.parse(sdf_path)
root = tree.getroot()

# Para lidar com namespaces (ignorar)
def strip_namespace(tag):
    return tag.split('}')[-1]



print("🔍 Obstáculos encontrados:")
# Armazena as posições dos obstáculos
obstaculos = []

for include in root.iter():
    if strip_namespace(include.tag) == "include":
        pose_elem = include.find("pose")
        uri_elem = include.find("uri")
        
        if uri_elem is not None and pose_elem is not None:
            pose_values = list(map(float, pose_elem.text.strip().split()))
            uri_text = uri_elem.text.strip()
            model_name = uri_text.split("/")[-1]

            x, y, z = pose_values[:3]
            obstaculos.append((x, y))
            print(f"🪵 {model_name}: posição (x={x:.2f}, y={y:.2f}, z={z:.2f})")

# --- PLOTAGEM COM MATPLOTLIB ---

# Extrai coordenadas
xs, ys = zip(*obstaculos) if obstaculos else ([], [])

# Configura o plot
plt.figure(figsize=(10, 10))
plt.scatter(xs, ys, c='red', label='Obstáculos')
plt.title('Mapa 2D dos Obstáculos no Mundo')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()
# --- CRIAÇÃO DA MATRIZ DE PONTUAÇÃO (MAPA DE CUSTO) ---

# Parâmetros
resolucao = 0.5         # metros por célula
valor_obstaculo = 1000  # pontuação máxima
alcance = 1          # raio de influência do obstáculo em metros

# Define os limites do mapa
margin = 1  # margem extra em metros
min_x = int(min(xs)) - margin
max_x = int(max(xs)) + margin
min_y = int(min(ys)) - margin
max_y = int(max(ys)) + margin

# Dimensões da matriz
nx = int((max_x - min_x) / resolucao)
ny = int((max_y - min_y) / resolucao)
mapa = np.zeros((ny, nx))

# Preencher o mapa com valores baseados na distância aos obstáculos
for ox, oy in obstaculos:
    cx = int((ox - min_x) / resolucao)
    cy = int((oy - min_y) / resolucao)

    raio_celulas = int(alcance / resolucao)
    for dy in range(-raio_celulas, raio_celulas + 1):
        for dx in range(-raio_celulas, raio_celulas + 1):
            x = cx + dx
            y = cy + dy
            if 0 <= x < nx and 0 <= y < ny:
                dist = np.hypot(dx * resolucao, dy * resolucao)
                if dist <= alcance:
                    score = valor_obstaculo * np.exp(-dist**2 / (2 * (alcance / 2)**2))
                    mapa[y, x] = max(mapa[y, x], score)

# Salvar em arquivo txt
np.savetxt("mapa_potencial.txt", mapa, fmt="%.2f")
print("✅ Mapa salvo como mapa_potencial.txt")

# Visualização opcional
plt.imshow(mapa, origin='lower', extent=[min_x, max_x, min_y, max_y], cmap='hot')
plt.colorbar(label="Custo")
plt.title("Mapa de Potencial (Custo)")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(False)
plt.show()