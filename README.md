# WORKSHOP ON ROS2

Este repositório contém um ambiente de simulação utilizando **ROS 2** e **Gazebo Ignition**, empacotado em um contêiner **Docker** para facilitar a portabilidade.

> ✅ Desenvolvido para ensinar ROS2

---

## 📦 Estrutura

```
ros2_workshop/
├── Dockerfile
├── workshop_assets/
│   └── assets/
│       ├── launch/
│       ├── models/
│       ├── scripts/
│       ├── src/
|       |-- world/
│       ├── package.xml
│       └── CMakeLists.txt
```

---

## 🚀 Como Rodar (Ubuntu 20.04 limpo)

### 👣 1. Atualize o sistema

```bash
sudo apt update && sudo apt upgrade -y
```

### 👣 2. Instale dependências

```bash
sudo apt install -y curl git gnupg2 lsb-release ca-certificates software-properties-common
```

### 👣 3. Instale o Docker

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

Adicione seu usuário ao grupo `docker`:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

Verifique:

```bash
docker --version
```

---

### 👣 4. Clone este repositório

**Via HTTPS**:

```bash
git clone https://github.com/milenafariap/ros2_workshop.git
cd ros2_workshop
```

---

### 👣 5. Construa a imagem Docker

```bash
docker build -t ros2_workshop .
```

---

### 👣 6. Configure o acesso gráfico (X11)

No terminal do **host**:

```bash
xhost +local:docker
```

---

### 👣 7. Execute o contêiner

```bash
docker run -it --rm \
  -v ~/ros2_workshop/workshop_assets:/root/workshop_assets \
  -e IGN_GAZEBO_RESOURCE_PATH=/root/workshop_assets/world:/root/workshop_assets/assets/sdf_world \
  -e GAZEBO_MODEL_PATH=/root/workshop_assets/assets/models \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  ros2_workshop
```

---

### 👣 8. Compile dentro do contêiner

```bash
cd /root/workshop_assets
colcon build
source install/setup.bash
ros2 launch explore_world gazebo_with_bridge.launch.py
```
### 👣 9. Rodar o arquivo launch dentro do contêiner

```bash
source install/setup.bash
ros2 launch explore_world gazebo_with_bridge.launch.py
```

### 👣 10. Abrindo o docker em outro terminal - consultando o nome do docker

```bash
docker ps
```

### 👣 11. Abrindo o docker em outro terminal - Executando o arquivo docker

```bash
docker exec it "NOME DO ARQUIVO" bash
```

✅ Isso abrirá o mundo `.sdf` no Gazebo Ignition com pontes ROS 2 ativas.

---

## 🔹 Modo Alternativo: Rodar somente o mundo `.sdf`

Caso queira apenas abrir o mundo sem ROS 2:

### Dentro do contêiner:

```bash
ign gazebo /root/workshop/workshop_assets/assets/world/explore_world.sdf --verbose
```

Ou se estiver no host com Ignition instalado:

```bash
ign gazebo path/to/building_robot.sdf --verbose
```

---

## 🧪 Testes rápidos

Reabrir o contêiner:

```bash
docker start -ai ros2_workshop
```

Remover e recriar limpo:

```bash
docker rm -f ros2_gz
docker run ... # (repita o comando completo)
```

---

## 🔒 Segurança

Ao terminar:

```bash
xhost -local:docker
```

---
