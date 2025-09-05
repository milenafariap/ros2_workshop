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
  --name ros2_workshop_container \
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
```

### 👣 9. Rodar o arquivo launch dentro do contêiner

```bash
source install/setup.bash
ros2 launch explore_world gazebo_with_bridge.launch.py
```

---

## 🤖 Testes - tópicos e movimento

### Abrindo o docker em outro terminal - consultando o nome do docker

```bash
docker ps
```

### Abrindo o docker em outro terminal - Executando o arquivo docker

```bash
docker exec it "NOME DO ARQUIVO" bash
```

## 👣 1. Listar os tópicos ativos:

```bash
ros2 topic list -t
```

## 👣 2. Conferir mensagens chegando no tópico:

```bash
ros2 topic echo /cmd_vel
```

## 👣 3. Mandar o robô andar para frente

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 👣 4. Mandar o robô girar no lugar

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

## 👣 5. Parar imediatamente

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
---

## 🔎 Debug avançado

Ver os nós ativos

```bash
ros2 node list
```

Ver os serviços disponíveis

```bash
ros2 service list
```

Ver informações detalhadas de um tópico

```bash
ros2 topic info /cmd_vel
```

Ver a taxa de publicação

```bash
ros2 topic hz /cmd_vel
```
---

🚀 Executar o teleop

No mesmo ambiente/contêiner em que seu robô está rodando:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Você verá algo assim:

Reading from keyboard
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold shift:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop



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
