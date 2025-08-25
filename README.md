🌾 ROS 2 + Gazebo Ignition: Simulação de Ambiente Agrícola com Docker
Este repositório contém um ambiente de simulação agrícola utilizando ROS 2 e Gazebo Ignition, empacotado em um contêiner Docker para facilitar a portabilidade.

✅ Desenvolvido para facilitar testes e integração de robôs móveis em cenários agrícolas simulados.

📦 Estrutura
ros2_gazebo_docker_farm/
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
🚀 Como Rodar (Ubuntu 20.04 limpo)
👣 1. Atualize o sistema
sudo apt update && sudo apt upgrade -y
👣 2. Instale dependências
sudo apt install -y curl git gnupg2 lsb-release ca-certificates software-properties-common
👣 3. Instale o Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
Adicione seu usuário ao grupo docker:

sudo usermod -aG docker $USER
newgrp docker
Verifique:

docker --version
👣 4. Clone este repositório
Via HTTPS:

git clone https://github.com/milenafariap/ros2_workshop.git
cd ros2_workshop
👣 5. Construa a imagem Docker
docker build -t ros2_workshop .
👣 6. Configure o acesso gráfico (X11)
No terminal do host:

xhost +local:docker
👣 7. Execute o contêiner
docker run -it --rm \
  -v ~/ros2_gazebo_docker_farm:/root/moray \
  -e GZ_SIM_RESOURCE_PATH=/root/workshop_assets/assets/sdf_world \
  -e GAZEBO_MODEL_PATH=/root/workshop_assets/assets/models \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  ros2_workshop
👣 8. Compile e rode dentro do contêiner
cd /root/workshop
colcon build
source install/setup.bash
./run_bridge.py --world ~/workshop/workshop_assets/assets/world/explore_world.sdf --gui
✅ Isso abrirá o mundo .sdf no Gazebo Ignition com pontes ROS 2 ativas.

🔹 Modo Alternativo: Rodar somente o mundo .sdf
Caso queira apenas abrir o mundo sem ROS 2:

Dentro do contêiner:
ign gazebo /root/workshop/workshop_assets/assets/world/explore_world.sdf --verbose
Ou se estiver no host com Ignition instalado:

ign gazebo path/to/building_robot.sdf --verbose
🧪 Testes rápidos
Reabrir o contêiner:

docker start -ai ros2_workshop
Remover e recriar limpo:

docker rm -f ros2_gz
docker run ... # (repita o comando completo)
🔒 Segurança
Ao terminar:

xhost -local:docker
📌 Requisitos
Ubuntu 20.04+
Docker e permissão para usar GPU (se desejar)
Interface gráfica ativa (X11) para visualização do Gazebo
