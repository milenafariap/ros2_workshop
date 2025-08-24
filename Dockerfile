FROM osrf/ros:humble-desktop-full

# Instala dependências ROS 2 + OpenCV + tf + ferramentas extras
RUN apt update && apt install -y \
    ros-humble-ros-ign \
    ros-humble-ros-ign-gazebo \
    ros-humble-joint-state-publisher-gui \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf-transformations \
    ros-humble-rviz2 \
    python3-opencv \
    libopencv-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Instala bibliotecas Python adicionais
RUN pip install --no-cache-dir numpy scipy opencv-python

# Variáveis de ambiente para localizar modelos e mundos
ENV GZ_SIM_RESOURCE_PATH=/root/workshop/sdf_world
ENV GAZEBO_MODEL_PATH=/root/workshop/models
ENV ROS_DISTRO=humble

# Ativa o ROS no bash
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

