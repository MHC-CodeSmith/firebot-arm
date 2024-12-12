# Base completa com ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop-full

# Define o ambiente como não interativo
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

# Configuração de fuso horário e atualização de pacotes
RUN ln -snf /usr/share/zoneinfo/America/Sao_Paulo /etc/localtime && echo "America/Sao_Paulo" > /etc/timezone \
    && apt-get update \
    && apt-get install -y \
        sudo \
        git \
        vim \
        gedit \
        build-essential \
        wget \
        curl \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-pip \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-xacro \
        ros-humble-moveit* \
        ros-humble-rviz2 \
        ros-humble-joint-state-publisher-gui \
        ros-humble-robot-state-publisher \
        ros-humble-tf2-ros \
        ros-humble-tf-transformations \
        ros-humble-ros-gz-* \
    && rm -rf /var/lib/apt/lists/*

# Instalar pacotes adicionais para gráficos e simulação
RUN apt-get update \
    && apt-get install -y \
        mesa-utils \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

# Configurar o usuário não-root
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# Configurar o workspace ROS
RUN mkdir -p ~/ARM_ws/src
WORKDIR /home/$USERNAME/ARM_ws

# Copiar os arquivos do workspace para o container
COPY . /home/$USERNAME/ARM_ws/

# Build do workspace com colcon
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Configuração de ambiente
ENV ROS_DISTRO=humble
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0
ENV HOME=/home/$USERNAME

# Configuração para inicialização
CMD ["/bin/bash"]
