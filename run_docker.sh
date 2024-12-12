#!/bin/bash

# Permitir acesso ao servidor X11
echo "Permitindo acesso ao servidor X11..."
xhost +local:docker

# Nome do container e imagem
CONTAINER_NAME="ros_arm_sim"
IMAGE_NAME="ros_arm:humble"

# Verificar se a GPU está disponível
if ! command -v nvidia-smi &>/dev/null; then
    echo "NVIDIA GPU não detectada. Certifique-se de que os drivers estão instalados."
    exit 1
fi

# Executar o container com mapeamento de GPU, display e volumes
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PWD:/home/developer/ARM_ws" \
    -v ~/.ssh:/home/developer/.ssh:ro \
    "$IMAGE_NAME"

# Revogar acesso ao servidor X11 após sair do container
echo "Revogando acesso ao servidor X11..."
xhost -local:docker
