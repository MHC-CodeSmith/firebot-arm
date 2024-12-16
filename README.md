Aqui está um novo e detalhado `README.md` explicando passo a passo como configurar e rodar o projeto do zero.

---

# Firebot Arm Control

Este projeto implementa a programação e simulação de um braço robótico utilizando ROS 2 Humble. O ambiente é configurado para rodar em Docker com suporte a GPU, permitindo a execução de aplicações gráficas como RViz e Gazebo.

---

## **Passo 1: Pré-requisitos**

Certifique-se de ter instalado:

1. **Docker** e **Docker Compose**.
   - Instale o Docker seguindo [este guia](https://docs.docker.com/engine/install/).
   - Configure o NVIDIA Container Toolkit para habilitar GPU no Docker:
     ```bash
     sudo apt-get install -y nvidia-container-toolkit
     sudo systemctl restart docker
     ```

2. **X11 Display** configurado para suporte gráfico:
   - Execute o comando para permitir que o Docker acesse seu display:
     ```bash
     xhost +local:
     ```

---

## **Passo 2: Build da Imagem Docker**

1. Clone este repositório:
   ```bash
   git clone https://github.com/SEU_USUARIO/firebot-arm.git
   cd firebot-arm
   ```

2. Construa a imagem Docker utilizando o `Dockerfile` fornecido:
   ```bash
   docker build -t ros_arm:humble -f docker/Dockerfile .
   ```

---

## **Passo 3: Rodando o Container Docker**

1. Execute o script `run_docker.sh` para iniciar o ambiente Docker: (lembre de dar chmod +X)
   ```bash
   ./run_docker.sh
   ```

2. Após entrar no container, você verá o terminal do ambiente ROS 2 configurado.

---

## **Passo 4: Configurando o Workspace**

1. Dentro do container, vá até o workspace:
   ```bash
   cd /home/developer/ARM_ws
   ```

2. Limpe qualquer build anterior (caso tenha):
   ```bash
   rm -rf build install log
   ```

3. Compile o workspace:
   ```bash
   colcon build
   ```

4. Configure os ambientes ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ARM_ws/install/setup.bash
   ```

---

## **Passo 5: Rodando a Simulação**

### 5.1. Executar o RViz
Para visualizar o robô no RViz, rode:
```bash
ros2 launch firebot_arm_pkg display.launch.py
```

### 5.2. Executar o Gazebo
Para carregar o robô no Gazebo, execute:
```bash
ros2 launch firebot_arm_pkg gazebo.launch.py
```

---

## **Estrutura do Projeto**

Abaixo está a estrutura do projeto:

```plaintext
.
├── docker/
│   ├── Dockerfile             # Configuração do ambiente Docker
├── src/
│   ├── firebot_arm_pkg/
│   │   ├── config/            # Arquivos de configuração ROS 2
│   │   ├── urdf/              # Arquivos URDF e Xacro
│   │   ├── launch/            # Arquivos de lançamento ROS 2
│   │   ├── scripts/           # Scripts auxiliares (opcional)
│   │   └── meshes/            # Malhas do modelo do robô
├── README.md                  # Documentação do projeto
└── run_docker.sh              # Script para rodar o container
```

---

## **Resolução de Problemas**

### 1. **Erro de Permissão no X11 Display**
   - Execute:
     ```bash
     xhost +local:
     ```

### 2. **Erro ao Conectar GPU**
   - Certifique-se de que o NVIDIA Container Toolkit está instalado:
     ```bash
     sudo apt-get install nvidia-container-toolkit
     sudo systemctl restart docker
     ```

### 3. **Problemas com Build no ROS 2**
   - Certifique-se de ter limpado o workspace antes de rebuildar:
     ```bash
     cd /home/developer/ARM_ws
     rm -rf build install log
     colcon build
     ```

---

## **Licença**

Este projeto está licenciado sob a licença [MIT](LICENSE).

--- 

