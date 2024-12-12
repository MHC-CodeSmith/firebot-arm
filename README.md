Aqui está o `README.md` atualizado para refletir o progresso e a configuração atual do projeto:

```markdown
# Firebot Arm Control

Este projeto implementa a programação e simulação de um braço robótico utilizando ROS 2 Humble. Ele está configurado para rodar em um ambiente Docker com suporte a GPU e aplicações gráficas como RViz e Gazebo.

---

## Estrutura do Projeto

```plaintext
.
├── README.md                  # Descrição geral do projeto
├── LICENSE                    # Licença do projeto
├── .gitignore                 # Arquivos e diretórios ignorados pelo Git
├── docker/                    # Configuração do Docker
│   ├── Dockerfile             # Configuração da imagem Docker
│   └── run_docker.sh          # Script para rodar o container Docker
├── src/                       # Código-fonte do projeto
│   ├── urdf/                  # Arquivos URDF e Xacro
│   ├── config/                # Arquivos de configuração do ROS 2
│   ├── scripts/               # Scripts auxiliares
│   └── launch/                # Arquivos de lançamento do ROS 2
├── tests/                     # Testes para o projeto
│   ├── unit/                  # Testes unitários
│   └── integration/           # Testes de integração
├── docs/                      # Documentação do projeto
│   ├── diagrams/              # Diagramas de fluxo e blocos
│   └── manuals/               # Manuais de uso
└── data/                      # Dados para simulações e configurações
```

---

## Como Configurar o Ambiente

### Pré-requisitos
- **Docker** e **Docker Compose** instalados.
- **NVIDIA Container Toolkit** (para suporte à GPU).

---

### Configurando o Ambiente Docker

1. **Clone o Repositório**
   ```bash
   git clone git@github.com:MHC-CodeSmith/firebot-arm.git
   cd firebot-arm
   ```

2. **Construa a Imagem Docker**
   ```bash
   docker build -t ros_arm:humble -f docker/Dockerfile .
   ```

3. **Execute o Container com o Script**
   Use o script `run_docker.sh` para iniciar o container:
   ```bash
   ./run_docker.sh
   ```

---

## Configuração no Container

1. **Configurar o Ambiente ROS 2**
   Dentro do container, configure o ambiente ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   mkdir -p ~/ARM_ws/src
   cd ~/ARM_ws
   colcon build
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/ARM_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

2. **Criar um Pacote ROS 2**
   Crie um pacote básico no workspace:
   ```bash
   cd ~/ARM_ws/src
   ros2 pkg create --build-type ament_python firebot_arm_pkg
   ```

3. **Testar o Pacote**
   Compile o workspace para verificar se o pacote foi criado corretamente:
   ```bash
   cd ~/ARM_ws
   colcon build
   ```

---

## Estrutura do Pacote ROS 2

Depois de criar o pacote `firebot_arm_pkg`, ele terá a seguinte estrutura:
```plaintext
firebot_arm_pkg/
├── package.xml               # Configuração do pacote ROS 2
├── setup.py                  # Configuração do ambiente Python
├── setup.cfg                 # Configuração do Python
├── resource/                 # Recursos do pacote
│   └── firebot_arm_pkg       # Arquivo de recurso
├── firebot_arm_pkg/          # Diretório do código-fonte
│   └── __init__.py           # Arquivo inicializador
└── test/                     # Testes do pacote
    ├── test_copyright.py     # Teste de copyright
    ├── test_flake8.py        # Teste de conformidade PEP8
    └── test_pep257.py        # Teste de conformidade PEP257
```

---

## Problemas Conhecidos

1. **Configuração de Chave SSH no Container**:
   Certifique-se de mapear corretamente sua chave SSH ao rodar o container.

2. **Erro no Git (Identidade do Autor)**:
   Dentro do container, configure o nome e e-mail do Git:
   ```bash
   git config --global user.name "Seu Nome"
   git config --global user.email "seu_email@exemplo.com"
   ```

---

## Próximos Passos
- Configurar e documentar os arquivos URDF/Xacro na pasta `src/urdf/`.
- Adicionar configurações de lançamento na pasta `src/launch/`.
- Implementar diagramas de fluxo no diretório `docs/diagrams/`.

---

## Licença
Este projeto está licenciado sob a licença [MIT](LICENSE).
```

Esse `README.md` cobre as configurações atuais e indica os próximos passos. Se precisar de ajustes ou mais informações, é só pedir! 😊