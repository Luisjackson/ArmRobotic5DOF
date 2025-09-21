# Braço robótico com 5 Graus de Liberdade

Projeto de um **braço robótico de 5 DOF** utilizando **ROS2** com pacotes para simulação, controle, descrição do modelo e firmware.

-----

## 📂 Estrutura do Projeto

  - `arm_control/` → Nó responsável pelo controle do braço robótico.
  - `arm_description/` → Arquivos de descrição do robô (URDF/Xacro e etc).
  - `arm_hardware/` → Integração com o hardware real.
  - `arm_py_examples/` → Exemplos em Python de controle do braço
  - `braco_moveit_config/` → Configuração do MoveIt para planejamento de movimento.
  - `firmware/` → Código embarcado para microcontrolador que controla os servomotores Dynamixel AX-12A.
  - `moveit_commander_ros2-main/` → Ferramentas para integração com o MoveIt no ROS2.

-----

## 🚀 Tecnologias Utilizadas

  - **ROS2** (Humble/Foxy)
  - **Gazebo** (simulação física)
  - **MoveIt2** (planejamento de movimento)
  - **Python3** (exemplos de controle)
  - **C++** (drivers/hardware)

-----

## ⚙️ Instalação

Clone o repositório:

```bash
git clone https://github.com/Luisjackson/ArmRobotic5DOF.git
cd ArmRobotic5DOF
```

Compile o workspace:

```bash
colcon build
source install/setup.bash
```

-----

## 🕹️ Como Usar

1.  **Executar simulação no Gazebo**
    ```bash
    ros2 launch arm_description display.launch.py
    ```
2.  **Iniciar MoveIt**
    ```bash
    ros2 launch braco_moveit_config demo.launch.py
    ```
3.  **Rodar exemplo em Python**
    ```bash
    ros2 run arm_py_examples move_test.py
    ```

-----

## 📸 Demonstração

![Braço robótico funcionando](assets/video.gif)

-----

## 📌 Status do Projeto

✅ Simulação básica no RViz/Gazebo

✅ Integração com MoveIt

🔄 Controle de hardware em desenvolvimento
