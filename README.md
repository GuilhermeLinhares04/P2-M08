# SLAM com Turtlebot3

Este projeto tem como objetivo a implementação de um sistema de SLAM (Simultaneous Localization and Mapping) em um robô Turtlebot3. Este processo é realizado através de um algoritmo de navegação autônoma que permite ao robô se localizar e mapear o ambiente ao mesmo tempo.

# Instalação e utilização

Para que a instalação seja realizada com êxito, é recomendado o uso do **Ubuntu 22.04**, com o  **ROS2** instalado e o **Nav2** disponível. Abaixo segue o passo a passo ordenado para a instalação: 

1. Clone este repositório
2. Abra um terminal e envie o comando `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` para abrir o ambiente de simulação
3. Abra um novo terminal e envie o comando `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=<caminho-do-mapa>.yaml` para iniciar o mapa através do Rviz
4. Abra um novo terminal e envie o comando `colcon build` para compilar o projeto e `source install/setup.bash` para executar o projeto
5. Por fim abra um novo terminal e envie o comando `ros2 run the_slam_py shazam` para iniciar a navegação autônoma