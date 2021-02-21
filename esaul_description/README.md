# Wheel-Track gazebo model

## Запуск модели 

1. Клонировать репо с помощью git clone --recursive https://github.com/lsd-maddrive/eSAUL-project.git (to fetch submodules)
2. Собрать пакет используя catkin_make 
3. Запустить модель roslaunch esaul_description esaul.launch
4. Управление моделью rostopic pub /eSAUl/cmd_vel geometry_msgs/Tst -r 3 -- '[1, 0.0, 0.0]' '[0.0, 0.0, 2]'
    - число, где указана 1 отвечает за линейную скорость
    - число, где указана 2 отвечает за угловую скорость

 