# Wheel-Track gazebo model

## Запуск модели 

1. Клонировать репо с помощью git clone --recursive https://github.com/lsd-maddrive/eSAUL-project.git (to fetch submodules)

2. Собрать пакет используя catkin_make 

3. Запустить модель roslaunch esaul_description esaul.launch

4. Управление моделью rostopic pub /eSAUl/cmd_vel geometry_msgs/Tst -r 3 -- '[1, 0.0, 0.0]' '[0.0, 0.0, 2]'
    - число, где указана 1 отвечает за линейную скорость
    - число, где указана 2 отвечает за угловую скорость

5. Управление с клавиатуры: roslaunch esaul_description twist_key.launch

6. Получение карты
    - Запуск лаунч файла: roslaunch esaul_description gmap.launch
    - Сохранение карты: rosrun map_server map_saver -f test-1 (test-1 - имя сохранения файла)         
 

## Полезные ссылки 

### Курсы
1. https://github.com/KaiL4eK/ros_course/tree/master/Topics

### Описание робота (модель, физика)
1. [link element in URDF](http://wiki.ros.org/urdf/XML/link)

2. [joint element in URDF](http://wiki.ros.org/urdf/XML/joint)

3. [zaWRka](https://github.com/lsd-maddrive/zaWRka-project/blob/1c5fed5e65bdc573844c7fcaa0568ca798f53ab2/wr8_description/urdf/model.urdf.xacro#L65)

4. [ROS plugin](http://gazebosim.org/tutorials/?tut=ros_plugins)

5. [Building a world in gazebo](http://gazebosim.org/tutorials?tut=build_world)

6. [Using roslaunch to start Gazebo, world files and URDF models](http://gazebosim.org/tutorials?tut=ros_roslaunch)

7. https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q

### Формулы
1. [Differential Drive Robots](http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)

### Решение различных проблем на форумах