# Gazebo的各种demo

## 工作环境

* Ubuntu 18.04
* ROS Melodic Morenia
* Gazebo 11.0

## 配置环境

```
    cd ~
    mkdir -p ~/catkin_gazebo_ws/src
    cd ~/catkin_gazebo_ws/src
    git clone https://github.com/gaoyichao/gazebo_demos.git
    cd ..
    catkin_make
    source devel/setup.bash
    roscd gazebo_demos
    source ./setup.sh
    roscore &
    gazebo worlds/velodyne.world &
    rostopic pub /my_velodyne/vel_cmd std_msgs/Float32 "data: 1.0" 
```

## 关联

* [Gazebo仿真](https://gaoyichao.com/Xiaotu/?book=Gazebo&title=index)

