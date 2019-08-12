## 机器人安装环境

## 1. ROS包
### ros kinetic install

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

    sudo apt-get update

    sudo apt-get install ros-kinetic-desktop
    or
    sudo apt-get install ros-kinetic-ros-base

    sudo rosdep init
    rosdep update

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

### install necessary ros pkg

    ...

### install dependency
    rosdep install --from-paths src --ignore-src -r -y

## install GPU dependency
### install nvidia-driver
### install cuda 

    install cuda 8.0.61

### install cudnn
    install cudnn 6

### install tensorflow 
    install tf 1.3.0

### install python pip pkg
    pip install ...
    
## install JAVA 
### install jdk
    jdk 1.8.0_211
### install node
    node v 10.16

## zed install

## opencv install
	install opencv 2.4.9



