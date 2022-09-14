#!/bin/sh
#################################################################
## Install dependancies script to facilitate the installation of 
## required myrobotics dependencies
#################################################################

######################
## Global variables ##
######################

DIRECTORY=~/Libraries
CORES=$(($(nproc)/2))

######################
## Help ##
######################

Instructions() {
    echo ""
    echo "        The script installs most of the dependencies of the My Robotic Framework."
    echo "        The user can select which libraries to install."
    echo ""
    echo "        To install all the libraries execute:"
    echo "            ./installDependencies.sh -a"
    echo ""
    echo "        To install specific libraries execute:"
    echo "            ./installDependencies.sh [Library Name 1] [Library Name 2]"
    echo ""
    echo "        The available libraries are":
    echo ""
    echo "          - BasicTools"
    echo ""
    echo "        If the user selects a specific library, this script does not take care of       "
    echo "    installing its dependencies. The user must select the order of the installation     "
    echo "    wisely. In the case of choosing all the libraries, they will be installed in the    "
    echo "    right order to guarantee no cross dependecy errors."
    echo ""
    echo ""
}



###################
## Dependancies  ##
###################

BasicTools() {
    sudo apt-get install --no-install-recommends --assume-yes apt-utils=2.0.5
    sudo apt-get install --no-install-recommends --assume-yes sudo=1.8.31-1ubuntu1.2
    sudo apt-get install --no-install-recommends --assume-yes ca-certificates=20210119~20.04.1
    sudo apt-get install --no-install-recommends --assume-yes wget=1.20.3-1ubuntu1
    sudo apt-get install --no-install-recommends --assume-yes git=1:2.25.1-1ubuntu3
    sudo apt-get install --no-install-recommends --assume-yes vim=2:8.1.2269-1ubuntu5
    sudo apt-get install --no-install-recommends --assume-yes ssh=1:8.2p1-4ubuntu0.2
    sudo apt-get install --no-install-recommends --assume-yes unzip=6.0-25ubuntu1
    sudo apt-get install  --no-install-recommends --assume-yes net-tools
    

    sudo apt-get install --no-install-recommends --assume-yes python3.8
    sudo apt-get install --no-install-recommends --assume-yes python3-pip

    sudo apt-get -y install python3-rpi.gpio
}

Update() {
    sudo apt-get install software-properties-common
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt-get update && sudo apt-get -y upgrade
}

Core() {
    pip3 install psutil
}

Ros2() {
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt update && sudo apt install curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    sudo apt install ros-foxy-ros-base
    sudo apt install python3-colcon-common-extensions
    pip3 install argcomplete
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    sudo apt install python3-rosdep2
    rosdep update

    # RosBridge
    sudo apt-get install ros-foxy-rosbridge-server
}

Vision() {
    sudo apt install libopencv-dev python3-opencv
}

Motors() {
   pip3 install Adafruit_PCA9685
}

##########################
## Installation Process ##
##########################

Install() {
    echo ""
    echo "----------------------------"
    echo " Installing $1"
    echo "----------------------------"
    echo ""

    if [ ! -d $DIRECTORY ]; then
        mkdir $DIRECTORY
    fi

    case "$1" in

        "Update")            Update                                          ;;
        "BasicTools")        BasicTools                                      ;;
        "Core")              Core                                            ;;
        "Vision")            Vision                                          ;;
        "Ros2")              Ros2                                            ;;
        *)                   printf. "The library $1 is not available \n"    ;;
    esac
}

InstallAll() {
    Install Update
    Install BasicTools
    Install Core
    Install Vision
    Install Ros2
}

##########
## Main ##
##########

if [ "$#" = 1 ]; then
    if [ "$1" = "help" ]; then
        Instructions
    elif [ "$1" = "-a" ]; then
        InstallAll
    else
        Install "$1"
    fi
elif [ "$#" -gt 1 ]; then
    for i in "$@"; do
        Install "$i"
    done
else
    Instructions
fi

