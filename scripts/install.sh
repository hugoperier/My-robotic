#!/bin/sh

######################
## Global variables ##
######################

DIRECTORY=~/Libraries
CORES=$(($(nproc)/2))

Instructions() {
    echo ""
    echo "        The script installs most of the dependencies of the CERN Robotic Framework in   "
    echo "    C++ The user can select which libraries to install. Each of them is downloaded and  "
    echo "    compiled inside the folder ~/Libraries."
    echo ""
    echo "        To install all the libraries execute:"
    echo "            ./installDependencies.sh All"
    echo ""
    echo "        To install specific libraries execute:"
    echo "            ./installDependencies.sh [Library Name 1] [Library Name 2]"
    echo ""
    echo "        The available libraries are":
    echo ""
    echo "          - BasicTools"
    echo ""
    echo "          - Abseil                - JSON                  - OpenCV"
    echo "          - Boost                 - Libconfig             - ViSP"
    echo "          - SML                   - Protobuf              - x264"
    echo "          - SpdLog                - TinyXML               - x265"
    echo "                                  - XML2                  - ZBar"
    echo "          - Eigen"
    echo "          - FLANN                 - cURL                  - FCL"
    echo "          - GSL                   - FreeTDS               - Libccd"
    echo "          - G2O                   - MySQLClient           - Octomap"
    echo "          - KDL                   - LZ4                   - PCL"
    echo "          - NLopt                 - Modbus"
    echo "          - OMPL                  - ODBC                  - Matplotlib"
    echo "          - SuitSparse            - Restbed               - QGLViewer"
    echo "                                  - ZMQ                   - Qt5"
    echo "          - DynamixelSDK                                  - VTK"
    echo "          - KinovaAPI"
    echo "          - SOEM"
    echo ""
    echo "          - Hokuyo"
    echo "          - IntelRealSense"
    echo "          - XeThruRadarAPI"
    echo ""
    echo "        If the user selects a specific library, this script does not take care of       "
    echo "    installing its dependencies. The user must select the order of the installation     "
    echo "    wisely. In the case of choosing all the libraries, they will be installed in the    "
    echo "    right order to guarantee no cross dependecy errors."
    echo ""
    echo "        Along the installation process you might have to write the sudo password and the"
    echo "    CERN credentials to download files located in gitlab.cern.ch"
    echo ""
    echo "        Unsupported libraries:"
    echo "          - CUDA"
    echo "          - OptrisAPI"
    echo "          - XSensAPI"
    echo "          - Tensorflow"
    echo "          - GPUVoxels"
    echo ""
    echo "        To install them go to the section 'Libraries & Dependencies' of the installation"
    echo "    process in the documentation."
    echo ""
    echo "        https://readthedocs.web.cern.ch/pages/viewpage.action?pageId=153518392"
    echo ""
}



##################
## Basic Tools  ##
##################

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
    pip install --user pipenv
}


Update() {
    sudo apt-get install software-properties-common
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt-get update && sudo apt-get -y upgrade
}


######################
## Generic Packages ##
######################

Abseil() {
    cd $DIRECTORY || return
    git clone https://github.com/abseil/abseil-cpp.git
    cd abseil-cpp || return
    mkdir build
    cd build || return
    cmake ../ -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$CORES
    sudo make install
}

Boost() {
    sudo apt-get install --no-install-recommends --assume-yes libboost-all-dev=1.71.0.0ubuntu2
}

SML() {
    cd $DIRECTORY || return
    git clone https://github.com/boost-ext/sml.git
    cd sml/ || return
    git checkout v1.1.3
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

SpdLog() {
    cd $DIRECTORY || return
    git clone https://github.com/gabime/spdlog.git
    cd spdlog || return
    git checkout v1.8.2
    mkdir build
    cd build || return
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ../
    make -j$CORES
    sudo make install
}



###########################
## Mathematical Packages ##
###########################

Eigen() {
    cd $DIRECTORY || return
    git clone https://gitlab.com/libeigen/eigen.git
    cd eigen || return
    git checkout 3.3.9
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

FLANN() {
    sudo apt-get install --no-install-recommends --assume-yes libflann-dev=1.9.1+dfsg-9build1
}

GSL() {
    sudo apt-get install --no-install-recommends --assume-yes libgsl-dev=2.5+dfsg-6build1
}

G2O() {
    cd $DIRECTORY || return
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o || return
    git checkout 20200410_git
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

KDL() {
    cd $DIRECTORY || return
    git clone https://github.com/orocos/orocos_kinematics_dynamics.git
    cd orocos_kinematics_dynamics || return
    git checkout v1.4.0
    cd orocos_kdl || return
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

NLopt() {
    cd $DIRECTORY || return
    git clone https://github.com/stevengj/nlopt.git
    cd nlopt || return
    git checkout v2.7.0
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

OMPL() {
    cd $DIRECTORY || return
    git clone https://github.com/ompl/ompl.git
    cd ompl || return
    git checkout 1.5.0
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

SuitSparse() {
    sudo apt-get install --no-install-recommends --assume-yes libsuitesparse-dev=1:5.7.1+dfsg-2
}



###############
## Acutators ##
###############

DynamixelSDK() {
    cd $DIRECTORY || return
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    cd DynamixelSDK || return
    git checkout 3.7.31
    cd c++/build/linux64 || return
    make -j$CORES
    sudo make install
}

KinovaAPI() {
    cd $DIRECTORY || return
    git clone https://gitlab.cern.ch/en-smm-mro/Robotronics/Libraries/kinovajaco2api.git
    cd kinovajaco2api || return
    git checkout 1.5.1
    cd Architectures/x86/64bits/ || return
    sudo sh InstallAPI64x86.sh
}

SOEM() {
    cd $DIRECTORY || return
    git clone https://github.com/OpenEtherCATsociety/SOEM.git
    cd SOEM || return
    mkdir build
    cd build || return
    cmake -DCMAKE_C_FLAGS=-fPIC -DCMAKE_INSTALL_PREFIX=/usr/local ../
    make -j$CORES
    sudo make install
}



#############
## Sensors ##
#############

Hokuyo() {
    cd $DIRECTORY || return
    wget -nc https://sourceforge.net/projects/urgnetwork/files/urg_library/urg_library-1.2.5.zip/download
    sudo apt-get --assume-yes install unzip
    unzip download
    rm download
    cd urg_library-1.2.5/ || return
    make -j$CORES
    sudo make install
}

IntelRealSense() {
    cd $DIRECTORY || return
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense || return
    git checkout v2.43.0
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

XeThruRadarAPI() {
    cd $DIRECTORY || return
    git clone https://gitlab.cern.ch/en-smm-mro/Robotronics/Libraries/xethruradarapi.git
    cd xethruradarapi || return
    git checkout 1.5.3
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}



###################################################
## Serialization and Configuration File Packages ##
###################################################

JSON() {
    cd $DIRECTORY || return
    git clone https://github.com/nlohmann/json.git
    cd json || return
    git checkout v3.9.1
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

Libconfig() {
    sudo apt-get install --no-install-recommends --assume-yes libconfig++-dev=1.5-0.4build1
}

Protobuf() {
    cd $DIRECTORY || return
    git clone https://github.com/protocolbuffers/protobuf.git
    cd protobuf || return
    git checkout 3.7.x
    git submodule update --init --recursive
    ./autogen.sh
    ./configure
    make -j$CORES
    sudo make install
}

TinyXML() {
    sudo apt-get install --no-install-recommends --assume-yes libtinyxml-dev=2.6.2-4build1
}

XML2() {
    sudo apt-get install --no-install-recommends --assume-yes libxml2-dev=2.9.10+dfsg-5
}



############################
## Communication Packages ##
############################

cURL() {
    cd $DIRECTORY || return
    sudo apt-get install --no-install-recommends --assume-yes libcurl4-openssl-dev=7.68.0-1ubuntu2.5
    git clone https://github.com/jpbarrette/curlpp.git
    cd curlpp || return
    git checkout v0.8.1
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

FreeTDS() {
    sudo apt-get install --no-install-recommends --assume-yes freetds-dev=1.1.6-1.1
}

MySQLClient() {
    sudo apt-get install --no-install-recommends --assume-yes libmysqlclient-dev=8.0.25-0ubuntu0.20.04.1
}

LZ4() {
    cd $DIRECTORY || return
    git clone https://github.com/lz4/lz4.git
    cd lz4 || return
    git checkout v1.9.3
    make -j$CORES
    sudo make install
}

Modbus() {
    sudo apt-get install --no-install-recommends --assume-yes libmodbus-dev=3.1.6-2
}

ODBC() {
    sudo apt-get install --no-install-recommends --assume-yes unixodbc-dev=2.3.6-0.1build1
}

Restbed() {
    cd $DIRECTORY || return
    git clone --recursive https://github.com/Corvusoft/restbed.git
    cd restbed || return
    git checkout 4.7
    mkdir build
    cd build || return
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SSL=OFF ../
    make -j$CORES
    sudo make install
}

ZMQ() {
    cd $DIRECTORY || return
    git clone https://github.com/zeromq/libzmq.git
    cd libzmq || return
    git checkout v4.2.5
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
    cd $DIRECTORY || return
    git clone https://github.com/zeromq/cppzmq.git
    cd cppzmq || return
    git checkout v4.2.2
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}



#########################################
## Image and Video Processing Packages ##
#########################################

OpenCV() {
    cd $DIRECTORY || return
    git clone https://github.com/opencv/opencv.git
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv_contrib || return
    git checkout 4.5.2
    MODULES=$(pwd)/modules
    cd ../opencv || return
    git checkout 4.5.2
    mkdir build
    cd build || return
    cmake ../ -DOPENCV_EXTRA_MODULES_PATH="$MODULES"
    make -j$CORES
    sudo make install
}

ViSP() {
    cd $DIRECTORY || return
    git clone https://github.com/lagadic/visp.git
    cd visp/ || return
    git checkout v3.4.0
    mkdir build
    cd build/ || return
    cmake ../
    make -j$CORES
    sudo make install
}

x264() {
    cd $DIRECTORY || return
    git clone https://gitlab.cern.ch/en-smm-mro/Robotronics/Libraries/x264.git
    cd x264 || return
    ./configure --enable-shared --enable-pic
    make -j$CORES
    sudo make install lib-shared
}

x265() {
    cd $DIRECTORY || return
    git clone https://gitlab.cern.ch/en-smm-mro/Robotronics/Libraries/x265.git
    cd x265 || return
    git checkout 3.0
    cd source || return
    mkdir build
    cd build || return
    cmake ../
    make -j$CORES
    sudo make install
}

ZBar() {
    sudo apt-get install --no-install-recommends --assume-yes libzbar-dev=0.23-1.3
}



#################################
## 3D Data Processing Packages ##
#################################

FCL() {
    cd $DIRECTORY || return
    git clone https://github.com/flexible-collision-library/fcl.git
    cd fcl || return
    git checkout v0.6.1
    mkdir build
    cd build/ || return
    cmake ../
    make -j$CORES
    sudo make install
}

Libccd() {
    cd $DIRECTORY || return
    git clone https://github.com/danfis/libccd.git
    cd libccd || return
    git checkout v2.1
    mkdir build
    cd build || return
    cmake -G "Unix Makefiles" ../
    make -j$CORES
    sudo make install
}

Octomap() {
    cd $DIRECTORY || return
    git clone https://github.com/OctoMap/octomap.git
    cd octomap/ || return
    git checkout v1.8.1
    mkdir build
    cd build/ || return
    cmake ../
    make -j$CORES
    sudo make install
}

PCL() {
    cd $DIRECTORY || return
    git clone https://github.com/PointCloudLibrary/pcl.git
    cd pcl/ || return
    git checkout pcl-1.9.1
    mkdir build
    cd build || return
    make clean
    cmake ../ -DBUILD_examples=OFF
    make -j$CORES
    sudo make install
}



######################################
## Visualization and User Interface ##
######################################

Matplotlib() {
    sudo apt-get install --no-install-recommends --assume-yes python3-matplotlib=3.1.2-1ubuntu4
}

Qt5() {
    sudo apt-get install --no-install-recommends --assume-yes qtdeclarative5-dev=5.12.8-0ubuntu1
}

QGLViewer() {
    sudo apt-get install --no-install-recommends --assume-yes libqglviewer-dev-qt5=2.6.3+dfsg2-6build1
}

VTK() {
    sudo apt-get install --no-install-recommends --assume-yes libvtk7-dev=7.1.1+dfsg2-2ubuntu1
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

        "BasicTools")        BasicTools                                      ;;
        "Abseil")            Abseil                                          ;;
        "Boost")             Boost                                           ;;
        "SML")               SML                                             ;;
        "SpdLog")            SpdLog                                          ;;
        "Eigen")             Eigen                                           ;;
        "FLANN")             FLANN                                           ;;
        "GSL")               GSL                                             ;;
        "G2O")               G2O                                             ;;
        "KDL")               KDL                                             ;;
        "NLopt")             NLopt                                           ;;
        "OMPL")              OMPL                                            ;;
        "SuitSparse")        SuitSparse                                      ;;
        "DynamixelSDK")      DynamixelSDK                                    ;;
        "KinovaAPI")         KinovaAPI                                       ;;
        "SOEM")              SOEM                                            ;;
        "Hokuyo")            Hokuyo                                          ;;
        "IntelRealSense")    IntelRealSense                                  ;;
        "XeThruRadarAPI")    XeThruRadarAPI                                  ;;
        "JSON")              JSON                                            ;;
        "Libconfig")         Libconfig                                       ;;
        "Protobuf")          Protobuf                                        ;;
        "TinyXML")           TinyXML                                         ;;
        "XML2")              XML2                                            ;;
        "cURL")              cURL                                            ;;
        "FreeTDS")           FreeTDS                                         ;;
        "MySQLClient")       MySQLClient                                     ;;
        "LZ4")               LZ4                                             ;;
        "Modbus")            Modbus                                          ;;
        "ODBC")              ODBC                                            ;;
        "Restbed")           Restbed                                         ;;
        "ZMQ")               ZMQ                                             ;;
        "OpenCV")            OpenCV                                          ;;
        "ViSP")              ViSP                                            ;;
        "x264")              x264                                            ;;
        "x265")              x265                                            ;;
        "ZBar")              ZBar                                            ;;
        "FCL")               FCL                                             ;;
        "Libccd")            Libccd                                          ;;
        "Octomap")           Octomap                                         ;;
        "PCL")               PCL                                             ;;
        "Matplotlib")        Matplotlib                                      ;;
        "QGLViewer")         QGLViewer                                       ;;
        "Qt5")               Qt5                                             ;;
        "VTK")               VTK                                             ;;
        *)                   printf. "The library $1 is not available \n"    ;;
    esac
}

InstallAll() {
   Update
   BasicTools
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

