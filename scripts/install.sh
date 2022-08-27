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

