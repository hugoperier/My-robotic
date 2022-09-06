#!/bin/sh
#################################################################
## Copy all configuration file from the provided folder into the .myrobotics folder
## and create it if it does not exist yet.
## Usage: install_configuration.sh <configuration folder>
#################################################################

MYROBOTICS_DIR="$HOME/.myrobotics"

if [ $# -eq 0 ]
  then
    echo "Usage: install_configuration.sh <configuration folder>"
    exit 1
fi

# verify that the argument is a folder
if [ ! -d "$1" ]; then
  echo "The given argument must be a folder"
  exit 1
fi

if [ ! -d $MYROBOTICS_DIR ]; then
    mkdir $MYROBOTICS_DIR
    echo "Folder not existing... creating $MYROBOTICS_DIR"
fi

cp -r $1/* $MYROBOTICS_DIR

echo "Configuration files copied:"
find $1 -type f
