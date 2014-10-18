#!/usr/bin/env bash

# This file will setup a catkin workspace for compiling project Galt
CURRENT_DIR=$(pwd)
WS_DIR=~/Workspace/ws/galt_ws
SRC_DIR=src

if [[ ! -d ${WS_DIR} ]]; then
    mkdir -p ${WS_DIR}/src
    echo "Creating folder ${WS_DIR}."
fi

cd ${WS_DIR}/src
catkin_init_workspace

echo "Linking Galt and nouka"
if [[ ! -L Galt ]]; then
    ln -s ~/Workspace/repo/Galt .
fi
if [[ ! -L nouka ]]; then
    ln -s ~/Workspace/repo/nouka .
fi
cd ${WS_DIR}
catkin_make

