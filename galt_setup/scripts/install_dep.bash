#!/usr/bin/env bash

# This file will install depencies which doesn't have pre-built binaries

CURRENT_DIR=$(pwd)
TMP_DIR=/tmp

# gtsam
echo "Installing gtsam."
cd ${TMP_DIR}
GTSAM=gtsam-3.1.0
curl -o ${GTSAM}.zip https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam-3.1.0.zip
unzip ${GTSAM}.zip
cd ${GTSAM}
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo "Done installing gtsam."

# g2o
echo "Installing g2o."
sudo apt-get install libeigen3-dev libsuitesparse-dev libqt4-dev qt4-qmake libqglviewer-dev
G2O=g2o
cd ${TMP_DIR}
git clone https://github.com/RainerKuemmerle/g2o
cd ${G2O}
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo "Done installing g2o"

# LibGeographiclib
echo "Installing GeographicLib."
GEOGRAPHICLIB=GeographicLib-1.38
cd ${TMP_DIR}
curl -L http://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.38.zip  > ${GEOGRAPHICLIB}.zip
unzip ${GEOGRAPHICLIB}.zip
cd ${GEOGRAPHICLIB}
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo "Done installing GeoGraphicLib."

# mvIMAPCT Acquire
echo "Installing mvIMPACT Acquire."
bash ~/Workspace/repo/nouka/drivers/bluefox2/install/install.bash
echo "Done installign mvIMPACT Acquire."

# eBus SDK
echo "Installing eBus SDK."
bash ~/Workspace/repo/nouka/drivers/flir_gige/install/install.bash
echo "Done installign eBus SDK."

sudo usermod -a -G dialout galt

