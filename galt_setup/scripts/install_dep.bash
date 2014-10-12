#!/usr/bin/env bash

# This file will install depencies which doesn't have pre-built binaries

CURRENT_DIR=$(pwd)
TMP_DIR=/tmp

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

