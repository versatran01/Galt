#!/bin/sh

# Install script for the Ocean Optics OmniDriver

if [ "$(id -u)" != "0" ]; then
   echo "This script must be run with sudo" 1>&2
   exit 1
fi

# directory to install into
INSTALL_BASE="/opt"

# uncompress archives
mkdir -pv $INSTALL_BASE

echo "Installing OmniDriver to $INSTALL_BASE"
tar -pxf OmniDriver.tar.gz --overwrite -C $INSTALL_BASE

echo "Installing JDK to $INSTALL_BASE"
tar -pxf jdk1.6.0_01.tar.gz --overwrite -C $INSTALL_BASE

echo "Installing rules to /etc/udev/rules.d"
cp $INSTALL_BASE/OmniDriver/drivers/10-oceanoptics.rules /etc/udev/rules.d/
service udev restart

# configure environment variables
bashrc_path=`echo ~/.bashrc`
echo "Updating $bashrc_path"

echo "export OMNIDRIVER_HOME=\"$INSTALL_BASE/OmniDriver\"" >> $bashrc_path
echo "export OOI_HOME=\"$INSTALL_BASE/OmniDriver/OOI_HOME\"" >> $bashrc_path
echo "export PATH=\$PATH:\$OOI_HOME" >> $bashrc_path
echo "export JDK_6_PATH=$INSTALL_BASE/jdk1.6.0_01" >> $bashrc_path

echo "Done."
