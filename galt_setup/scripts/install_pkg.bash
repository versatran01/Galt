#!/usr/bin/env bash

# This file will install all dependencies from apt-get
PACKAGES=(
autojump
cmake
curl
dstat
git
gitg
git-review
guake
htop
meld
openssh-server
pkg-config
python-matplotlib
python-numpy
python-pip
python-scipy
tig
tmux
vim
vim-gtk
)

#Stop if a command ends in an error
set -e

#Check to see if we are running with root privileges
if [[ $(id -u) -ne 0 ]] ; then
    echo "Please run this script as root (eg using sudo)"
    exit 1
fi

echo "Making sure all system software is up to date."
apt-get update --force-yes
apt-get upgrade --force-yes

#Intall packages
apt-get install ${PACKAGES[@]}
