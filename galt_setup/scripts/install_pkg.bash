#!/usr/bin/env bash

# This file will install all dependencies from apt-get
PACAKGES=(
autojump
bpython
clang
clang-format-3.5
cmake
curl
dstat
exuberant-ctags
git
gitg
git-review
guake
htop
libceres-dev
meld
pkg-config
python-matplotlib
python-numpy
python-pip
python-scipy
tig
tmux
valgrind
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
apt-get -y upgrade --force-yes

#Intall packages
apt-get -y --force-yes install ${PACKAGES[@]}
