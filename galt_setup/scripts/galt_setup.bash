#!/usr/bin/env bash

# Installs everything
sudo ./install_pkg.bash
sudo ./install_ros.bash
./install_repo.bash
./install_dep.bash
./setup_ws.bash
