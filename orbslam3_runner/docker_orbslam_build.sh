#!/bin/bash
#
# WARNING: Do not run this script locally, it installs packages in a very unclean manner.

set -euo pipefail

apt update -y
apt install -y git
apt install -y cmake

export DEBIAN_FRONTEND=noninteractive
TZ=Etc/UTC apt install -yq tzdata

apt install -y libopencv-dev
apt install -y libboost-all-dev
apt install -y libeigen3-dev

# Pangolin and its dependencies.
apt install -y libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev libc++-dev libglew-dev libeigen3-dev cmake g++
cd /
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir target
cd target
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make -j8
make install

cd /
git clone https://github.com/AaltoML/vio_benchmark.git
cd vio_benchmark
git submodule update --init --recursive jsonl-recorder
git submodule update --init --recursive orbslam3_runner/ORB_SLAM3

cd orbslam3_runner
./build_dependencies.sh

mkdir -p target
cd target
cmake ..
make -j8

# For running the benchmark tools.
apt install -y python3-pip
pip3 install numpy
pip3 install matplotlib
