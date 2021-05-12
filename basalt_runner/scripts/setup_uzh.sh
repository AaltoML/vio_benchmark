#!/bin/bash
# <https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md#uzh-dataset>
set -euo pipefail

mkdir -p download/uzh
cd download/uzh
wget http://rpg.ifi.uzh.ch/datasets/uzh-fpv/calib/indoor_forward_calib_snapdragon_cam.bag
wget http://rpg.ifi.uzh.ch/datasets/uzh-fpv/calib/indoor_forward_calib_snapdragon_imu.bag
