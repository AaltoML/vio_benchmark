#!/bin/bash
# <https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md#euroc-dataset>
set -euo pipefail

mkdir -p download/euroc
cd download/euroc
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/calibration_datasets/cam_april/cam_april.bag
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/calibration_datasets/imu_april/imu_april.bag
