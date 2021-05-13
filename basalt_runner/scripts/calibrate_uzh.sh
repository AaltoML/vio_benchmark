#!/bin/bash
# Based on <https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md#uzh-dataset>
set -euo pipefail

CAM_TYPE=kb4
OUT_DIR=results/uzh

target/basalt_calibrate --dataset-path download/uzh/indoor_forward_calib_snapdragon_cam.bag --dataset-type bag --aprilgrid basalt-mirror/data/aprilgrid_5x4_uzh.json --result-path "$OUT_DIR" --cam-types $CAM_TYPE $CAM_TYPE

target/basalt_calibrate_imu \
  --dataset-path download/uzh/indoor_forward_calib_snapdragon_imu.bag \
  --dataset-type bag \
  --aprilgrid basalt-mirror/data/aprilgrid_5x4_uzh.json \
  --result-path "$OUT_DIR" \
  --gyro-noise-std 0.05 \
  --accel-noise-std 0.1 \
  --gyro-bias-std 4e-5 \
  --accel-bias-std 0.002

target/calibration_converter \
  --calib-path "$OUT_DIR"/calibration.json \
  --output-path "$OUT_DIR"/calibration_conv.json \
