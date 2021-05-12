#!/bin/bash
set -euo pipefail

CAM_TYPE=kb4
OUT_DIR=results/euroc

target/basalt_calibrate --dataset-path download/euroc/cam_april.bag --dataset-type bag --aprilgrid basalt-mirror/data/aprilgrid_6x6.json --result-path "$OUT_DIR" --cam-types $CAM_TYPE $CAM_TYPE

target/basalt_calibrate_imu \
  --dataset-path download/euroc/imu_april.bag \
  --dataset-type bag \
  --aprilgrid basalt-mirror/data/aprilgrid_6x6.json \
  --result-path "$OUT_DIR" \
  --gyro-noise-std 0.000282 \
  --accel-noise-std 0.016 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001

target/calibration_converter \
  --calib-path "$OUT_DIR"/calibration.json \
  --output-path "$OUT_DIR"/calibration_conv.json \
