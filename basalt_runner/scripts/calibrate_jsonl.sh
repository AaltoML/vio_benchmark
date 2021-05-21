#!/bin/bash
set -euo pipefail

DATA=$1
CAM_TYPE=kb4
OUT_DIR=results/jsonl

target/basalt_calibrate \
  --dataset-path "$1" \
  --dataset-type jsonl \
  --aprilgrid "$1"/aprilgrid.json \
  --result-path "$OUT_DIR" \
  --cam-types $CAM_TYPE $CAM_TYPE

target/basalt_calibrate_imu \
  --dataset-path "$1" \
  --dataset-type jsonl \
  --aprilgrid "$1"/aprilgrid.json \
  --result-path "$OUT_DIR"

target/calibration_converter \
  --calib-path "$OUT_DIR"/calibration.json \
  --output-path "$OUT_DIR"/calibration_conv.json \
