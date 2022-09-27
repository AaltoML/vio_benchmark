#!/bin/bash

set -euo pipefail

cd vio_benchmark/orbslam3_runner/

python3 run_benchmark.py euroc-mh-01-easy \
	-dir /data \
	-vocab ./ORB_SLAM3/Vocabulary \
	-compare groundTruth,slam \
	-metricSet full_3d_align \
	-config ./ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml
