# Simple CLI to Basalt

Simple CLI tool to run [Basalt](https://vision.in.tum.de/research/vslam/basalt) with standard JSONL formats presetnted in this repo.

## Build

You don't need to build Basalt separately, it's included in the step 2.

1. Install dependencies `cd basalt-mirror && ./scripts/install_deps.sh && cd ..`
2. Build with `mkdir target && cd target && cmake .. && make -j8`.

## Basalt Runner

Finally run the benchmark, for example `python run_benchmark.py euroc-mh-01-easy -dir ~/euroc_benchmarks/ -calib ./example/euroc_kb4_calibration.json -config ./basalt-mirror/data/euroc_config.json`, asuming your `EuroC` data is in `~/euroc_benchmarks` directory in converted format.

## Basalt Calibration

As an example, here are steps to calibrate Intel RealSense Tracking Camera T265 device using [RealSense capture](https://github.com/AaltoVision/realsense-capture) tool to record data from the device. Same steps apply for any `jsonl-recorder` format compatible device.

1. Download [Aprilgrid 6x6 0.8x0.8 m (unscaled)](https://github.com/ethz-asl/kalibr/wiki/downloads)
2. Record session with RealSense capture using screen, TV or printed out test pattern as target. Store it to `../data/realsense-calib/calib/`, asuming you are in `basalt_runner/target` directory.
3. Measure Apriltag dimensions. For this pattern, it's easiest to measure the edge length of the full thing consisting of 7 small spacer squares (of size 0.3a) and 6 apriltags and **divide by** 6 + 0.3 * 7 = **8.1**
4. Write out your measurements to tagSize field  in `../data/realsense-calib/aprilgrid_6x6.json`, for example with 66.5cm you get .665 / 8.1 = 0.082...:
```
{
  "tagCols": 6,
  "tagRows": 6,
  "tagSize": 0.08209876543,
  "tagSpacing": 0.3
}
```
5. Rename `jsonl` file to `data.jsonl`
6. Convert videos to png files:
```
mkdir video && mkdir video2
ffmpeg -i recording-2021-03-08T09-11-23Z-video.avi -vsync 0 "video/frame_%05d.png"
ffmpeg -i recording-2021-03-08T09-11-23Z-video2.avi -vsync 0 "video2/frame_%05d.png"
```
7. Now we are ready for calibration. Detailed instructions can be found at [Basalts documentation](https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md). tl;dr press buttons on the left from top to bottom when calibrating both cameras and imu
8. Calibrate cameras. It's recommded that in `optimize` step you use `opt_until_converge` to ensure you reach converge without mouse finger injury. Keep the command line window open so you can see what's going on.
```
./basalt_calibrate \
	--dataset-path ../data/realsense-calib \
	--dataset-type jsonl \
	--aprilgrid ../data/realsense-calib/aprilgrid_6x6.json \
	--result-path ../data/realsense-calib/calib/ \
	--cam-types ds ds
```
9. Calibrate IMU.
```
./basalt_calibrate_imu \
	--dataset-path ../data/realsense-calib \
	--dataset-type jsonl \
	--aprilgrid ../data/realsense-calib/aprilgrid_6x6.json \
	--result-path ../data/realsense-calib/calib/ \
	--gyro-noise-std 0.000282 \
	--accel-noise-std 0.016 \
	--gyro-bias-std 0.0001 \
	--accel-bias-std 0.001
```

10. Finally converting from serialized Basalt json format into more generalized json:
```
./calibration_converter \
	--calib-path ../data/realsense-calib/calib/calibration.json \
	--output-path ../example/realsense_kb4_calibration.json
```

## License

Some files in this folder are copied/modified from Basalt and are licensed under BSD 3-Clause License, see `basalt-mirror/LICENSE`.
