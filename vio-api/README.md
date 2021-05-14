# Vio API draft and integrations

This contains a draft of a C++ API for Vio implementation and examples how any such implementation can be easily integrated with ROS framework or RealSense SDK.

TODO: Add an example implementation using Basalt.

## How to

1. Add mobile-cv-suite to root level via symbolic link or just cloning the repo:
```
git clone https://github.com/AaltoML/mobile-cv-suite
```
2. All examples follow same approach. You must add your own Vio implementation, here `custom-vio` to `CMakeLists.txt` and you are good to go.

## Example: cli

1. Create target folder: `mkdir target && cd target`
2. Build example: `cmake .. && make -j8`
3. Run euroc-mh-01-easy dataset, asuming it's in `data` folder in format supported by `jsonl-recorder`:
```
./main \
    ./data/euroc-mh-01-easy/config.json \
    ./data/euroc-mh-01-easy/calibration.json \
    ./data/euroc-mh-01-easy/data.jsonl \
    ./data/euroc-mh-01-easy/data.mp4 \
    ./data/euroc-mh-01-easy/data2.mp4
```

## Example: cli_visual

Same as `cli` example with addition of visualizations, if your `custom-vio` supports such. Allows picking either `VIDEO`, `POSE` or both and displays them.

# Example: realsense

Integrates with RealSense SDK and runs `custom-vio` real time on connected Intel RealSense T265 device. Uses same visualizations as shown in `cli_visual` example.

Make sure to connect T265 to a blue USB3 port. USB2 ports will hang after short period.

1. Symlink librealsense `ln -s /your/librealsense/path/here librealsense` and build it.
2. Create target folder: `cd cli_visual && mkdir target && cd target`
3. Build example: `cmake .. && make -j8`
4. Run with RealSense T265 attached:
```
./main \
    ./realsense_config.json \
    ./realsense_calibration.json \
    both
```

# Example: ros-vio

See `ros-vio/README.md`.

# Example: ros-realsense

See `ros-realsense/README.md`.
