# Custom Vio - Ros and Realsense integration

See `realsense` example on how to install `librealsense` and `ros-vio` example for instructions how to work with ROS framework. This is a hybrid of those two, where we interface directly with RealSense T265 and only publish resulting pose to ROS to achieve the best performance.

# Running

Make sure to connect T265 to a blue USB3 port. USB2 ports will hang after short period.

Start roscore where we can publish the messages:
```
roscore
```

In another terminal, start Custom Vio ROS integration:
```
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<insert repo path here>/mobile-cv-suite/build/host/lib rosrun ros-realsense realsense \
    _config:=<insert repo path here>/ros-realsense/config.jsonl \
    _calibration:=<insert repo path here>/ros-realsense/calibration.json
```

View the results in `rviz` for example, see `realsense` example for detailed instructions.
