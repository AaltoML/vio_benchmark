# Custom Vio - Ros integration

## Intro

This example assumes you are running Raspberry Pi 4 and RealSense T265. These are not step-by-step instructions, but rough guidlines one what is required.

## Operating system

* Install Ubuntu 20.04.2 LTS Server edition using [Raspberry Pi Imager](https://www.raspberrypi.org/software/)
* Install desktop using `sudo apt install ubuntu-desktop`

## ROS

Install ROS Noetic using [instructions](http://wiki.ros.org/noetic/Installation/Ubuntu). Install recommended packages.

Instructions assume you source ROS env:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Create catkin workspace to your home dir:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

And then source that as well:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## RealSense

Build RealSense ROS integration from sources or use prebuilt packages [instructions here](https://github.com/IntelRealSense/realsense-ros)

## Building this ROS integration example

If you built RealSense integration from sources you already have `src` folder in catkin workspace, but if not let's create it

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```

Soft link ´ros-vio´ example folder to `catkin_ws/src/` folder and build it:
```
ln -s <insert repo path here>/ros-vio .
catkin_make
```

## Running

Make sure to connect T265 to a blue USB3 port. USB2 ports will hang after short period.

Start RealSense ROS, by default it doesn't stream cameras, so they are enabled via params:
```
roslaunch realsense2_camera rs_t265.launch enable_fisheye1:=true enable_fisheye2:=true
```

You can check that it's working with `rostopic list -v`, you should see a lot of topics.

Start Custom Vio ROS integration:
```
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<insert repo path here>/mobile-cv-suite/build/host/lib rosrun ros-vio vio \
    _config:=<insert repo path here>/ros-vio/config.jsonl \
    _calibration:=<insert repo path here>/ros-vio/calibration.json
```

You should see the output coordinates in stdout now.

To visualize the output:
* start Rviz with `rviz`
* Click "Add" button at bottom left, go to "By Topic" and select odom Pose
* In Displays menu, Global Options open Fixed Frame dropwn and select odom
* In Dsiplays menu, Pose, open Shape dropdown and select Axes
* You should now see the live pose of the Custom Vio output in the 3D view
