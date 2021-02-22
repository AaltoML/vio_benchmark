# Notes on using Kalibr

Github: https://github.com/ethz-asl/kalibr

* No Mac support
* Building in latest ubuntu docker image seemed complex, requires ROS installation etc.
* Building in latest ROS docker image also didn't seem straightforward and failed
* Using existing prebuilt docker image with Kalibr seemed best "out of the box" solution

## File layout

Following example except this file layout. Only lists important files. `~/kalbir/device_name` can be folder of your choosing, you can think of it as a placeholder. Filenames must be exactly these, otherwise scripts won't find them.

    ~/kalibr/
        device_name/
            dynamic_calibration_raw/
                data.avi                 # Filename must be data.[avi|mp4|mov]
                data2.avi                # If data is stereo i.e. optional
                data.jsonl
            stationary_calibration_raw/
                data.jsonl
            converted/
                cam0/
                imu0.csv
            allan/
                imu.yaml
        data.bag
        april_6x6_80x80cm.yaml
        camchain-data.yaml
        camchain-imucam-data.yaml

## Step 1. Collecting calibration data

* Create empty directory `~/kalibr/device_name`
* Download target pdf and companying yaml file: https://github.com/ethz-asl/kalibr/wiki/downloads (Aprilgrid 6x6 0.8x0.8 m (unscaled) recommended)
* Open the target full screen on your computer and use max brightness on the display if possible
* Measure the width of one apriltag with ruler and fix the value for `tagSize` in `april_6x6_80x80cm.yaml`, see this for more info how it's measured https://github.com/ethz-asl/kalibr/wiki/calibration-targets
* For this pattern, it's easiest to measure the edge length of the full thing consisting of 7 small spacer squares (of size 0.3a) and 6 apriltags and **divide by** 6 + 0.3 * 7 = **8.1**
* Record calibration data in Data Collection mode, moving in all directions etc. you know the drill
* Put data to `camera_calibration_raw/` folder

## Step 2. Collecting imu calibration data (Optional)

You can also try to find manufacturer provided values for this. File format looks like this:

    accelerometer_noise_density: 0.00074562202949377
    accelerometer_random_walk: 0.0011061605306550387
    gyroscope_noise_density: 3.115084637301622e-05
    gyroscope_random_walk: 1.5610557862757885e-05
    rostopic: /imu0
    update_rate: 500

If you do have the values already, write them to `allan/imu.yaml` file and skip to Step 3.

Camera recording can be disabled since only IMU is used. We don't have option for that yet in the app.

* Collect 2-4 hour session(?) of the device stationary. It may move during first and last 10 seconds, so you can turn it on and off without messing up the results.
* Put data to `imu_calibration_raw/` folder

## Step 3. Converting the data for Kalibr

Dynamic data conversion:

    python convert/jsonl-to-kalibr.py ~/kalibr/device_name/dynamic_calibration_raw -output ~/kalibr/device_name/converted/

Optional stationary IMU conversion:

    python convert/jsonl-to-kalibr.py ~/kalibr/device_name/stationary_calibration_raw -output ~/kalibr/device_name/allan -stationary

## Step 4. Running Kalibr

Use Docker image from `stereolabs` to run prebuilt Kalibr. Following command downloads the image, maps host `~/kalbir` to vm `/kalibr` folder and starts shell.

    docker run -v ~/kalibr:/kalibr -it stereolabs/kalibr:kinetic

The dynamic camera data has to be converted to rosbag format, this can take a while:

    cd /kalibr/device_name
    kalibr_bagcreater --folder converted --output-bag data.bag

If you are working with monocular data, we need to disable an unnecessary check:

    apt-get install nano
    nano /kalibr_workspace/src/Kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras

Comment out graph check, unless you have multiple cameras it fails, but isn't actually required:

        #if not self.isGraphConnected():
        #    sm.logError("The cameras are not connected through mutual target observations! "
        #                "Please provide another dataset...")
        #
        #    self.plotGraph()
        #    sys.exit(0)

Now calibrate camera parameters, you can ignore the errors at the end caused by missing display. See https://github.com/ethz-asl/kalibr/wiki/supported-models for supported models.

Mono:

    kalibr_calibrate_cameras --bag data.bag \
        --topics /cam0/image_raw \
        --models pinhole-radtan \
        --target april_6x6_80x80cm.yaml \
        --dont-show-report

Stereo:

    kalibr_calibrate_cameras --bag data.bag \
        --topics /cam0/image_raw /cam1/image_raw \
        --models pinhole-radtan pinhole-radtan \
        --target april_6x6_80x80cm.yaml \
        --dont-show-report

Next calibrate camera-IMU

    kalibr_calibrate_imu_camera --bag data.bag --cams camchain-data.yaml --target april_6x6_80x80cm.yaml --imu allan/imu.yaml  --dont-show-report
