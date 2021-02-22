# VIO Benchmark

A collection of Python scripts for benchmarking Visual-Inertial Odometry (VIO) solutions.

These scripts are best used together with data collection apps which output the used [JSONL data format](#jsonl-format), such as the following:

* [u-blox C099-F9P](https://github.com/AaltoML/u-blox-capture)
* [Intel RealSense Tracking Camera T265](https://github.com/AaltoVision/realsense-capture)
* [MYNT EYE S](https://github.com/AaltoVision/mynt-capture)
* [Android](https://github.com/AaltoML/android-viotester)

some of which use [jsonl-recorder](https://github.com/AaltoVision/jsonl-recorder).

## benchmark.py

### Usage

`benchmark.py` is a library for running multiple parallel benchmarks. It's not a stand alone script. See `benchmark/example_benchmark.py` on how to use it.

### Running benchmarks sets

To run benchmark sets, use the option `-set name_of_the_set`. The benchmark sets are define in json files in current directory or in given `-setDir <folder>`.

```
python benchmark/example_benchmark.py -set example
```

Following example benchmark set will run each dataset in "benchmarks" array once for every entry in "parameterSets", meaning you will get 6 x 2 = 12 benchmarks ran in total. You can use same dataset with different parameters as seen with the last 4 datasets. "parameterSets" is optional. All "params" flags are appended. If you run a benchmark with flags `-set example -params "-something=true"`, the full arguments for dataset euroc-mh-01-vR-01 and first parameter set would be `-something=true -customParam1=0.0 -customParam2=0.01`.

```
{
    "benchmarks": [
        {"folder": "advio-01"},
        {"folder": "advio-02"},
        {"folder": "euroc-mh-01-easy", "params": "-customParam1=0.01", "name": "euroc-mh-01-vR-01"},
        {"folder": "euroc-mh-01-easy", "params": "-customParam1=0.05", "name": "euroc-mh-01-vR-05"},
        {"folder": "euroc-mh-02-easy", "params": "-customParam1=0.01", "name": "euroc-mh-02-vR-01"},
        {"folder": "euroc-mh-02-easy", "params": "-customParam1=0.05", "name": "euroc-mh-02-vR-05"}
    ],
    "parameterSets": [
        {"params": "-customParam2=0.0", "name": "filterOff"},
        {"params": "-customParam2=0.1", "name": "filterOn"}
    ]
}
```

### combine_jsonl.py

Script for combining JSONL data from multiple sources. Use `add_time_offset.py` to synchronize them first.

If you store you data in following layout:

```
folder/containing/all/datasets
                              /arcore
                                     /data.jsonl
                                     /info.jsonl
                              /arkit
                                    /data.jsonl
                                    ...
```

You can combine data into `arcore-combined` output folder with:

```
python -m vio_benchmark.benchmark.combine_jsonl \
    arcore \
    -root folder/containing/all/datasets/ \
    -output arcore-combined \
    -arkit arkit \
    -rtk rtkgps \
    -realsense realsense \
    -gps arkit
```

## Benchmark data format

Each dataset, ie a single continuous recording of sensor data, is made out of a single folder with the following contents:

* `data.jsonl` in the format described [below](#jsonl-format).
* `data.mp4` (or other video file extension) for monocular camera.
* (optional) `data2.mp4` second camera for stereo recordings.
* (optional) `parameters.txt` see [Parameters format](#parameters-format) below.

## JSONL format

This section defines the [JSONL](https://jsonlines.org/)-based data format that most scripts in this repository produce or consume. The format can include sensor data used as input and pose estimates produced by different VIO methods.

### Timestamps

Each JSONL line that describes temporal data should have a `time` field at the root using seconds as units. The whole JSONL file is preferably sorted by these timestamps in ascending order. The timestamps may begin from any value, including negative ones.

### Inertial sensors (IMU)

IMU and other "N-axis" sensors define a `sensor` field with sub-fields `type` and `values`. The units for `values` are m/s^2 for `accelerometer`, rad/s for `gyroscope`, μT for `magnetometer`, and K for `imuTemperature`.

### Camera frames

For each frame in `data.mp4`, and a perfectly synchronized `data2.mp4` in case of stereo, there is a single line with `frames` in the root whose array value lists outputs for each camera. The `number` field starts from 0 and increments by one per frame. `cameraInd` 0 refers to `data.mp4` and 1 to `data2.mp4`. `cameraParameters` follow the same format as in [Parameters format](#parameters-format) below, except that matrices given in `[[],[],[],[]]` are read as row-major.

### GNSS

Defines `gps` in the root with the fields `longitude`, `latitude`, and `altitude` (meters) in the [Geographic coordinate system](https://en.wikipedia.org/wiki/Geographic_coordinate_system). The field `accuracy` has no precise definition, but in most cases is a distance (meters) corresponding to a given confidence level.

### Ground truth and VIO output

Defines `groundTruth` or name of a VIO method in the root, with subfields `position` (`t`) and optionally `orientation` (`q`). Position is given in meters, with negative z-axis pointing along gravity. Orientation is given as a unit quaternion. Together these define a 4x4 matrix `T = [R(q), t; 0, 1]` that transforms homogeneous device coordinates `p_d` to world coordinates `p_w` by left-multiplication `p_w = T * p_d`.

### Example data.jsonl

```
{"groundTruth":{"position":{"x":-0.007567568216472864,"y":0.022782884538173676,"z":0.00817866250872612},"orientation":{"w":0.53464,"x":-0.15299,"y":-0.826976,"z":-0.082863}},"time":1.444770263671875}
{"sensor":{"type":"accelerometer","values":[-0.03824593499302864,9.121655464172363,-2.983182907104492]},"time":1.43846240234375}
{"sensor":{"type":"gyroscope","values":[0.003195890923961997,-0.17364339530467987,0.015979453921318054]},"time":1.44976953125}
{"sensor":{"type":"imuTemperature","values":[292.8961]},"time":1.44976953125}
{"groundTruth":{"position":{"x":-0.007713410072028637,"y":0.022989293560385704,"z":0.008272084407508373},"orientation":{"w":0.53464,"x":-0.15299,"y":-0.826976,"z":-0.082863}},"time":1.449769287109375}
{"frames":[{"cameraInd":0,"cameraParameters":{"focalLengthX":284.929992675781,"focalLengthY":285.165496826172,"principalPointX":416.4547119140625,"principalPointY":395.77349853515625,"distortionModel":"KANNALA_BRANDT4","distortionCoefficients":[-0.004973,0.03975,-0.0374,0.006239]},"imuToCamera":[[0.01486,0.9995,-0.02577,0.06522],[-0.9998,0.01496,0.003756,-0.0207],[0.00414,0.02571,0.9996,-0.008054],[0,0,0,1]],"time":1.436207275390625},{"cameraInd":1,"cameraParameters":{"focalLengthX":284.559509277344,"focalLengthY":284.4418029785162,"principalPointX":410.81329345703125,"principalPointY":394.1506042480469,"distortionModel":"KANNALA_BRANDT4","distortionCoefficients":[-0.006496,0.04365,-0.04025,0.006813]},"imuToCamera":[[0.01255,0.9995,-0.02538,-0.0449],[-0.9997,0.01301,0.0179,-0.02056],[0.01822,0.02515,0.9995,-0.008638],[0,0,0,1]],"time":1.436207275390625}],"number":28,"time":1.436207275390625}
{"sensor":{"type":"gyroscope","values":[-0.025567127391695976,-0.16512103378772736,0.034089501947164536]},"time":1.4547685546875}
{"sensor":{"type":"imuTemperature","values":[292.8961]},"time":1.4547685546875}
{"groundTruth":{"position":{"x":-0.00783445406705141,"y":0.02315470017492771,"z":0.008362464606761932},"orientation":{"w":0.53464,"x":-0.15299,"y":-0.826976,"z":-0.082863}},"time":1.45476806640625}
{"sensor":{"type":"gyroscope","values":[-0.07137490063905716,-0.1523374617099762,0.04793836548924446]},"time":1.45976806640625}
{"sensor":{"type":"imuTemperature","values":[292.8961]},"time":1.45976806640625}
{"groundTruth":{"position":{"x":-0.007959198206663132,"y":0.023323576897382736,"z":0.008457313291728497},"orientation":{"w":0.53464,"x":-0.15299,"y":-0.826976,"z":-0.082863}},"time":1.459767333984375}
{"sensor":{"type":"accelerometer","values":[0.4015823304653168,9.446745872497559,-2.8301992416381836]},"time":1.4539404296875}
{"sensor":{"type":"gyroscope","values":[-0.12037855386734009,-0.12037855386734009,0.07457078248262405]},"time":1.46476708984375}
{"sensor":{"type":"imuTemperature","values":[292.8961]},"time":1.46476708984375}
{"groundTruth":{"position":{"x":-0.008127331733703613,"y":0.023527292534708977,"z":0.008570007979869843},"orientation":{"w":0.53464,"x":-0.15299,"y":-0.826976,"z":-0.082863}},"time":1.464766357421875}
{"frames":[{"cameraInd":0,"cameraParameters":{"focalLengthX":284.929992675781,"focalLengthY":285.165496826172,"principalPointX":416.4547119140625,"principalPointY":395.77349853515625,"distortionModel":"KANNALA_BRANDT4","distortionCoefficients":[-0.004973,0.03975,-0.0374,0.006239]},"imuToCamera":[[0.01486,0.9995,-0.02577,0.06522],[-0.9998,0.01496,0.003756,-0.0207],[0.00414,0.02571,0.9996,-0.008054],[0,0,0,1]],"time":1.46955859375},{"cameraInd":1,"cameraParameters":{"focalLengthX":284.559509277344,"focalLengthY":284.4418029785162,"principalPointX":410.81329345703125,"principalPointY":394.1506042480469,"distortionModel":"KANNALA_BRANDT4","distortionCoefficients":[-0.006496,0.04365,-0.04025,0.006813]},"imuToCamera":[[0.01255,0.9995,-0.02538,-0.0449],[-0.9997,0.01301,0.0179,-0.02056],[0.01822,0.02515,0.9995,-0.008638],[0,0,0,1]],"time":1.46955859375}],"number":29,"time":1.46955859375}
{"sensor":{"type":"gyroscope","values":[-0.15979453921318054,-0.08735434710979462,0.08841964602470398]},"time":1.4697841796875}
{"sensor":{"type":"imuTemperature","values":[292.8961]},"time":1.4697841796875}
{"groundTruth":{"position":{"x":-0.007131610997021198,"y":0.022098174318671227,"z":0.008283869363367558},"orientation":{"w":0.53464,"x":-0.15299,"y":-0.826976,"z":-0.082863}},"time":1.469782958984375}
{"sensor":{"type":"gyroscope","values":[-0.18323107063770294,-0.06178722158074379,0.09481143206357956]},"time":1.474782958984375}
{"gps": {"accuracy": 4.0, "altitude": 14.18831106834269, "latitude": 60.173783793064594, "longitude": 24.906486344581662}, "time":1.474782958984375}
```

## Parameters format

Most of the conversion scripts additionally produce a `parameters.txt` file that stores session-wide constants and other hints for using the data as input for a VIO method.

The parameter names are separated from the values by space and parameters are separated by `;`, which may be followed by whitespace including newlines.

The parameters are:

* `focalLengthX`: first camera horizontal focal length.
* `focalLengthY`: first camera vertical focal length.
* `principalPointX`: first camera horizontal principal point (pixels).
* `principalPointY`: first camera vertical principal point (pixels).
* `secondFocalLengthX`, `secondFocalLengthY`, `secondPrincipalPointX`, `secondPrincipalPointY`: same but for the second camera.
* `fisheyeCamera` (boolean): if `true` `distortionCoeffs` define the first four [Kannala-Brandt model](http://www.ee.oulu.fi/~jkannala/calibration/Kannala_Brandt_calibration.pdf) parameters, if `false` they define [OpenCV radial distortion model](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) (only the k1, k2, k3 parameters).
* `distortionCoeffs`: array of 3 or 4 values depending on `fisheyeCamera`.
* `secondDistortionCoeffs`: same but for the second camera.
* `imuToCameraMatrix`: describing pose difference of the IMU and camera component, lists a column-major representation of 4x4 matrix that transforms homogeneous coordinates as: `p_camera = T * p_imu`.
* `secondImuToCameraMatrix`: same but for the second camera.
* `matchStereoIntensities`: when `true`, a hint that the stereo cameras are not visually synchronized, eg due to differing exposure timings.

### Example parameters.txt

```
focalLengthX 458.654;focalLengthY 457.296;principalPointX 367.215;principalPointY 248.375;
distortionCoeffs -0.28340811,0.07395907,0.00019359;
secondFocalLengthX 457.587;secondFocalLengthY 456.134;
secondPrincipalPointX 379.999;secondPrincipalPointY 255.238;
secondDistortionCoeffs -0.28368365,0.07451284,-0.00010473;
imuToCameraMatrix 0.01486554298179427,-0.9998809296985752,0.004140296794224038,0.0,0.9995572490083462,0.01496721332471924,0.025715529947966016,0.0,-0.02577443669744028,0.0037561883579669726,0.9996607271779023,0.0,0.06522290953553112,-0.02070638549271943,-0.008054602460029517,1.0;
secondImuToCameraMatrix 0.012555267089102956,-0.9997550997231162,0.018223771455443325,0.0,0.999598781151433,0.013011905181503854,0.02515883631155237,0.0,-0.025389800891746528,0.01790058382525125,0.999517347077547,0.0,-0.04490198068250875,-0.020569771258915234,-0.008638135126028098,1.0;
```
