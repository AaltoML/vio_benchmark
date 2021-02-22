#!/usr/bin/python
#
# Convert data from <https://fpv.ifi.uzh.ch> “The UZH FPV Dataset”.
# Paper: <https://www.zora.uzh.ch/id/eprint/197739/1/ICRA19_Delmerico.pdf>
#
# Usage:
# * Download raw data from the website. Choose Snapdragon, ZIP version.
# * Extract the zips into folder. I've used the following naming scheme:
#   * "indoor/outdoor" -> "in/out"
#   * "forward/45" => "fwd/down"
#   * Use the original number and omit rest.
# * Download calibration data from the website (the small zip files).
# * Edit the global variables at the top of the script.
# * Run `python scripts/convert/uzh_fpv_to_benchmark.py`.

import csv
import json
import os
from pathlib import Path
import re
import subprocess
import yaml

TEST_SET = False
if TEST_SET:
    # <https://fpv.ifi.uzh.ch/2019-2020-submission-format/>
    # An online competition with no ground truth.
    HAS_GROUND_TRUTH = False
    RAW = "data/raw/uzh-fpv-test"
    OUT = "data/benchmark/uzh-fpv-test"
    CALIB = "data/raw/uzh-fpv"
    # Use original verbose names because of the submission format.
    DATASETS = [
      "indoor_45_16_snapdragon",
      "indoor_45_3_snapdragon",
      "indoor_forward_11_snapdragon",
      "indoor_forward_12_snapdragon",
      "outdoor_forward_10_snapdragon",
      "outdoor_forward_9_snapdragon",
    ]
else:
    # All Snapdragon sets with available ground truth.
    HAS_GROUND_TRUTH = True
    RAW = "data/raw/uzh-fpv"
    OUT = "data/benchmark/uzh-fpv"
    CALIB = RAW
    DATASETS = [
        "indown-02",
        "indown-04",
        "indown-09",
        "indown-12",
        "indown-13",
        "indown-14",
        "infwd-03",
        "infwd-05",
        "infwd-06",
        "infwd-07",
        "infwd-09",
        "infwd-10",
        "outdown-01",
        "outfwd-01",
        "outfwd-03",
        "outfwd-05",
    ]

def convertVideo(files, output):
    fps="30" # 30 Hz for Snapdragon, 50 Hz for Davis.
    subprocess.run(["ffmpeg",
        "-y",
        "-r", fps,
        "-f", "image2",
        "-start_number", "0", "-i", files,
        "-c:v", "libx264",
        # "-preset", "ultrafast",
        "-preset", "veryslow",
        "-crf", "0", # lossless compression
        "-vf", "format=yuv420p",
        "-an",
        output])

def calibrationFile(dataset):
    if TEST_SET:
        if dataset.startswith("indoor_45"):
            folder = "{}/indown-calibration".format(CALIB)
        elif dataset.startswith("indoor_forward"):
            folder = "{}/infwd-calibration".format(CALIB)
        elif dataset.startswith("outdoor_45"):
            folder = "{}/outdown-calibration".format(CALIB)
        elif dataset.startswith("outdoor_forward"):
            folder = "{}/outfwd-calibration".format(CALIB)
        else:
            raise Exception("Could not determine calibration path.")
    else:
        folder = "{}/{}-calibration".format(CALIB, dataset.split("-")[0])

    for filename in os.listdir(folder):
        if filename.startswith("camchain-imucam"):
            return "{}/{}".format(folder, filename)
    raise Exception("Failed to find calibration YAML file.")

def getTimestamps(path):
    images = {}
    with open(path) as imageFile:
        imageFile.readline() # Skip header
        for line in imageFile.readlines():
            row = line.split()
            images[row[1]] = row[2]
    return images

def convert(dataset):
    print("---\nConverting {}\n---".format(dataset))

    rawPath = "{}/{}".format(RAW, dataset)
    outPath = "{}/{}".format(OUT, dataset)
    Path(outPath).mkdir(parents=True, exist_ok=True)

    timestamps = []
    # Use images that are present for both cameras. Needed for at least the "infwd-10" case.
    # Rename bad files so that they do not match glob `*.png` given for ffmpeg.
    timestamps0 = getTimestamps("{}/left_images.txt".format(rawPath))
    timestamps1 = getTimestamps("{}/right_images.txt".format(rawPath))
    for t, f0 in timestamps0.items():
        if t not in timestamps1.keys():
            f = "{}/{}".format(rawPath, f0)
            os.rename(f, f + "_hdn")
            print("{}: {} not found in cam1, ignoring".format(dataset, t))
        else:
            timestamps.append(t)
    for t, f0 in timestamps1.items():
        if t not in timestamps0.keys():
            f = "{}/{}".format(rawPath, f0)
            os.rename(f, f + "_hdn")
            print("{}: {} not found in cam0, ignoring".format(dataset, t))

    convertVideo("{}/img/image_0_%d.png".format(rawPath), "{}/data.mp4".format(outPath))
    convertVideo("{}/img/image_1_%d.png".format(rawPath), "{}/data2.mp4".format(outPath))

    # Reverse image name changes.
    imageDir = "{}/img".format(rawPath)
    for filename in os.listdir(imageDir):
        f = "{}/{}".format(imageDir, filename)
        m = re.search("(.+)_hdn", f)
        if m:
            os.rename(f, m.group(1))

    parameters = []
    intrinsics = []
    distortionCoeffs = []
    imuToCameras = []
    imuToCamerasTime = []
    with open(calibrationFile(dataset)) as yamlFile:
        try:
            data = yaml.load(yamlFile, Loader=yaml.FullLoader)
            for i in [0, 1]:
                cam = "cam{}".format(i)
                intrinsics.append(data[cam]["intrinsics"])
                parameters.append({
                    "focalLengthX": intrinsics[i][0],
                    "focalLengthY": intrinsics[i][1],
                    "principalPointX": intrinsics[i][2],
                    "principalPointY": intrinsics[i][3]
                })
                distortionCoeffs.append(data[cam]["distortion_coeffs"])
                # The text files (kalibr output?) clarify this is IMU-to-camera transform.
                imuToCameras.append(data[cam]["T_cam_imu"])
                # The text files define: "timeshift cam0 to imu0: [s] (t_imu = t_cam + shift)"
                # Change sign to get imuToCamera.
                imuToCamerasTime.append(-float(data[cam]["timeshift_cam_imu"]))
        except yaml.YAMLError as exc:
            print(exc)
            raise Exception("Failed to read camera parameters.")

    # The starting time is very large, shift timestamps to around zero to reduce floating point
    # accuracy issues.
    t0 = float(timestamps[0])

    output = []
    number = 0
    for timestamp in timestamps:
        t = float(timestamp) - t0
        output.append({
            "number": number,
            "time": t,
            "frames": [
                {"cameraInd": 0, "cameraParameters": parameters[0], "time": t},
                {"cameraInd": 1, "cameraParameters": parameters[1], "time": t},
            ],
        })
        number += 1

    with open(rawPath + '/imu.txt') as imuFile:
        imuFile.readline() # Skip header
        for line in imuFile.readlines():
            row = line.split()
            t = float(row[1]) - t0
            output.append({
                "sensor": {
                    "type": "gyroscope",
                    "values": [float(row[2]), float(row[3]), float(row[4])]
                },
                "time": t
            })
            output.append({
                "sensor": {
                    "type": "accelerometer",
                    "values": [float(row[5]), float(row[6]), float(row[7])]
                },
                "time": t
            })

    if HAS_GROUND_TRUTH:
        with open(rawPath + '/groundtruth.txt') as groundTruthFile:
            groundTruthFile.readline() # Skip header
            for line in groundTruthFile.readlines():
                # format: id timestamp tx ty tz qx qy qz qw
                row = line.split()
                t = float(row[1]) - t0
                output.append({
                    "groundTruth": {
                        "position": {
                            "x": float(row[2]), "y": float(row[3]), "z": float(row[4])
                        },
                        # NOTE As of February 2021, the website has this note:
                        # “Important: We have recently found an issue in the rotations provided in the ground truth, see the report. We are currently working on a corrected version of the ground truth. This does not affect ground truth positions.”
                        "orientation": {
                            "w": float(row[8]), "x": float(row[5]), "y": float(row[6]), "z": float(row[7])
                        }
                    },
                    "time": t,
                })

    # Write JSONL
    output = sorted(output, key=lambda row: row["time"]) # Sort by time
    with open(outPath + "/data.jsonl", "w") as f:
        for obj in output:
            f.write(json.dumps(obj, separators=(',', ':')))
            f.write("\n")

    # Write parameters
    with open(outPath + "/parameters.txt", "w") as f:
        f.write("focalLengthX {}; focalLengthY {};\nprincipalPointX {}; principalPointY {};\n".format(*intrinsics[0]))
        f.write("distortionCoeffs {},{},{},{};\n".format(*distortionCoeffs[0]))
        f.write("secondFocalLengthX {}; secondFocalLengthY {};\nsecondPrincipalPointX {}; secondPrincipalPointY {};\n".format(*intrinsics[1]))
        f.write("secondDistortionCoeffs {},{},{},{};\n".format(*distortionCoeffs[1]))
        f.write("fisheyeCamera true; rot 0;\n")
        for cam in [0, 1]:
            columnMajor = []
            for i in [0, 1, 2, 3]:
                for j in [0, 1, 2, 3]:
                    columnMajor.append(str(imuToCameras[cam][j][i]))
            f.write("{} {};\n".format(
                "imuToCameraMatrix" if cam == 0 else "secondImuToCameraMatrix",
                ",".join(columnMajor)))
        for cam in [0, 1]:
            f.write("{} {};\n".format(
                "imuToCameraShiftSeconds" if cam == 0 else "secondImuToCameraShiftSeconds",
                imuToCamerasTime[cam]))

def main():
    for dataset in DATASETS:
        convert(dataset)

if __name__ == "__main__":
    main()
