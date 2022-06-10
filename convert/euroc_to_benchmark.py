#!/usr/bin/python

# Downloads and converts EuRoC dataset to JSONL format.
# Requires installing pyyaml for yaml support. Install this with pip install pyyaml

import argparse
import os
import csv
import json
import os
from pathlib import Path
import re
import subprocess
import yaml
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("-pretty", help="Prints human readable json", action="store_true")
args = parser.parse_args()

LINK_PREFIX = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
TEMP_DIR = "data/raw/euroc"
OUTPUT_DIR ="data/benchmark"
DATASETS = [
    {"name": "euroc-mh-01-easy", "link": "machine_hall/MH_01_easy/MH_01_easy.zip"},
    {"name": "euroc-mh-02-easy", "link": "machine_hall/MH_02_easy/MH_02_easy.zip"},
    {"name": "euroc-mh-03-medium", "link": "machine_hall/MH_03_medium/MH_03_medium.zip"},
    {"name": "euroc-mh-04-difficult", "link": "machine_hall/MH_04_difficult/MH_04_difficult.zip"},
    {"name": "euroc-mh-05-difficult", "link": "machine_hall/MH_05_difficult/MH_05_difficult.zip"},
    {"name": "euroc-v1-01-easy", "link": "vicon_room1/V1_01_easy/V1_01_easy.zip"},
    {"name": "euroc-v1-02-medium", "link": "vicon_room1/V1_02_medium/V1_02_medium.zip"},
    {"name": "euroc-v1-03-difficult", "link": "vicon_room1/V1_03_difficult/V1_03_difficult.zip"},
    {"name": "euroc-v2-01-easy", "link": "vicon_room2/V2_01_easy/V2_01_easy.zip"},
    {"name": "euroc-v2-02-medium", "link": "vicon_room2/V2_02_medium/V2_02_medium.zip"},
    {"name": "euroc-v2-03-difficult", "link": "vicon_room2/V2_03_difficult/V2_03_difficult.zip"}
]

TO_SECONDS = 1000 * 1000 * 1000 # Timestamps are in nanoseconds

def convertVideo(files, output, fps):
    # Use `-crf 0` for lossless compression.
    fps=str(fps)
    subprocess.run(["ffmpeg",
        "-y",
        "-r", fps,
        "-f", "image2",
        "-pattern_type", "glob", "-i", files,
        "-c:v", "libx264",
        # "-preset", "ultrafast",
        "-preset", "veryslow",
        "-crf", "0",
        "-vf", "format=yuv420p",
        "-an",
        output])

def getCameraParameters(filename):
    with open(filename) as f:
        intrinsics = yaml.load(f, Loader=yaml.FullLoader)["intrinsics"]
        # Syntax: intrinsics: [458.654, 457.296, 367.215, 248.375] #fu, fv, cu, cv
        return {
            "focalLengthX": intrinsics[0],
            "focalLengthY": intrinsics[1],
            "principalPointX": intrinsics[2],
            "principalPointY": intrinsics[3]
        }
    raise Exception("Failed to read camera params")

def makeDataset(outputIn, tmpdir, name, t0, kind):
    assert(kind == "cam0" or kind == "cam1" or kind == "stereo")
    stereo = kind == "stereo"
    output = list(outputIn) # clone

    outputdir = OUTPUT_DIR + "/" + name
    if not stereo:
        outputdir = outputdir + "-" + kind
    Path(outputdir).mkdir(parents=True, exist_ok=True)

    if stereo:
        parameters0 = getCameraParameters("{}/mav0/cam0/sensor.yaml".format(tmpdir))
        parameters1 = getCameraParameters("{}/mav0/cam1/sensor.yaml".format(tmpdir))
    else:
        parameters0 = getCameraParameters("{}/mav0/{}/sensor.yaml".format(tmpdir, kind))

    timestamps = []
    if stereo:
        # Use images that are present for both cameras.
        # Rename bad files so that they do not match glob `*.png` given for ffmpeg.
        timestamps0 = []
        timestamps1 = []
        dir0 = "{}/mav0/cam0/data".format(tmpdir)
        dir1 = "{}/mav0/cam1/data".format(tmpdir)
        for filename in os.listdir(dir0):
            timestamps0.append(filename)
        for filename in os.listdir(dir1):
            timestamps1.append(filename)
        for t in timestamps0:
            if t not in timestamps1:
                f = "{}/{}".format(dir0, t)
                os.rename(f, f + "_hdn")
                print(t, "not found in cam1, ignoring")
            else:
                timestamps.append(int(os.path.splitext(t)[0]) / TO_SECONDS)
        for t in timestamps1:
            if t not in timestamps0:
                f = "{}/{}".format(dir1, t)
                os.rename(f, f + "_hdn")
                print(t, "not found in cam0, ignoring")
    else:
        # Use all images.
        for filename in os.listdir("{}/mav0/{}/data".format(tmpdir, kind)):
            timestamps.append(int(os.path.splitext(filename)[0]) / TO_SECONDS)

    timestamps = sorted(timestamps)
    number = 0
    for timestamp in timestamps:
        t = timestamp - t0
        x = {
            "number": number,
            "time": t
        }
        if stereo:
            x["frames"] = [
                {"cameraInd": 0, "cameraParameters": parameters0, "time": t},
                {"cameraInd": 1, "cameraParameters": parameters1, "time": t}
            ]
        else:
            x["frames"] = [
                {"cameraInd": 0, "cameraParameters": parameters0, "number": number, "time": t}
            ]
        output.append(x)
        number += 1

    # Video
    with open("{}/mav0/cam0/sensor.yaml".format(tmpdir)) as f:
        fps0 = yaml.load(f, Loader=yaml.FullLoader)["rate_hz"]
    if stereo:
        with open("{}/mav0/cam1/sensor.yaml".format(tmpdir)) as f:
            fps1 = yaml.load(f, Loader=yaml.FullLoader)["rate_hz"]

        convertVideo("{}/mav0/cam0/data/*.png".format(tmpdir), outputdir + "/data.mp4", fps0)
        convertVideo("{}/mav0/cam1/data/*.png".format(tmpdir), outputdir + "/data2.mp4", fps1)
    else:
        convertVideo("{}/mav0/{}/data/*.png".format(tmpdir, kind), outputdir + "/data.mp4", fps0)

    # Reverse image name changes.
    if stereo:
        dir0 = "{}/mav0/cam0/data".format(tmpdir)
        dir1 = "{}/mav0/cam1/data".format(tmpdir)
        for directory in [dir0, dir1]:
            for filename in os.listdir(directory):
                f = "{}/{}".format(directory, filename)
                m = re.search("(.+)_hdn", f)
                if m:
                    os.rename(f, m.group(1))

    # Write JSONL
    output = sorted(output, key=lambda row: row["time"]) # Sort by time
    with open(outputdir + "/data.jsonl", "w") as f:
        for obj in output:
            if args.pretty:
                f.write(json.dumps(obj, indent=4))
            else:
                f.write(json.dumps(obj, separators=(',', ':')))
            f.write("\n")

    # Write info
    with open(outputdir + "/info.json", "w") as f:
        f.write(json.dumps({
            "tags": ["euroc"]
        }, indent=4))

    # Write parameters
    with open(outputdir + "/parameters.txt", "w") as f, \
        open(tmpdir + '/mav0/cam0/sensor.yaml') as s0, \
        open(tmpdir + '/mav0/cam1/sensor.yaml') as s1:

        sensor0 = yaml.load(s0, Loader=yaml.FullLoader)
        sensor1 = yaml.load(s1, Loader=yaml.FullLoader)
        # Inverses of sensor.yaml matrices, column-major.
        m0 = ",".join(str(i) for i in np.linalg.inv(np.array(sensor0["T_BS"]["data"]).reshape((4,4))).T.reshape(16))
        m1 = ",".join(str(i) for i in np.linalg.inv(np.array(sensor1["T_BS"]["data"]).reshape((4,4))).T.reshape(16))
        if stereo:
            i0 = sensor0["intrinsics"]
            i1 = sensor1["intrinsics"]
            d0 = sensor0["distortion_coefficients"]
            d1 = sensor1["distortion_coefficients"]
        elif kind == "cam0":
            i0 = sensor0["intrinsics"]
            d0 = sensor0["distortion_coefficients"]
        elif kind == "cam1":
            i0 = sensor1["intrinsics"]
            d0 = sensor1["distortion_coefficients"]
            m0 = m1

        f.write("focalLengthX {};focalLengthY {};principalPointX {};principalPointY {};\n".format(*i0)
            + "distortionCoeffs {},{},{};\n".format(*d0))
        if stereo:
            f.write("secondFocalLengthX {};secondFocalLengthY {};\nsecondPrincipalPointX {};secondPrincipalPointY {};\n".format(*i1)
            + "secondDistortionCoeffs {},{},{};\n".format(*d1))
        f.write("rot 0;\nvideoRotation NONE;\n"
            + "imuToCameraMatrix {};\n".format(m0))
        if stereo:
            f.write("secondImuToCameraMatrix {};\n".format(m1))
            f.write("matchStereoIntensities true;\n")

def convert(dataset):
    name = dataset['name']
    link = dataset['link']
    output = []
    print("Converting " + name)

    # Setup
    tmpdir = TEMP_DIR + "/" + name
    Path(tmpdir).mkdir(parents=True, exist_ok=True)

    # Download
    subprocess.run(["wget", LINK_PREFIX + "/" + link, "-O", tmpdir + ".zip"])
    subprocess.run(["unzip", tmpdir + ".zip", "-d", tmpdir])
    for f in os.listdir(tmpdir):
        if f.endswith(".zip"):
            os.remove(os.path.join(tmpdir, f))

    # The starting time is very large, shift timestamps to around zero to reduce floating point
    # accuracy issues.
    t0 = None

    # Read groudtruth
    with open(tmpdir + '/mav0/state_groundtruth_estimate0/data.csv') as csvfile:
        # 0  timestamp,
        # 1  p_RS_R_x [m],
        # 2  p_RS_R_y [m],
        # 3  p_RS_R_z [m],
        # 4  q_RS_w [],
        # 5  q_RS_x [],
        # 6  q_RS_y [],
        # 7  q_RS_z [],
        # 8  v_RS_R_x [m s^-1],
        # 9  v_RS_R_y [m s^-1],
        # 10 v_RS_R_z [m s^-1],
        # 11 b_w_RS_S_x [rad s^-1],
        # 12 b_w_RS_S_y [rad s^-1],
        # 13 b_w_RS_S_z [rad s^-1],
        # 14 b_a_RS_S_x [m s^-2],
        # 15 b_a_RS_S_y [m s^-2],
        # 16 b_a_RS_S_z [m s^-2]
        csvreader = csv.reader(csvfile, delimiter=',')
        next(csvreader) # Skip header
        for row in csvreader:
            t = int(row[0]) / TO_SECONDS
            if not t0:
                t0 = t
            timestamp = t - t0
            output.append({
                "groundTruth": {
                    "position": {
                        "x": float(row[1]), "y": float(row[2]), "z": float(row[3])
                    },
                    "orientation": {
                        "w": float(row[4]), "x": float(row[5]), "y": float(row[6]), "z": float(row[7])
                    }
                },
                "time": timestamp
            })
            # f.write(",".join([str(timestamp), str(-float(row[1])), row[3], row[2], row[4], row[5], row[6], row[7]]) + "\n")

    # Read IMU
    with open(tmpdir + '/mav0/imu0/data.csv') as csvfile:
        # 0 timestamp [ns],
        # 1 w_RS_S_x [rad s^-1]
        # 2 w_RS_S_y [rad s^-1]
        # 3 w_RS_S_z [rad s^-1]
        # 4 a_RS_S_x [m s^-2]
        # 5 a_RS_S_y [m s^-2]
        # 6 a_RS_S_z [m s^-2]
        csvreader = csv.reader(csvfile, delimiter=',')
        next(csvreader) # Skip header
        for row in csvreader:
            timestamp = int(row[0]) / TO_SECONDS - t0
            output.append({
                "sensor": {
                    "type": "gyroscope",
                    "values": [float(row[1]), float(row[2]), float(row[3])]
                },
                "time": timestamp
            })
            output.append({
                "sensor": {
                    "type": "accelerometer",
                    "values": [float(row[4]), float(row[5]), float(row[6])]
                },
                "time": timestamp
            })

    makeDataset(output, tmpdir, name, t0, "stereo")
    # This dataset has lots of missing frames for cam0, so make monocular dataset from cam1.
    if name == "euroc-v2-03-difficult":
        makeDataset(output, tmpdir, name, t0, "cam1")

    print(name + " done!")


def main(args):
    for dataset in DATASETS:
        convert(dataset)
    print("Original non-converted files were left to {} folder, please delete them manually if you don't need them.".format(TEMP_DIR))
    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    main(args)
