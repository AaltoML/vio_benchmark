#!/usr/bin/python

# Convert <http://www.zjucvg.net/eval-vislam/dataset/> to JSONL format.
#
# The data uses similar format as EuRoC; compare the conversion scripts.
#
# First download the dataset zips (A0.zip, A1.zip, …) from the site (Google Drive is faster) into a folder.
# Unzip while creating a folder for each dataset:
#
#     for i in $(ls); do
#     d=$(echo $i | cut -d'.' -f1)
#     unzip $i -d $d
#     done

# Assuming the downloaded data is at `data/benchmark/sensetime-raw/`, run
#
#     cd data/benchmark
#     ../../scripts/convert/sensetime_to_benchmark.py
#
# which creates `data/benchmark/sensetime/` with the processed data.

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import yaml

RAW = "sensetime-raw"
OUT = "sensetime"

DATASETS = [
    "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7",
    "B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7",
]

parser = argparse.ArgumentParser()
args = parser.parse_args()

def convertVideo(files, output):
    # Use `-crf 0` for lossless compression.
    fps="30"
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

def convert(dataset):
    print("Converting", dataset)
    rawPath = "{}/{}".format(RAW, dataset)
    outPath = "{}/{}".format(OUT, dataset)
    Path(outPath).mkdir(parents=True, exist_ok=True)

    convertVideo("{}/camera/images/*.png".format(rawPath), "{}/data.mp4".format(outPath))

    with open(rawPath + "/camera/sensor.yaml") as yamlFile:
        try:
            yamlFile.readline() # Skip the YAML version line
            data = yaml.load(yamlFile, Loader=yaml.FullLoader)
            intrinsics = data["intrinsic"]["camera"]
            cameraParameters = {
                "focalLengthX": intrinsics[0],
                "focalLengthY": intrinsics[1],
                "principalPointX": intrinsics[2],
                "principalPointY": intrinsics[3]
            }
            # TODO Compute into `imuToCamera`.
            translation = data["extrinsic"]["p"]
        except yaml.YAMLError as exc:
            print(exc)
            raise Exception("Failed to read camera parameters.")

    number = 0
    output = []

    imageFiles = os.listdir(rawPath + "/camera/images")
    with open(rawPath + '/camera/data.csv') as csvfile:
        #t[s:double],filename[string]
        csvreader = csv.reader(csvfile, delimiter=',')
        next(csvreader) # Skip header
        for row in csvreader:
            timestamp = float(row[0])
            name = row[1]
            assert(name in imageFiles) # check consistency
            output.append({
                "frames": [
                    {"cameraInd": 0, "cameraParameters": cameraParameters, "number": number, "time": timestamp}
                ],
                "number": number,
                "time": timestamp,
            })
            number += 1
    assert(number == len(imageFiles)) # check consistency

    with open(rawPath + '/imu/data.csv') as csvfile:
        #t[s:double],w.x[rad/s:double],w.y[rad/s:double],w.z[rad/s:double],
        #a.x[m/s^2:double],a.y[m/s^2:double],a.z[m/s^2:double]
        csvreader = csv.reader(csvfile, delimiter=',')
        next(csvreader) # Skip header
        for row in csvreader:
            timestamp = float(row[0])
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

    with open(rawPath + '/groundtruth/data.csv') as csvfile:
        #t[s:double],q.x[double],q.y[double],q.z[double],q.w[double],
        #p.x[m:double],p.y[m:double],p.z[m:double]
        csvreader = csv.reader(csvfile, delimiter=',')
        next(csvreader) # Skip header
        for row in csvreader:
            output.append({
                "groundTruth": {
                    "position": {
                        "x": float(row[5]), "y": float(row[6]), "z": float(row[7])
                    },
                    "orientation": {
                        "w": float(row[4]), "x": float(row[1]), "y": float(row[2]), "z": float(row[3])
                    }
                },
                "time": float(row[0])
            })

    # Write JSONL
    output = sorted(output, key=lambda row: row["time"]) # Sort by time
    with open(outPath + "/data.jsonl", "w") as f:
        for obj in output:
            f.write(json.dumps(obj, separators=(',', ':')))
            f.write("\n")

    # Write parameters
    with open(outPath + "/parameters.txt", "w") as f:
        f.write("focalLengthX {};focalLengthY {};principalPointX {};principalPointY {};\n".format(*intrinsics))
        # Missing IMU-to-camera translation.
        f.write("rot 1; imuToCameraMatrix 0,-1,0,-1,0,0,0,0,-1;\n")

def main(args):
    for dataset in DATASETS:
        convert(dataset)

if __name__ == "__main__":
    main(args)
