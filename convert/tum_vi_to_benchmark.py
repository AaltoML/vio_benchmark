#!/usr/bin/python
#
# Convert data from <https://vision.in.tum.de/data/datasets/visual-inertial-dataset>.
# Paper <https://arxiv.org/abs/1804.06120> “The TUM VI Benchmark for Evaluating Visual-Inertial Odometry”
#
# The data uses similar format as EuRoC; compare the conversion scripts.
#
# Usage:
# * Download some datasets under the "Euroc/DSO 512x512" link from the above site.
# * Extract the tarballs so that folders have paths like `data/raw/tum-vi/dataset-room1_512_16/`.
# * List the short names of the downloaded datasets in `DATASETS` variable. (Comment out the ones you don't have).
# * Run `python convert/tum_vi_to_benchmark.py`.

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import yaml

RAW = "data/raw/tum-vi"
OUT = "data/benchmark/tum-vi"

# "room" (1-6) sets are the only ones with full ground truth.
# "slides" (1-3) may be interesting because of little visual information inside the slides.
# "corridor4" is reasonably short.
DATASETS = [
    "room1",
    "room2",
    "room3",
    "room4",
    "room5",
    "room6",
    "slides1",
    "slides2",
    "slides3",
    "corridor4",
]

DATA_RESOLUTION = 512 # 512 or 1024 depending which raw data was downloaded

parser = argparse.ArgumentParser()
args = parser.parse_args()

TO_SECONDS = 1000 * 1000 * 1000 # Timestamps are in nanoseconds

def convertVideo(files, output):
    # Use `-crf 0` for lossless compression.
    fps="20"
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

def convert(datasetOut):
    print("---\nConverting {}\n---".format(datasetOut))
    dataset = "dataset-{}_{}_16".format(datasetOut, DATA_RESOLUTION)

    rawPath = "{}/{}".format(RAW, dataset)
    outPath = "{}/{}".format(OUT, datasetOut)
    Path(outPath).mkdir(parents=True, exist_ok=True)

    convertVideo("{}/mav0/cam0/data/*.png".format(rawPath), "{}/data.mp4".format(outPath))
    convertVideo("{}/mav0/cam1/data/*.png".format(rawPath), "{}/data2.mp4".format(outPath))

    parameters = []
    intrinsics = []
    distortionCoeffs = []
    imuToCameras = []
    with open(rawPath + "/dso/camchain.yaml") as yamlFile:
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
                # The paper explains the convention is T_A_B means transform from B to A,
                # so this is IMU-to-camera transform.
                imuToCameras.append(data[cam]["T_cam_imu"])
        except yaml.YAMLError as exc:
            print(exc)
            raise Exception("Failed to read camera parameters.")

    # The two stereo folder image files seem to be perfectly matched, unlike in the EuRoC data.
    timestamps = []
    dir0 = "{}/mav0/cam0/data".format(rawPath)
    for filename in os.listdir(dir0):
        timestamps.append(int(os.path.splitext(filename)[0]) / TO_SECONDS)

    # The starting time is very large, shift timestamps to around zero to reduce floating point
    # accuracy issues.
    timestamps = sorted(timestamps)
    t0 = timestamps[0]

    output = []
    number = 0
    for timestamp in timestamps:
        t = timestamp - t0
        x = {
            "number": number,
            "time": t,
            "frames": [
                {"cameraInd": 0, "cameraParameters": parameters[0], "time": t},
                {"cameraInd": 1, "cameraParameters": parameters[1], "time": t},
            ],
        }
        output.append(x)
        number += 1

    with open(rawPath + '/mav0/imu0/data.csv') as csvfile:
        # timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
        # a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
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

    with open(rawPath + '/mav0/mocap0/data.csv') as csvfile:
        # timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
        # q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []
        csvreader = csv.reader(csvfile, delimiter=',')
        next(csvreader) # Skip header
        for row in csvreader:
            timestamp = int(row[0]) / TO_SECONDS - t0
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

def main(args):
    for dataset in DATASETS:
        convert(dataset)

if __name__ == "__main__":
    main(args)
