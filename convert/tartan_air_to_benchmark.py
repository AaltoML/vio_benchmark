#!/usr/bin/env python
#
# Download and convert TartanAir data <https://theairlab.org/tartanair-dataset/>.
#
# NOTE The whole dataset is several terabytes, so be sure to tune the `LEVELS` and
# `DATASETS` variables before running.
#
# It is recommended to install "AzCopy", an official tool for Azure, to get tolerable
# download speeds (pass `--azcopy` flag to enable).
#
# NOTE At the time of writing the data does not include simulated IMU samples.

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess

from tartan_air_transformations import fixTartan
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--azcopy', action='store_true', default=False, help='download the data with AzCopy')
args = parser.parse_args()

# Since the downloads can be slow, an option to leave the downloaded zip files in the RAW directory.
BACKUP_ZIPS = False

RAW = "data/raw/tartan-air"
OUT = "data/benchmark/tartan-air"

# <https://github.com/castacks/tartanair_tools/blob/master/download_training_zipfiles.txt>
RELEASE = "https://tartanair.blob.core.windows.net/tartanair-release1"

LEVELS = ["Easy", "Hard"]

DATASETS = [
    "abandonedfactory",
    "amusement",
    "carwelding",
    "endofworld",
    "gascola",
    "hospital",
    "japanesealley",
    "neighborhood",
    "ocean",
    "office",
    "office2",
    "oldtown",
    "seasidetown",
    "seasonsforest"
    "soulcity",
    "westerndesert",
]

DOWNLOAD_CMD = "wget -O"
UNZIP_CMD = "unzip -o -d"

# The data doesn't have time information of any sort,
# so pick something that makes the videos run at a pleasant speed.
FPS = 10

def runCmd(cmd):
    print("Running command:", cmd)
    os.system(cmd)

def convertVideo(files, output):
    # Use `-crf 0` for lossless compression.
    subprocess.run(["ffmpeg",
        "-y",
        "-r", str(FPS),
        "-f", "image2",
        "-pattern_type", "glob", "-i", files,
        "-c:v", "libx264",
        "-preset", "ultrafast",
        # "-preset", "veryslow",
        "-crf", "0",
        "-vf", "format=yuv420p",
        "-an",
        output])

def getExtractedPath(dataset, level):
    # For some reason `dataset` is duplicated in the zip hierarchy.
    return "{}/{}/{}/{}".format(RAW, dataset, dataset, level)

def download(dataset, level):
    extractedPath = getExtractedPath(dataset, level)
    if os.path.isdir(extractedPath):
        print(extractedPath, "already exists, skipping.")
        return

    outPath = RAW
    Path(outPath).mkdir(parents=True, exist_ok=True)

    for d in ["image_left", "image_right"]:
        url = "{}/{}/{}/{}.zip".format(RELEASE, dataset, level, d)
        z = "{}/{}.zip".format(outPath, d)
        if args.azcopy:
            cmd = "azcopy copy {} {}".format(url, z)
            runCmd(cmd)
        else:
            cmd = "{} {} {}".format(DOWNLOAD_CMD, z, url)
            runCmd(cmd)

        cmd = "{} {} {}".format(UNZIP_CMD, outPath, z)
        runCmd(cmd)

        src = "{}/{}.zip".format(outPath, d)
        if BACKUP_ZIPS:
            name = "{}-{}-{}".format(dataset, level, d)
            dst = "{}/{}.zip".format(outPath, name)
            os.rename(src, dst)
        else:
            os.remove(src)

def convert_sequence(fullPath, sequence, dataset, level):
    datasetOut = "{}/{}-{}".format(dataset, level.lower(), sequence)
    outPath = "{}/{}".format(OUT, datasetOut)
    Path(outPath).mkdir(parents=True, exist_ok=True)

    convertVideo("{}/image_left/*.png".format(fullPath), "{}/data.mp4".format(outPath))
    convertVideo("{}/image_right/*.png".format(fullPath), "{}/data2.mp4".format(outPath))

    output = []
    number = 0
    time = 0.0
    dt = 1.0 / FPS
    p0 = [None, None, None]
    # We define ground truth as pose of the left camera.
    with open("{}/pose_left.txt".format(fullPath)) as f:
        # format: tx ty tz qx qy qz qw
        csvRows = csv.reader(f, delimiter=' ')
        rows = []
        for row in csvRows:
            rows.append(row)

        # The general coordinate transformation has the form
        #     M -> W*M*L,  where M = M(p, q)
        # The W and L matrices were found by experimentation starting with transforms
        # in `ned2cam()` function in the TartanAir repository's scripts.
        W = np.array([
            [0,1,0,0],
            [1,0,0,0],
            [0,0,-1,0],
            [0,0,0,1]], dtype=np.float32)
        L = np.array([
            [0,0,1,0],
            [1,0,0,0],
            [0,1,0,0],
            [0,0,0,1]], dtype=np.float32)
        fixedRows = fixTartan(W, L, rows)

        for row in fixedRows:
            if not p0[0]:
                p0 = [row[0], row[1], row[2]]
            p = [row[0] - p0[0], row[1] - p0[1], row[2] - p0[2]]
            q = [row[6], row[3], row[4], row[5]] # wxyz
            gt = {
                "groundTruth": {
                    "position": {
                        "x": p[0], "y": p[1], "z": p[2]
                    },
                    "orientation": {
                        "w": q[0], "x": q[1], "y": q[2], "z": q[3]
                    }
                },
                "time": time
            }
            frame = {
                "number": number,
                "time": time,
                "frames": [
                    {"cameraInd": 0, "time": time},
                    {"cameraInd": 1, "time": time},
                ],
            }
            output.append(gt)
            output.append(frame)
            time += dt
            number += 1

    # Write JSONL
    with open(outPath + "/data-no-imu.jsonl", "w") as f:
        for obj in output:
            f.write(json.dumps(obj, separators=(',', ':')))
            f.write("\n")

    # Write parameters
    with open(outPath + "/parameters.txt", "w") as f:
        # <https://github.com/castacks/tartanair_tools/blob/master/data_type.md>
        fx = 320
        fy = 320
        cx = 320
        cy = 240
        f.write("focalLengthX {}; focalLengthY {};\nprincipalPointX {}; principalPointY {};\n".format(fx, fy, cx, cy))
        f.write("secondFocalLengthX {}; secondFocalLengthY {};\nsecondPrincipalPointX {}; secondPrincipalPointY {};\n".format(fx, fy, cx, cy))
        f.write("rot 0;\n")
        # Define the (non-existent) IMU to have the same pose as the left camera.
        for cam in [0, 1]:
            columnMajor = []
            for i in [0, 1, 2, 3]:
                for j in [0, 1, 2, 3]:
                    if cam == 1 and i == 3 and j == 0:
                        num = "-0.25" # baseline
                    elif i == j:
                        num = "1"
                    else:
                        num = "0"
                    columnMajor.append(num)
            f.write("{} {};\n".format(
                "imuToCameraMatrix" if cam == 0 else "secondImuToCameraMatrix",
                ",".join(columnMajor)))

def convert(dataset, level):
    extractedPath = getExtractedPath(dataset, level)
    folders = [ (f.path, f.name) for f in os.scandir(extractedPath) if f.is_dir() ]
    folders.sort()
    for fullPath, sequence in folders:
        convert_sequence(fullPath, sequence, dataset, level)

def main():
    for dataset in DATASETS:
        for l in LEVELS:
            download(dataset, l)
            convert(dataset, l)

if __name__ == "__main__":
    main()
