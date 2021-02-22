# Example usage:
#  Take every 5th frame between frames 250 and 500
#
# python ~/convert/export_to_colmap.py \
#   ./raw_data/ \
#   ./output/vio-output.csv \
#   ./output \
#   -start 250 \
#   -stop 500
#   -frames 5 \

import argparse
import json
import os
import shutil
import subprocess
import numpy as np
# quaternion aka numpy-quaternion doesn't work, avoid it
import re
import platform
from pathlib import Path
import math
import sqlite3

SYSTEM = platform.system()
if SYSTEM == "Darwin":
    COLMAP="/Applications/COLMAP.app/Contents/MacOS/colmap"

parser = argparse.ArgumentParser(description="Export tool to Colmap")
parser.add_argument("folder", help="Input folder containing video and jsonl data")
parser.add_argument("poses", help="CSV file containg poses produced with a vio algorithm")
parser.add_argument("output", help="Output folder")
parser.add_argument("-frames", help="Every Nth frame to export", type=int, default=5)
parser.add_argument("-start", help="First frame to export", type=int, default=20) # Skipping init phase is probably smart
parser.add_argument("-stop", help="Last frame to export", type=int, default=100)
parser.add_argument("-onlyColmap", help="Skips convertion steps", action="store_true")
parser.add_argument("-skipColmap", help="Skips colmap steps", action="store_true")
# parser.add_argument("-permutations", help="Do all permutations", action="store_true")
parser.add_argument("-cam_f", help="Focal length", type=float, default=980.0)
parser.add_argument("-cam_px", help="px", type=float, default=640.0)
parser.add_argument("-cam_py", help="py", type=float, default=480.0)

# Linear interpolation between two rotation quaternions
def slerp(q0, q1, t):
    q0 = np.array(q0)
    q1 = np.array(q1)
    dot = np.sum(q0 * q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        result = q1 + t * (q1 - q0)
        return result / np.sqrt(np.sum(result**2))
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q0) + (s1 * q1)


# Quaternion to rotation matrix
def quatToMatrix(q):
    return np.array([
        [1 - 2*q[2]*q[2] - 2*q[3]*q[3],	2*q[1]*q[2] - 2*q[3]*q[0],	2*q[1]*q[3] + 2*q[2]*q[0]],
        [2*q[1]*q[2] + 2*q[3]*q[0],	1 - 2*q[1]*q[1] - 2*q[3]*q[3],	2*q[2]*q[3] - 2*q[1]*q[0]],
        [2*q[1]*q[3] - 2*q[2]*q[0],	2*q[2]*q[3] + 2*q[1]*q[0],	1 - 2*q[1]*q[1] - 2*q[2]*q[2]]
    ])


# Rotation matrix to quaternion
def matrixToQuat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        S = math.sqrt(tr+1.0) * 2 # S=4*qw
        qw = 0.25 * S
        qx = (m[2][1] - m[1][2]) / S
        qy = (m[0][2] - m[2][0]) / S
        qz = (m[1][0] - m[0][1]) / S
    elif (m[0][0] > m[1][1]) and (m[0][0] > m[2][2]):
        S = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2 # S=4*qx
        qw = (m[2][1] - m[1][2]) / S
        qx = 0.25 * S
        qy = (m[0][1] + m[1][0]) / S
        qz = (m[0][2] + m[2][0]) / S
    elif m[1][1] > m[2][2]:
        S = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2 # S=4*qy
        qw = (m[0][2] - m[2][0]) / S
        qx = (m[0][1] + m[1][0]) / S
        qy = 0.25 * S
        qz = (m[1][2] + m[2][1]) / S
    else:
        S = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2 # S=4*qz
        qw = (m[1][0] - m[0][1]) / S
        qx = (m[0][2] + m[2][0]) / S
        qy = (m[1][2] + m[2][1]) / S
        qz = 0.25 * S
    return np.array([qw, qx, qy, qz])


# def axisPermutations():
#     swaps = [
#       [[1,0,0],[0,1,0],[0,0,1]],
#       [[0,1,0],[1,0,0],[0,0,1]],
#       [[0,0,1],[0,1,0],[1,0,0]],
#       [[1,0,0],[0,0,1],[0,1,0]],
#       [[0,1,0],[0,0,1],[1,0,0]],
#       [[0,0,1],[1,0,0],[0,1,0]],
#     ]
#     flips = [
#         [0,0,0],
#         [0,0,1],
#         [0,1,0],
#         [0,1,1],
#         [1,0,0],
#         [1,1,0],
#         [1,0,1],
#         [1,1,1],
#     ]
#     permutations = []
#     for s in swaps:
#         for f in flips:
#             p = np.array(s) * (np.array(f) * 2 - 1)
#             permutations.append(p)
#     return permutations


# Intepolates pose for each frame
def interpolate(frames, poses):
    index = 0
    result = []
    for f in frames:
        t = f[0]
        while index < len(poses) and t > poses[index][0]:
            index += 1
        if index >= len(poses):
            pose0 = poses[index - 1]
            pose1 = poses[index - 1]
        else:
            pose0 = poses[max(0, index - 1)]
            pose1 = poses[index]
        x = (t - pose0[0]) / (pose0[1] - pose0[0])
        #T, ID, QW, QX, QY, QZ, TX, TY, TZ
        result.append(
            f
            + slerp(pose0[1:5], pose1[1:5], x).tolist()
            + (x * pose0[5:8] + (1 - x) * pose0[5:8]).tolist()
        )

    return result


def chunks(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


# Export given frame numbers from video into PNG files
def exportFrames(args, videoFile, outputFolder):
    os.makedirs(outputFolder, exist_ok=True)
    start = args.start
    stop = args.stop - 1
    cmd = "ffmpeg -i {} -vf select='not(mod(n\\,{}))*between(n\\,{}\\,{})' -vsync 0 {}/frame_%05d.png" \
        .format(videoFile, args.frames, start, stop, outputFolder)
    subprocess.run(cmd, shell=True)

    cmd = "ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 {}" \
        .format(videoFile)
    resolution = subprocess.run(cmd, capture_output=True, shell=True, encoding="utf-8").stdout.strip().split("x")

    return resolution


# Finds data.* video
def findVideo(args):
    p = re.compile(r'data\.(mp4|avi|mov)')
    for f in os.listdir(args.folder):
        if p.match(f):
            return os.path.join(args.folder, f)
    raise Exception("Couldn't find supported 'data.*' video file in {}".format(args.folder))


def getImuToCameraMatrix(args):
    parameters = args.folder + "/parameters.txt"
    with open(parameters) as f:
        for param in f.read().replace('\n', '').split(";"):
            pair = param.split(" ")
            if pair[0].strip() == "imuToCameraMatrix":
                v = [int(x) for x in pair[1].strip().split(",")]
                return np.array([
                    [v[0],v[1],v[2]],
                    [v[3],v[4],v[5]],
                    [v[6],v[7],v[8]]
                ])
    raise Exception("Couldn't find 'imuToCameraMatrix' from {}".format(parameters))


def vioToColmap(pose, imuToCameraMatrix):
    # https://colmap.github.io/format.html#output-format
    # The quaternion is defined using the Hamilton convention, which is, for example,
    # also used by the Eigen library. The coordinates of the projection/camera center
    # are given by -R^t * T, where R^t is the inverse/transpose of the 3x3 rotation
    # matrix composed from the quaternion and T is the translation vector. The local
    # camera coordinate system of an image is defined in a way that the X axis points
    # to the right, the Y axis to the bottom, and the Z axis to the front as seen from
    # the image.

    # T, QW, QX, QY, QZ, TX, TY, TZ
    q = np.array(pose[1:5])
    if np.linalg.norm(np.array(pose[1:5])) < .9:
        q = np.array([1, 0, 0, 0])
    t = np.array(pose[5:8])

    worldToImu = quatToMatrix(q)
    worldToCamera = np.matmul(imuToCameraMatrix, worldToImu)

    # The pose of each image is defined with respect to a world coordinate system and
    # the position of the image in the world coordinate system is given as -R^t*T.
    qT = -np.matmul(worldToCamera, t)

    result = np.array(
        pose[0:1].tolist()
        + matrixToQuat(worldToCamera).tolist()
        + qT.tolist()
    )

    return result


# TODO: Convert this to JSONL format
# Extracts pose information from CSV file
def extractPoses(args):
    raw = np.genfromtxt(args.poses, delimiter=',')
    poses = np.hstack([raw[:,i:(i+1)] for i in [0,8,9,10,11,2,3,4]])
    transformed = []
    imuToCameraMatrix = getImuToCameraMatrix(args)
    for p in poses:
        transformed.append(vioToColmap(p, imuToCameraMatrix))
    return transformed


def writeImagesTxt(args, frameposes):
    index = 1
    result = []
    with open(args.output + "/images.txt", "w") as f:
        for pose in frameposes:
            # IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
            arr = [str(index)] \
                + [" ".join([str(x) for x in pose[2:9]])] \
                + [str(index)] \
                + ["frame_{:05d}.png".format(index)]
            f.write(" ".join(arr) + "\n\n")
            index += 1
            result.append(arr)
    return result


def writeCamerasTxt(args, resolution, imageArr):
    with open(args.output + "/cameras.txt", "w") as f:
        # ID model width height focal px py
        for x in imageArr:
            f.write("{} SIMPLE_PINHOLE {} {} {} {} {}\n" \
                .format(x[0], resolution[0], resolution[1], args.cam_f, args.cam_px, args.cam_py))


def runColmap(args, imageArr):
    database = args.output + "/database.db"
    images = args.output + "/images"
    triangulated = args.output + "/sparse"

    Path(triangulated).mkdir(parents=True, exist_ok=True)
    cmd = "{} feature_extractor --database_path {} --image_path {}".format(COLMAP, database, images) \
        + " --ImageReader.camera_model SIMPLE_PINHOLE --ImageReader.camera_params \"{},{},{}\"".format(
            args.cam_f, args.cam_px, args.cam_py) \
        + " --SiftExtraction.estimate_affine_shape=true --SiftExtraction.domain_size_pooling=true"
    subprocess.run(cmd, shell=True)

    # Colmap can't ingest camera.txt properly, edit DB directly
    conn = sqlite3.connect(database)
    c = conn.cursor()
    c.execute("UPDATE cameras SET model = 0, params = ?", (np.array([args.cam_f, args.cam_px, args.cam_py]).tostring(),))
    conn.commit()

    # Colmap reorders images, export order from DB
    # CREATE TABLE images   (
    # 0 image_id   INTEGER  PRIMARY KEY AUTOINCREMENT  NOT NULL,
    # 1 name       TEXT                                NOT NULL UNIQUE,
    # 2 camera_id  INTEGER                             NOT NULL,
    # ...
    os.rename(args.output + "/images.txt", args.output + "/images.txt.backup")
    c = conn.cursor()
    with open(args.output + "/images.txt", "w") as f:
        for row in c.execute("SELECT * FROM images"):
            # Cam ID + image file name stay same, image id changes
            img = imageArr[int(row[2]) - 1]
            img[0] = str(row[0])
            f.write(" ".join(img) + "\n\n")
    conn.close()

    cmd = "{} exhaustive_matcher --database_path {} --SiftMatching.guided_matching=true".format(COLMAP, database)
    subprocess.run(cmd, shell=True)

    # Using very high max reproj error (4 default) gives us points even with inaccurate poses
    cmd = "{} point_triangulator --database_path {} --image_path {} --input_path {} --output_path {} --Mapper.filter_max_reproj_error 20" \
        .format(COLMAP, database, images, args.output, triangulated)
    subprocess.run(cmd, shell=True)


# Converts data into colmap ingestable format
def convertData(args):
    Path(args.output).mkdir(parents=True, exist_ok=True)

    frames = []
    with open(args.folder + "/data.jsonl") as f:
        for line in f.readlines():
            entry = json.loads(line)
            if entry.get("frames") \
                and entry["number"] % args.frames == 0 \
                and entry["number"] < args.stop \
                and entry["number"] >= args.start:
                #QW, QX, QY, QZ, TX, TY, TZ
                frames.append([entry["time"], entry["number"]])

    poses = extractPoses(args)
    frameposes = interpolate(frames, poses)
    imageArr = writeImagesTxt(args, frameposes)
    with open(args.output + '/points3D.txt', 'w') as f:
        pass

    resolution = exportFrames(args, findVideo(args), args.output + "/images")
    writeCamerasTxt(args, resolution, imageArr)

    if args.skipColmap:
        return
    if not COLMAP:
        raise Exception("TODO: Add colmap path on this OS")

    runColmap(args, imageArr)


def main(args):
    # if not args.permutations:
        # TODO: Transformation matrix is wrong for vio
    convertData(args)
    # else:
    #     permutations = axisPermutations()
    #     output = args.output
    #     for i, p in enumerate(permutations):
    #         args.output = "{}_{:02d}".format(output, i)
    #         Path(args.output).mkdir(parents=True, exist_ok=True)
    #         with open(args.output + "/matrix.txt", "w") as f:
    #             f.write(np.array_str(p))
    #         convertData(args, p)
    print("Done!")


if __name__ == "__main__":
    main(parser.parse_args())
