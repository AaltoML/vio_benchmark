#!/usr/bin/python

import argparse
import os
import csv
import json
from pathlib import Path
import subprocess
import yaml
import re
import allantools
import numpy as np
import matplotlib.pyplot
import math

parser = argparse.ArgumentParser()
parser.add_argument("folder", help="Folder containing JSONL and video file")
parser.add_argument("-output", help="Output folder, if not current directory")
parser.add_argument("-nthframes", help="Every Nth frame, default 4", default=4)
parser.add_argument("-stationary", help="Process data as stationary calibration set, skips first&last 15s", action="store_true")
args = parser.parse_args()

SECONDS_TO_NS = 1000 * 1000 * 1000

# kalibr_bagcreater doesn't support timestamps smaller than 1 second, add an offset
TIME_OFFSET_NS = 1 * SECONDS_TO_NS


def log10(x):
    return math.log(x, 10)


def getRandomWalkSegment(tau, sigma):
    M = -0.5  # slope of random walk
    i = 1
    idx = 1
    mindiff = 999
    logTau = -999
    while (logTau < 0):
        logTau = log10(tau[i])
        slope = (log10(sigma[i]) - log10(sigma[i - 1])) / (logTau - log10(tau[i - 1]))
        diff = abs(slope - M)
        if (diff < mindiff):
            mindiff = diff
            idx = i
        i = i + 1
    x1 = log10(tau[idx])
    y1 = log10(sigma[idx])
    x2 = 0
    y2 = M * (x2 - x1) + y1
    return (pow(10, x1), pow(10, y1), pow(10, x2), pow(10, y2))


def getBiasInstabilityPoint(tau, sigma):
    i = 1
    while (i < tau.size):
        if (tau[i] > 1) and ((sigma[i] - sigma[i - 1]) > 0):  # only check for tau > 10^0
            break
        i = i + 1
    return (tau[i], sigma[i])


# Allan variance computed as described in https://github.com/GAVLab/allan_variance/blob/master/scripts/allan.py
def computeNoiseRandomWalk(imu, outputFolder):
    firstTimeStamp = imu[0][0] / SECONDS_TO_NS
    lastTimeStamp = imu[len(imu) - 1][0] / SECONDS_TO_NS
    sampleRate = len(imu) / (lastTimeStamp - firstTimeStamp)
    print("Computed sample rate: {}".format(sampleRate))
    isDeltaType = False
    numTau = 1000 # number of lags

    # Form Tau Array
    taus = [None]*numTau
    cnt = 0
    for i in np.linspace(-2.0, 5.0, num=numTau): # lags will span from 10^-2 to 10^5, log spaced
        taus[cnt] = pow(10, i)
        cnt = cnt + 1

    N = len(imu) # number of measurement samples
    data = np.zeros( (6, N) ) # preallocate vector of measurements
    if isDeltaType:
        scale = sampleRate
    else:
        scale = 1.0

    cnt = 0
    for imuSample in imu:
        data[0,cnt] = imuSample[4] * scale
        data[1,cnt] = imuSample[5] * scale
        data[2,cnt] = imuSample[6] * scale
        data[3,cnt] = imuSample[1] * scale
        data[4,cnt] = imuSample[2] * scale
        data[5,cnt] = imuSample[3] * scale
        cnt = cnt + 1

    # Allan Variance
    results = []
    figure, subplots = matplotlib.pyplot.subplots(2, 3, figsize=(10,10))
    subplots = np.ravel(subplots)
    for index in range(6):
        (taus_used, adev, adev_err, adev_n) = allantools.oadev(data[index], data_type='freq', rate=float(sampleRate), taus=np.array(taus))

        randomWalkSegment = getRandomWalkSegment(taus_used,adev)
        biasInstabilityPoint = getBiasInstabilityPoint(taus_used,adev)

        randomWalk = randomWalkSegment[3]
        biasInstability = biasInstabilityPoint[1]

        if (index == 0):
            name = 'accelerometer_x'
        elif (index == 1):
            name = 'accelerometer_y'
        elif (index == 2):
            name = 'accelerometer_z'
        elif (index == 3):
            name = 'gyroscope_x'
        elif (index == 4):
            name = 'gyroscope_y'
        elif (index == 5):
            name = 'gyroscope_z'

        with open(outputFolder + "/summary.txt", 'a') as f:
            summary = "{}, randomWalk: {}, biasInstability: {}".format(name, randomWalk, biasInstability)
            f.write(summary + "\n")
            print(summary)

        results.append([randomWalk, biasInstability])

        # Plot Result
        plt = subplots[index]
        plt.set_yscale('log')
        plt.set_xscale('log')

        plt.plot(taus_used,adev)
        plt.plot([randomWalkSegment[0], randomWalkSegment[2]],
                 [randomWalkSegment[1], randomWalkSegment[3]], 'k--')
        plt.plot(1, randomWalk, 'rx', markeredgewidth=2.5, markersize=14.0)
        plt.plot(biasInstabilityPoint[0], biasInstabilityPoint[1], 'ro')

        plt.grid(True, which="both")
        plt.title.set_text(name)
        plt.set_xlabel('Tau (s)')
        plt.set_ylabel('ADEV')

    figure.savefig(outputFolder + "/plots.png")

    with open(outputFolder + "/imu.yaml", 'wt') as f:
        acc_random_walk = (results[0][0] + results[1][0] + results[2][0]) / 3
        acc_bias = (results[0][1] + results[1][1] + results[2][1]) / 3
        gyro_random_walk = (results[3][0] + results[4][0] + results[5][0]) / 3
        gyro_bias = (results[3][1] + results[4][1] + results[5][1]) / 3
        f.write("accelerometer_noise_density: {}\n".format(acc_random_walk))
        f.write("accelerometer_random_walk: {}\n".format(acc_bias))
        f.write("gyroscope_noise_density: {}\n".format(gyro_random_walk))
        f.write("gyroscope_random_walk: {}\n".format(gyro_bias))
        f.write("rostopic: {}\n".format("/imu0"))
        f.write("update_rate: {}\n".format(sampleRate))


def getNanoseconds(seconds):
    return int(seconds * SECONDS_TO_NS + TIME_OFFSET_NS)


def getVideoFile(folder, name):
    for f in os.listdir(folder):
        if re.search("{}\\.[avi|mp4|mov]".format(name), f):
            return f


# Export given frame numbers from video into PNG files and rename them to timestamps
def exportFrames(videoFile, outputFolder, nthframes, timestamps):
    os.makedirs(outputFolder, exist_ok=True)
    cmd = "ffmpeg -i {} -vf select='not(mod(n\\,{}))' -vsync 0 {}/frame_%05d.png" \
        .format(videoFile, nthframes, outputFolder)
    subprocess.run(cmd, shell=True)
    files = [f for f in os.listdir(outputFolder)]
    anyExtra = False
    for f in sorted(files):
        index = int(f.split("_")[1].split(".")[0])
        index = (index - 1) * nthframes
        fpath = os.path.join(outputFolder, f)
        if index in timestamps:
            newFilename = str(timestamps[index]) + ".png"
            os.rename(fpath, os.path.join(outputFolder, newFilename))
            assert(not anyExtra) # extra frames at the end of the recording are OK
        else:
            anyExtra = True
            print('WARNING: extra frame removed %s' % fpath)
            os.remove(fpath)

# Read acc+gyro and frame timestamps, convert time to nanoseconds
def readJsonl(folder):
    gyro = []
    acc = []
    frames = {}

    with open(folder + "/data.jsonl") as f:
        for line in f.readlines():
            try:
                entry = json.loads(line)
            except:
                print("Ignoring bad JSONL line:", line)
                continue
            if entry.get("sensor"):
                values = entry["sensor"]["values"]
                arr = [entry["time"], values[0], values[1], values[2]]
                if entry["sensor"]["type"] == "gyroscope":
                    gyro.append(arr)
                if entry["sensor"]["type"] == "accelerometer":
                    acc.append(arr)
            elif entry.get("frames"):
                if "number" in entry:
                    number = entry["number"]
                else:
                    number = entry["frames"][0]["number"]
                frames[number] = getNanoseconds(entry["time"])

    # fix dropped frames
    mapping = {}
    for num in sorted(frames.keys()):
        if num not in mapping:
            mapping[num] = len(mapping)
    mapped = { mapping[num]: frames[num] for num in frames.keys() }
    frames = mapped

    accStartIndex = 0
    synced = []
    for gyroSample in gyro:
        closestAccSample = acc[0]
        i = accStartIndex
        while i < len(acc):
            accSample = acc[i]
            if abs(closestAccSample[0] - gyroSample[0]) > abs(accSample[0] - gyroSample[0]):
                closestAccSample = accSample
                accStartIndex = i # Always start finding match for next sample where we left off
            if closestAccSample[0] > gyroSample[0]:
                break
            i += 1

        try:
            synced.append([getNanoseconds(gyroSample[0]), gyroSample[1], gyroSample[2], gyroSample[3], closestAccSample[1], closestAccSample[2], closestAccSample[3]])
        except:
            print("Failed {}".format(gyroSample[0]))

    return synced, frames # synced gyro + nearest acc, frmae timestamps


def main(args):
    imuData, frameTimestamps = readJsonl(args.folder)
    outputFolder = args.output if args.output else "."
    os.makedirs(outputFolder, exist_ok=True)

    if args.stationary:
        # For stationary calibration remove first and last 15 seconds when device was probably disturbed
        firstTimestamp = imuData[0][0] + 15 * SECONDS_TO_NS
        lastTimestamp = imuData[len(imuData) - 1][0] - 15 * SECONDS_TO_NS
        clipped = []
        for imuSample in imuData:
            if imuSample[0] > firstTimestamp and imuSample[0] < lastTimestamp:
                clipped.append(imuSample)
        computeNoiseRandomWalk(clipped, outputFolder)
        return


    with open(outputFolder + "/imu0.csv", "w") as csvfile:
        csvfile.write("timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z\n")
        for imuSample in imuData:
            csvfile.write(",".join([str(x) for x in imuSample]) + "\n")

    video0 = getVideoFile(args.folder, "data")
    exportFrames(args.folder + "/" + video0, outputFolder + "/cam0", args.nthframes, frameTimestamps)
    video1 = getVideoFile(args.folder, "data2")
    if video1:
        exportFrames(args.folder + "/" + video1, outputFolder + "/cam1", args.nthframes, frameTimestamps)


if __name__ == "__main__":
    main(args)
