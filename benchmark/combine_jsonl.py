import argparse
import json
import os
import shutil
import sys
import numpy as np

# Suggested workflow for using this:
# 1. When recording a session with multiple devices, have visual cue such as bringing thumb and forefinger together in beginning
# 2. Point add_time_offset.py to folder containing your test cases and follow it's instructions:
#    python vio_benchmark/benchmark/add_time_offset.py "data/benchmark/2020-07-02"
# 3. Pick the data source you want to augment with the tracking information from other sources. For example here
#    we add arkit and arengine tracking information to ttalo-03-arcore case and run this script:
#       python vio_benchmark/benchmark/combine_jsonl.py "data/benchmark/2020-07-02/ttalo-03-arcore" \
#           -arkit "data/benchmark/2020-07-02/ttalo-03-arkit" \
#           -arengine "data/benchmark/2020-07-02/ttalo-03-arengine"
#

parser = argparse.ArgumentParser(description="Combine tracking information from multiple JSONL files.")
parser.add_argument("folder", help="Folder containing data.jsonl file to be augmented with extra tracking info")
parser.add_argument("-our", help="Directory containing data.jsonl with your method tracking info")
parser.add_argument("-arkit", help="Directory containing data.jsonl with ARKit tracking info")
parser.add_argument("-arcore", help="Directory containing data.jsonl with ARCore tracking info")
parser.add_argument("-arengine", help="Directory containing data.jsonl with AREngine tracking info")
parser.add_argument("-groundtruth", help="Directory containing data.jsonl with Ground Truth tracking info")
parser.add_argument("-realsense", help="Directory containing data.jsonl with Realsense tracking info")
parser.add_argument("-gps", help="Directory containing data.jsonl with GPS location info")
parser.add_argument("-output", help="Name of directory to put the combined data in. If empty, overwrites original")
parser.add_argument("-rtk", help="RTK GPS, aligned using gpsTime and cut to start and end with frames")
parser.add_argument("-gpstime", help="Directory containing data.jsonl with gpsTime info")
parser.add_argument("-notrim", help="Unless this flag is set, output is trimmed based on 'frames' events", action="store_true")
parser.add_argument("-root", help="Root directory that's prefixed in front of all given paths", default=".")


TIME_PADDING_SECONDS = 1.0 # Add some padding before trimming data


def noOffsetInfo(folder):
    print("Couldn't find timeOffset field from {}/info.json. You must add it manually by looking through video footaged to be merged and finding timestamp for same event in each. This is used to align timestamps from different sources.".format(folder))
    sys.exit()


def shiftTime(entry, t):
    if "time" in entry:
        entry["time"] = entry["time"] - t
        if "frames" in entry:
            for frame in entry["frames"]:
                frame["time"] = frame["time"] - t
    else:
        entry["time"] = 0.0
        if not "model" in entry:
            print("Expected time field in JSONL line:", entry)


# Get lines *not* matching filter.
def readMainJSONL(outputArray, folder, rev_filter):
    timeOffset = None
    t0 = None
    with open(folder + "/info.json") as f:
        data = json.load(f)
        if "timeOffset" not in data:
            noOffsetInfo(folder)
        timeOffset = data["timeOffset"]

    with open(folder + "/data.jsonl") as f:
        for line in f.readlines():
            try:
                entry = json.loads(line)
            except:
                print("Ignoring bad JSONL line:", line)
                continue
            # Set the main method time to start from zero.
            # Ignore exactly zero timestamps because in some of our datasets there are
            # in the beginning rows that have zero for all fields, but the real timestamps
            # do not start from zero.
            if not t0 and 'time' in entry and entry["time"] != 0.0:
                t0 = entry["time"]
            if t0 and not any(entry.get(x) for x in rev_filter):
                shiftTime(entry, t0)
                outputArray.append(entry)

    return timeOffset, t0


# Get lines matching filter.
def readJSONL(outputArray, folder, filter, alignmentOffset=0, t0=0):
    if not folder:
        return

    with open(folder + "/info.json") as f:
        data = json.load(f)
        timeOffset = data["timeOffset"]

    if timeOffset is None:
        noOffsetInfo(folder)

    with open(folder + "/data.jsonl") as f:
        offset = timeOffset - alignmentOffset + t0
        print("Time realignment for {} is {} seconds".format(folder, offset))

        for line in f.readlines():
            try:
                entry = json.loads(line)
            except:
                print("Ignoring bad JSONL line:", line)
                continue
            if filter == "realsense" and entry.get("output"):
                shiftTime(entry, offset)
                entry["realsense"] = entry.pop("output")
                outputArray.append(entry)
            elif entry.get(filter):
                shiftTime(entry, offset)
                outputArray.append(entry)


def readRTKGPS(sortedOutputArray, rtkgpsFolder):
    gpsTimeOffets = []
    for x in sortedOutputArray:
        gps = x.get("gpsTime")
        if gps:
            gpsTimeOffets.append(x["time"] - gps["utcSeconds"])
    if not gpsTimeOffets:
        raise Exception("Couldn't find GPS events to sync RTK GPS using GPS time")
    gpsTimeOffset = np.median(gpsTimeOffets) # Pick median to avoid outliers, is this smart?
    # firstFrame = next(x for x in sortedOutputArray if x.get("frames"))["time"] - TIME_PADDING_SECONDS
    # lastFrame = sortedOutputArray[next(i for i in reversed(range(len(sortedOutputArray))) if sortedOutputArray[i].get("frames"))]["time"] + TIME_PADDING_SECONDS
    with open(rtkgpsFolder + "/data.jsonl") as f:
        for line in f.readlines():
            rtkgps = json.loads(line)
            alignedTime = rtkgps["time"] + gpsTimeOffset
            # if alignedTime >= firstFrame and alignedTime <= lastFrame:
            sortedOutputArray.append({
                "rtkgps": {
                    "latitude": rtkgps["lat"],
                    "longitude": rtkgps["lon"],
                    "altitude": rtkgps["altitude"],
                    "accuracy": rtkgps["accuracy"],
                    "verticalAccuracy": rtkgps["verticalAccuracy"],
                    "gpsTime": rtkgps["time"]
                },
                "time": alignedTime
            })


def trimArray(sortedOutputArray):
    print("Trimming output")
    firstFrame = next(x for x in sortedOutputArray if x.get("frames"))["time"] - TIME_PADDING_SECONDS
    lastFrame = sortedOutputArray[next(i for i in reversed(range(len(sortedOutputArray))) if sortedOutputArray[i].get("frames"))]["time"] + TIME_PADDING_SECONDS
    def filt(obj):
        time = obj.get("time")
        if time:
            return time >= firstFrame and time <= lastFrame
        return True
    return [row for row in sortedOutputArray if filt(row)]


def combineJSONL(mainFolder, outputFolder, folders, methods, rtkgpsFolder=None, trim=True):
    outputArray = []
    offset, t0 = readMainJSONL(outputArray, mainFolder, methods)
    for folder, method in zip(folders, methods):
        readJSONL(outputArray, folder, method, offset, t0)

    outputArray.sort(key=lambda x: x["time"])

    if rtkgpsFolder:
        readRTKGPS(outputArray, rtkgpsFolder)
        outputArray.sort(key=lambda x: x["time"])

    if trim:
        outputArray = trimArray(outputArray)

    if outputFolder:
        if os.path.exists(outputFolder):
            shutil.rmtree(outputFolder)
        os.mkdir(outputFolder)
    else:
        outputFolder = mainFolder
        os.rename(outputFolder + "/data.jsonl", outputFolder + "/data_backup.jsonl")

    with open(outputFolder + "/data.jsonl", "w") as f:
        for line in outputArray:
            f.write(json.dumps(line) + "\n")

    return t0


if __name__ == "__main__":
    args = parser.parse_args()

    args.root = args.root + "/"

    methods = []
    folders = []
    if args.our:
        methods.append("output")
        folders.append(args.root + args.our)
    if args.arkit:
        methods.append("ARKit")
        folders.append(args.root + args.arkit)
    if args.arcore:
        methods.append("arcore")
        folders.append(args.root + args.arcore)
    if args.arengine:
        methods.append("arengine")
        folders.append(args.root + args.arengine)
    if args.groundtruth:
        methods.append("groundTruth")
        folders.append(args.root + args.groundtruth)
    if args.realsense:
        methods.append("realsense")
        folders.append(args.root + args.realsense)
    if args.gps:
        methods.append("gps")
        folders.append(args.root + args.gps)
    if args.gpstime:
        methods.append("gpsTime")
        folders.append(args.root + args.gpstime)

    combineJSONL(args.root + args.folder, args.root + args.output, folders, methods, args.root + args.rtk, not args.notrim)
    print("Done!")
