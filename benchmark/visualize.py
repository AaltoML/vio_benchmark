import argparse
from matplotlib import pyplot
import json
import os
import sys

parser = argparse.ArgumentParser(description="JSONL visualizer")
parser.add_argument("case", help="Folder containing data.jsonl file", nargs='?', default=None)
parser.add_argument("-dir", help="Directory containing benchmarks you want to plot")
parser.add_argument("-zero", help="Rescale time to start from zero", action='store_true')


def addSubplot(plot, x, y, title, style=None):
    plot.title.set_text(title)
    if style is not None:
        plot.plot(x, y, style)
    else:
        plot.plot(x, y)


def plotDataset(folder, args):
    jsonlFile = folder + "/data.jsonl"

    title = os.path.basename(os.path.normpath(folder))

    accelerometer = {"x": [], "y": [], "z": [], "t": []}
    gyroscope = {"x": [], "y": [], "z": [], "t": []}
    cameras = {}

    timeOffset = 0
    if args.zero:
        timeOffset = sys.maxsize
        with open(jsonlFile) as f:
            for line in f.readlines():
                measurement = json.loads(line)
                if measurement.get("time") is not None:
                    if timeOffset > measurement["time"]:
                        timeOffset = measurement["time"]

    with open(jsonlFile) as f:
        for line in f.readlines():
            measurement = json.loads(line)
            if measurement.get("sensor") is not None:
                measurementType = measurement["sensor"]["type"]
                if measurementType == "accelerometer":
                    accelerometer["x"].append(measurement["sensor"]["values"][0])
                    accelerometer["y"].append(measurement["sensor"]["values"][1])
                    accelerometer["z"].append(measurement["sensor"]["values"][2])
                    accelerometer["t"].append(measurement["time"] - timeOffset)
                if measurementType == "gyroscope":
                    gyroscope["x"].append(measurement["sensor"]["values"][0])
                    gyroscope["y"].append(measurement["sensor"]["values"][1])
                    gyroscope["z"].append(measurement["sensor"]["values"][2])
                    gyroscope["t"].append(measurement["time"] - timeOffset)
            if measurement.get("frames") is not None:
                frames = measurement.get("frames")
                for f in frames:
                    ind = f["cameraInd"]
                    if cameras.get(ind) is None:
                        cameras[ind] = {"diff": [], "t": []}
                        cameras[ind]["diff"].append(0.)
                        cameras[ind]["t"].append(measurement["time"] - timeOffset)
                    else:
                        diff = measurement["time"] - cameras[ind]["t"][-1]
                        # print("Time {}, Diff {}".format(measurement["time"], diff))
                        cameras[ind]["diff"].append(diff * 1000.)
                        cameras[ind]["t"].append(measurement["time"] - timeOffset)

    fig, subplots = pyplot.subplots(6 + len(cameras.keys()))

    pyplot.subplots_adjust(hspace=.5)

    addSubplot(subplots[0], accelerometer["t"], accelerometer["x"], "acc x (m/s)")
    addSubplot(subplots[1], accelerometer["t"], accelerometer["y"], "acc y (m/s)")
    addSubplot(subplots[2], accelerometer["t"], accelerometer["z"], "acc z (m/s)")

    addSubplot(subplots[3], gyroscope["t"], gyroscope["x"], "gyro x (m/s)")
    addSubplot(subplots[4], gyroscope["t"], gyroscope["y"], "gyro y (m/s)")
    addSubplot(subplots[5], gyroscope["t"], gyroscope["z"], "gyro z (m/s)")

    i = 0
    for ind in cameras.keys():
        camera = cameras[ind]
        addSubplot(subplots[6 + i], camera["t"], camera["diff"], "frame time #{} (ms)".format(ind), ".")
        i += 1

    fig.suptitle(title, fontsize=16)

    pyplot.show()


def main(args):
    if args.case:
        plotDataset(args.case, args)
    elif args.dir:
        groups = {}
        for x in os.walk(args.dir):
            for file in x[2]:
                if file == "data.jsonl":
                    plotDataset(x[0], args)
    else:
        print("Invalid arguments")
        exit(1)


if __name__ == "__main__":
    main(parser.parse_args())
