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
    jsonlFile = folder if folder.endswith(".jsonl") else folder + "/data.jsonl"

    title = os.path.basename(os.path.normpath(folder))

    location = {"lat": [], "lon": [], "t": []}
    velocity = {"north": [], "east": [], "down": [], "t": []}
    speed = {"speed": [], "speedAcc": [], "t": []}

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
            time = measurement["time"] - timeOffset

            location["lat"].append(measurement["lat"])
            location["lon"].append(measurement["lon"])
            location["t"].append(time)

            if measurement.get("velocity"):
                velocity["north"].append(measurement["velocity"]["north"])
                velocity["east"].append(measurement["velocity"]["east"])
                velocity["down"].append(measurement["velocity"]["down"])
                velocity["t"].append(time)

                speed["speed"].append(measurement["groundSpeed"])
                speed["speedAcc"].append(measurement["speedAccuracy"])
                speed["t"].append(time)

    fig, subplots = pyplot.subplots(7)

    pyplot.subplots_adjust(hspace=.5)

    addSubplot(subplots[0], location["t"], location["lat"], "lat")
    addSubplot(subplots[1], location["t"], location["lon"], "lon")

    addSubplot(subplots[2], velocity["t"], velocity["north"], "vel north (m/s)")
    addSubplot(subplots[3], velocity["t"], velocity["east"], "vel east (m/s)")
    addSubplot(subplots[4], velocity["t"], velocity["down"], "vel down (m/s)")

    addSubplot(subplots[5], speed["t"], speed["speed"], "speed (m/s")
    addSubplot(subplots[6], speed["t"], speed["speedAcc"], "speed acc (m/s)")

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
