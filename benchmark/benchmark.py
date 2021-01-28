# Vio tracker benchmark library

import argparse
import sys
from datetime import datetime
import time
import subprocess
import os
from pathlib import Path
import json
from collections import deque
from .compute_metrics import compute_metrics
from .draw_slam_maps import draw_slam_maps
from .utils import GpsToLocalConverter
import concurrent.futures
from functools import partial
import multiprocessing
import traceback
import math
import socket
import re


DATE_FORMAT = "%Y-%m-%d_%H-%M-%S"


def getArgParser():
    parser = argparse.ArgumentParser(description="Benchmark runner script")
    parser.add_argument("tags", help="Benchmark case", nargs='?', default=None)
    parser.add_argument("-output", help="Output directory for benchmark", default="output")
    parser.add_argument("-dir", help="Directory of benchmark data folders", default="data/benchmark")
    parser.add_argument("-set", help="Run a benchmark set")
    parser.add_argument("-setDir", help="An additional directory to search for benchmark set")
    parser.add_argument("-params", help="Parameters as a string ie. '-params \"-displayVideo=true -v=1\"' for example")
    parser.add_argument("-threads", help="How many CPU threads to use for running benchmarks", default=6)
    parser.add_argument("-runId", help="Run ID for the benchmark")
    parser.add_argument("-parallelBenchmark", help="", action="store_true")
    parser.add_argument("-rotateCases", help="", action="store_true")
    parser.add_argument("-groundTruth", help="Ground truth csv file", default="ground-truth.csv")
    parser.add_argument("-skipAggregate", help="Skips aggregated benchmark. Good for running serveral benchmarks in parallel", action="store_true")
    parser.add_argument("-skipBenchmark", help="Skips running benchmark and only aggregates existing ones", action="store_true")
    parser.add_argument("-distributedBatchSize", help="Defines total number of batches i.e. how many worker nodes there are")
    parser.add_argument("-distributedBatchNumber", help="Defines number of this batch, only benchmarks belonging to this batch number are ran with formula: X %% distributedBatchSize == distributedBatchNumber")
    parser.add_argument("-testDeterministic", help="Assumes test set has N identical tests and runs diff on output log", action="store_true")
    parser.add_argument("-compare", help="Comma separated list of comparison datasets: arengine,arkit,arcore,groundtruth,ondevice")
    parser.add_argument("-piecewise", help="When enabled visualize piecewise track", action="store_true")
    parser.add_argument("-offsetTracks", help="When enabled, tracks are stacked instead of overlaid", action="store_true")
    parser.add_argument('-metricSet', type=str, default='default', choices={'default', 'xy_only', 'full_2d_align', 'full_3d_align', 'full_sim3'})
    parser.add_argument("-displayHostname", help="When enabled, each benchmark metrics show machine hostname", action="store_true")
    return parser


class Benchmark:
    dir = None
    params = None
    name = None
    benchmarkComparison = None
    paramSet = None
    origName = None

    def __init__(self, dir, name=None, params=None, benchmarkComparison=None, paramSet=None, origName=None):
        self.dir = dir
        self.name = name if name else os.path.basename(dir)
        self.origName = origName if origName else os.path.basename(dir)
        self.params = params
        self.benchmarkComparison = benchmarkComparison
        self.paramSet = paramSet if paramSet else "DEFAULT"


def metricsKwargs(args, single=False):
    """Extra parameters given to compute_metrics.py"""
    if single:
        kwargs = {}
    else:
        kwargs = dict(piecewise=args.piecewise, offsetTracks=args.offsetTracks, comparisonList=args.compare)

    kwargs['metric_set'] = args.metricSet
    return kwargs


def readLastLine(filename):
    with open(filename, 'rb') as f:
        f.seek(-2, os.SEEK_END)
        while f.read(1) != b'\n':
            f.seek(-2, os.SEEK_CUR)
        return f.readline().decode()


def filterByTags(info, tags): # filter_by_tags.py
    requiredTags = tags.split(",")
    infoDict = json.loads(open(info).read())
    dirPath = os.path.dirname(info)
    dirName = os.path.basename(dirPath)
    foundTags = infoDict.get("tags", [])
    foundTags.append(dirName)
    for reqTag in requiredTags:
        if not reqTag in foundTags:
            return
    return dirPath


def runAndCapture(cmd):
    return subprocess.run(cmd, stdout=subprocess.PIPE, shell=True).stdout.decode('utf-8').strip()


def withMkdir(dir):
    Path(dir).mkdir(parents=True, exist_ok=True)
    return dir


def groundTruthColor(groundTruthPath):
    if "arkit" in groundTruthPath:
        return "lime"
    elif "gps" in groundTruthPath:
        return "red"
    elif "tango" in groundTruthPath:
        return "magenta"
    return "orange"


def runComputeMetrics(q, fn):
    try:
        q.put(fn())
    except Exception:
        track = traceback.format_exc()
        print(track)
        q.put(-1) # We must return something, otherwise queue.get() waits forever


def computeMetrics(fn):
    queue = multiprocessing.Queue()
    p = multiprocessing.Process(target=runComputeMetrics, args=(queue, fn))
    p.start()
    result = queue.get(timeout=600)
    p.join(600)
    if p.is_alive(): # Terminate still running i.e. join timeout triggered
        p.terminate()
        raise Exception("Plotting failed, ran into 10 minute timeout")
    if result == -1: # runComputeMetrics will return -1 on error
        raise Exception("Plotting failed, return -1")
    return result


def loadBenchmarkComparisonData(benchmarkComparison, caseDir, groundTruth, jsonlFile, gtCopy, gtJson, slamMapFile):
    if benchmarkComparison and benchmarkComparison.endswith(".csv"):
        gtFile = caseDir + "/" + benchmarkComparison
        if not os.path.exists(gtFile):
            raise Exception("Requested benchmarkComparison doesn't exist: " + gtFile)
        subprocess.run(["cp", gtFile, gtCopy])
        return

    gtFile = caseDir + "/" + groundTruth
    if not benchmarkComparison and os.path.exists(gtFile):
        subprocess.run(["cp", gtFile, gtCopy])
        return

    if not os.path.exists(jsonlFile):
        return

    gpsConverter = GpsToLocalConverter()
    rtkgpsConverter = GpsToLocalConverter()
    dataGroundTruth = []
    dataARKit = []
    dataARCore = []
    dataAREngine = []
    dataOnDevice = []
    dataRealsense= []
    dataGps = []
    dataRtkGps = []
    with open(jsonlFile) as f:
        for line in f.readlines():
            dataRow = json.loads(line)
            pos = None
            if dataRow.get("groundTruth") is not None:
                pos = dataRow["groundTruth"]["position"]
                dataGroundTruth.append([dataRow["time"], pos["x"], pos["z"], -pos["y"]])
            elif dataRow.get("ARKit") is not None:
                pos = dataRow["ARKit"]["position"]
                dataARKit.append([dataRow["time"], pos["x"], pos["z"], -pos["y"]])
            elif dataRow.get("arcore") is not None:
                pos = dataRow["arcore"]["position"]
                dataARCore.append([dataRow["time"], pos["x"], pos["y"], pos["z"]])
            elif dataRow.get("arengine") is not None:
                pos = dataRow["arengine"]["position"]
                if pos and pos["x"]: # AREngine sometimes gives nulls, skip them
                    dataAREngine.append([dataRow["time"], pos["x"], pos["y"], pos["z"]])
            elif dataRow.get("output") is not None:
                pos = dataRow["output"]["position"]
                dataOnDevice.append([dataRow["time"], -pos["x"], pos["z"], pos["y"]])
            elif dataRow.get("realsense") is not None:
                pos = dataRow["realsense"]["position"]
                dataRealsense.append([dataRow["time"], pos["x"], pos["y"], pos["z"]])
            elif dataRow.get("gps") is not None:
                pos = gpsConverter.convert(**dataRow["gps"])
                dataGps.append([dataRow["time"], pos["x"], pos["z"], -pos["y"]])
            elif dataRow.get("rtkgps") is not None:
                pos = rtkgpsConverter.convert(**dataRow["rtkgps"])
                dataRtkGps.append([dataRow["time"], pos["x"], pos["z"], -pos["y"]])


    dataWrite = None
    if (not benchmarkComparison or benchmarkComparison == "groundTruth") and dataGroundTruth:
        dataWrite = dataGroundTruth
    elif (not benchmarkComparison or benchmarkComparison == "ARKit") and dataARKit:
        dataWrite = dataARKit
    elif (not benchmarkComparison or benchmarkComparison == "ARCore") and dataARCore:
        dataWrite = dataARCore
    elif (not benchmarkComparison or benchmarkComparison == "AREngine") and dataAREngine:
        dataWrite = dataAREngine
    elif (not benchmarkComparison or benchmarkComparison == "OnDevice") and dataOnDevice:
        dataWrite = dataOnDevice
    elif (not benchmarkComparison or benchmarkComparison == "RealSense") and dataRealsense:
        dataWrite = dataRealsense
    elif (not benchmarkComparison or benchmarkComparison == "GPS") and dataGps:
        dataWrite = dataGps
    elif (not benchmarkComparison or benchmarkComparison == "RTKGPS") and dataRtkGps:
        dataWrite = dataRtkGps

    if not dataWrite and benchmarkComparison:
        raise Exception("Requested benchmarkComparison doesn't exist: " + benchmarkComparison)

    if dataWrite:
        with open(gtCopy, "w") as f:
            for entry in dataWrite:
                f.write(",".join(["{}".format(x) for x in entry]) + "\n")

        slamMap = []
        if os.path.exists(slamMapFile):
            with open(slamMapFile) as f:
                for line in f.readlines():
                    y = [float(x) for x in line.split(",")]
                    slamMap.append([y[0], -y[1], y[3], y[2]])

        with open(gtJson, "w") as f:
            def addDataSet(array, name, dataset, benchmark):
                if dataset:
                    if benchmark == name:
                        array.insert(0, {'name': name, 'data': dataset})
                    else:
                        array.append({'name': name, 'data': dataset})

            datasets = []
            addDataSet(datasets, "groundTruth", dataGroundTruth, benchmarkComparison)
            addDataSet(datasets, "ARKit", dataARKit, benchmarkComparison)
            addDataSet(datasets, "ARCore", dataARCore, benchmarkComparison)
            addDataSet(datasets, "AREngine", dataAREngine, benchmarkComparison)
            addDataSet(datasets, "slam", slamMap, benchmarkComparison)
            addDataSet(datasets, "OnDevice", dataOnDevice, benchmarkComparison)
            addDataSet(datasets, "RealSense", dataRealsense, benchmarkComparison)
            addDataSet(datasets, "GPS", dataGps, benchmarkComparison)
            addDataSet(datasets, "RTKGPS", dataRtkGps, benchmarkComparison)
            json.dump({'datasets': datasets}, f)

def singleBenchmark(benchmark, dirs, vioTrackingFn, gtColor, cArgs):
    args = cArgs
    caseDir = benchmark.dir
    caseName = benchmark.name
    jsonlFile = caseDir + "/data.jsonl"
    outputFile = "{}/{}.csv".format(dirs.out, caseName)
    slamMapFile = "{}/{}.csv".format(dirs.slamMaps, caseName)
    gtCopy = "{}/{}.csv".format(dirs.gt, caseName)
    gtJson = "{}/{}.json".format(dirs.gt, caseName)
    figure = "{}/{}.png".format(dirs.figures, caseName)
    endPosFile = "{}/{}.txt".format(dirs.endpos, caseName)
    datasetInfo = "{}/{}.json".format(dirs.info, caseName)

    start = time.time()

    failed = True
    for i in range(5): # Attempt to run benchmark 5 times if it fails
        # Clear output files from previous failed runs
        if os.path.exists(outputFile): os.remove(outputFile)
        if os.path.exists(slamMapFile): os.remove(slamMapFile)
        # Run tracking
        vioTrackingFn(args, benchmark, dirs.results, outputFile, slamMapFile)
        duration = time.time() - start
        # Verify that output file has correct number of columns, if not retry
        with open(outputFile, "r") as f:
            columns = {}
            for line in f.readlines():
                columns[line.count(',')] = True
            if len(columns) == 1:
                # Success, all rows have same number of columns
                failed = False
                break
        print("Warning! Failed attempt number {}. Unequal number of colums in output file".format(i))

    if failed:
        raise Exception("Error! Failed to run benchmark 5 times")

    loadBenchmarkComparisonData(benchmark.benchmarkComparison, caseDir, args.groundTruth, jsonlFile, gtCopy, gtJson, slamMapFile)

    metric = computeMetrics(partial(compute_metrics, folder=dirs.results, casename=caseName, figure_output=figure, **metricsKwargs(args, single=True)))
    if metric:
        print("{}: {:.2f} seconds, metric: {:.2f}".format(caseName, duration, metric))
    else:
        print("{}: {:.2f} seconds, no ground-truth".format(caseName, duration))

    endPosition = readLastLine(outputFile).split(",")[3:5]
    with open(endPosFile, "w") as f:
        f.write(caseName + "," + ",".join(endPosition) + "\n")

    hostname = socket.gethostname()
    if args.displayHostname:
        firstPart = hostname.find('.')
        if firstPart > 0:
            hostname = hostname[0:firstPart]
    else:
        hostname = None

    with open(datasetInfo, "w") as f:
        f.write(json.dumps({
            "caseName": caseName,
            "dir": benchmark.dir,
            "paramSet": benchmark.paramSet,
            "origName": benchmark.origName,
            "hostname": hostname
        }))

    return True


def aggregateBenchmarks(args, dirs, gtColor, params, infoTxt, runId, deterministic=False):
    print("Aggregating benchmarks...")

    with open(dirs.results + "/end_positions.txt", 'w') as f:
        for x in os.walk(dirs.endpos):
            for file in x[2]:
                with open(os.path.join(x[0], file)) as f2:
                    f.write(f2.read())

    metricsFile = dirs.results + "/metrics.csv"
    if os.path.exists(metricsFile):
        os.remove(metricsFile)
    #    raise Exception("Metrics file already exists, have you already ran the report aggregation for this benchmark runId?")
    summaryFigure = dirs.results + "/figures.png"
    summaryFigureZ = dirs.results + "/figures-z-axis.png"

    if metricsFile:
        with open(metricsFile, "a") as f: f.write("case,metrics\n")
    else: sys.stdout.write("case,metrics\n")

    compute_metrics(folder=dirs.results, figure_output=summaryFigure, title=params, metricFile=metricsFile, **metricsKwargs(args))
    compute_metrics(folder=dirs.results, figure_output=summaryFigureZ, title=params, metricFile=None, z_axis=True, **metricsKwargs(args))

    slamMapFigure = dirs.results + "/slam-maps.png"
    draw_slam_maps(map_dir=dirs.slamMaps, figure_output=slamMapFigure)

    metricsLines = 0
    with open(metricsFile, 'r') as f:
        for line in f.readlines():
            metricsLines += 1
    mean = 0
    if metricsLines > 1:
        sum = 0
        with open(metricsFile, "r") as f:
            next(f) # Skip header row
            for line in f:
                sum += float(line.strip().split(",")[1])
        mean = sum / (metricsLines - 1)
        print("mean: {}".format(mean))
        with open(infoTxt, "a") as f:
            f.write("mean metrics: {}\n".format(mean))

    # Also put summary figures to their own folder.
    Path(args.output + "/figures/").mkdir(parents=True, exist_ok=True)
    subprocess.run(["cp", summaryFigure, args.output + "/figures/" + runId + ".png"])

    if deterministic:
        compare = []
        for x in os.walk(dirs.logs):
            for file in sorted(x[2]):
                inn = os.path.join(x[0], file)
                out = os.path.join(x[0], os.path.splitext(os.path.basename(file))[0] + "_deterministic.txt")
                compare.append(out)
                with open(inn) as f, open(out, "w") as f2:
                    for line in f:
                        if not "Timings" in line and not "timings" in line: # Filter out most of the varying output
                            f2.write(line)
        for i0, f0 in enumerate(compare):
            for i1, f1 in enumerate(compare):
                if i0 >= i1:
                    continue
                print("Comparing {} to {}", i0 + 1, i1 + 1)
                diffLineCount = int(subprocess.Popen("diff -y --suppress-common-lines " + f0 + " " + f1 + " | wc -l",
                    shell=True, stdout=subprocess.PIPE, universal_newlines=True).communicate()[0])
                #         stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                if diffLineCount < 20:
                    print("Diff line count {}, within acceptable limits".format(diffLineCount))
                else:
                    print("Diff line count {}".format(diffLineCount))
                    # print("Opening file")
                    # subprocess.run("/usr/bin/opendiff " + f0 + " " + f1, shell=True)

    # Finally print end time
    with open(infoTxt, "a") as f:
        f.write("end time:" + datetime.now().strftime(DATE_FORMAT) + "\n")


def loadBenchmarkSet(args, setName):
    # Look for the set file in a predefined directory or by path.
    searchDirs = [args.setDir + "/", os.getcwd(), ""]
    success = False
    for searchDir in searchDirs:
        try:
            path = searchDir + setName
            if not ".json" in path:
                path += ".json"
            setFile = open(path)
            success = True
            break
        except FileNotFoundError:
            pass
    if not success:
        raise Exception("Benchmark set %s not found" % setName)

    setDefintion = json.loads(setFile.read())
    pathPrefix = args.dir
    benchmarks = []
    parameterSets = [{}]
    if setDefintion.get("parameterSets") != None:
        parameterSets = setDefintion["parameterSets"]
        print("For {} parameters sets: {}".format(len(setDefintion["parameterSets"]),
            ", ".join("[" + s["params"] + "]" for s in setDefintion["parameterSets"])))
    for benchmark in setDefintion["benchmarks"]:
        for parameterSet in parameterSets:
            dir = (pathPrefix + "/" + benchmark["folder"])
            params = []
            if parameterSet.get("params"): params.append(parameterSet["params"])
            if benchmark.get("params"): params.append(benchmark["params"])
            name = benchmark.get("name").replace(' ', '-') if benchmark.get("name") else os.path.basename(dir)
            if parameterSet.get("name"): name = "{}-{}".format(name, parameterSet.get("name").replace(' ', '-'))
            benchmarks.append(Benchmark(
                dir=dir,
                params=(None if len(params) == 0 else " ".join(params)),
                name=name,
                origName=benchmark.get("name"),
                benchmarkComparison=benchmark.get("benchmarkComparison"),
                paramSet=parameterSet.get("name")
            ))
    x = lambda s : "{} ({})".format(s["folder"], s["params"]) if s.get("params") else s["folder"]
    print("Running {} benchmark datasets:\n  {}".format(len(setDefintion["benchmarks"]),
        "\n  ".join(x(s) for s in setDefintion["benchmarks"])))

    if len(benchmarks) != len(set(b.name for b in benchmarks)):
        raise Exception("All benchmarks don't have unique names! Make sure all parameterSets and duplicate data sets have 'name' field")
    return benchmarks


def loadBenchmarks(args, tags):
    cases = deque([])
    if tags:
        # Filter datasets that match given tags.
        # `find` -L flag finds the files in symlinked folders.
        infoFiles = []
        for x in os.walk(args.dir):
            for file in x[2]:
                if file == "info.json":
                    infoFiles.append(os.path.join(x[0], file))
        infoFiles.sort()
        for infoFile in infoFiles:
            path = filterByTags(infoFile, tags)
            if path:
                cases.append(path)
    else:
        # Run every dataset. Most useful with the `-dir` option.
        folders = next(os.walk(args.dir))[1]
        folders.sort()
        for x in folders:
            cases.append(os.path.join(args.dir, x))

    # When doing parallel benchmarks, depending on the filesystem it may improve performance
    # to avoid reading the same files concurrently, so this reorders the case list.
    if args.rotateCases:
        cases.rotate(-int(args.rotateCases))

    print("Running benchmarks:\n  " + "\n  ".join([os.path.basename(c) for c in cases]))
    return [Benchmark(case) for case in cases]


def benchmark(args, vioTrackingFn, setupFn=None, teardownFn=None):
    startTime = datetime.now()
    now = startTime.strftime(DATE_FORMAT)

    if args.skipBenchmark and not args.runId:
        raise Exception("skipBenchmark is true and runId isn't set , cannot aggregate existing results without runId")
    if args.skipBenchmark and args.runId and not os.path.exists(args.output + "/" + args.runId):
        raise Exception("Benchmark folder for runId doesn't exist, cannot aggregate report")
    runId = args.runId if args.runId else now

    # Direcotries
    class Dirs:
        results = withMkdir(args.output + "/" + runId)
        gt = withMkdir(results + "/ground-truth")
        out = withMkdir(results + "/output")
        slamMaps = withMkdir(results + "/slam-maps")
        figures = withMkdir(results + "/figures")
        logs = withMkdir(results + "/logs")
        endpos = withMkdir(results + "/endpositions")
        info = withMkdir(results + "/info")
    dirs = Dirs()

    if setupFn:
        setupFn(args, dirs.results)

    if not args.skipAggregate: # When skipping aggregate, shouldn't touch any shared files
        infoTxt = dirs.results + "/info.txt"
        with open(infoTxt, "w") as f:
            f.write("start time: " + datetime.now().strftime(DATE_FORMAT) + "\n")
            # f.write("main flags: " + mainFlags + "\n")
            if runAndCapture("command -v shasum"):
                f.write("fingerprint:" + runAndCapture("shasum -a 256 " + dirs.results + "/main") + "\n")
            f.write("system:" + runAndCapture("uname -a") + "\n")
            f.write("odometry commit:"
                + runAndCapture("git rev-parse --short HEAD")
                + runAndCapture("git rev-parse --abbrev-ref HEAD")
                + "\n")
            subprocess.run("git show > \"" + dirs.results +  "/git.odometry.show\"", shell=True)
            subprocess.run("git diff > \"" + dirs.results +  "/git.odometry.diff\"", shell=True)
            subprocess.run("git diff --staged > \"" + dirs.results +  "/git.odometry.staged.diff\"", shell=True)
        with open(infoTxt, 'r') as f:
            print(f.read())

    gtColor = groundTruthColor(args.groundTruth)

    if not args.skipBenchmark:
        benchmarks = None
        if args.set:
            benchmarks = loadBenchmarkSet(args, args.set)
        else:
            benchmarks = loadBenchmarks(args, args.tags)

        if args.distributedBatchSize and args.distributedBatchNumber:
            benchmarksFiltered = []
            for idx, b in enumerate(benchmarks):
                if idx % int(args.distributedBatchSize) == int(args.distributedBatchNumber):
                    benchmarksFiltered.append(b)
            benchmarks = benchmarksFiltered
            print("Distributed mode on. Only running following benchmarks: " + ", ".join([b.name for b in benchmarks]))

        if len(benchmarks) == 0:
            print("No matching benchmarks found! Exiting")
            return

        threadFunction = partial(singleBenchmark,
            dirs=dirs, vioTrackingFn=vioTrackingFn, gtColor=gtColor, cArgs=args)

        if args.threads == 1:
            for b in benchmarks:
                threadFunction(b)
        else:
            workers = int(args.threads) if args.threads else multiprocessing.cpu_count()
            print("Using {} threads for running benchmarks, change this with '-threads #'".format(workers))
            with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
                for i in executor.map(threadFunction, benchmarks):
                    assert(i)

    if not args.skipAggregate:
        aggregateBenchmarks(args, dirs, gtColor, args.params, infoTxt, runId, args.testDeterministic)

    if teardownFn:
        teardownFn(args, dirs.results)
