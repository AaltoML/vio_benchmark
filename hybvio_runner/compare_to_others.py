# Produce good numbers for our method on specific datasets and compare to results reported by other methods.
#
# The EuRoC "postprocess" results are copied from the paper “ORB-SLAM3: An Accurate Open-Source Library
# for Visual, Visual-Inertial and Multi-Map SLAM”. The TUM VI results are copied from the paper
# “The TUM VI Benchmark for Evaluating Visual-Inertial Odometry”, with the exception of ORB-SLAM3 result
# from their own paper.

# The ORB-SLAM3 paper explains the results are (with the exception of visual-only mono methods) computed using
# SE(3) RMS ATE (Root Mean Square Absolute Trajectory Error) over the whole sequence — the standard metric for
# VIO methods which have access to scale information either through stereo cameras or IMU. In our benchmark
# framework this metric is enabled with `-metricSet full_3d_align` (RMSE).
#
# The EuRoC "VIO" results are copied from the paper “Visual-Inertial Mapping with Non-Linear Factor Recovery”
# (the Basalt paper).
#
# The ZJU/SenseTime results are presented in the paper “Survey and Evaluation of Monocular
# Visual-Inertial SLAM Algorithms for Augmented Reality” and were copied from here
# <http://www.zjucvg.net/eval-vislam/evaluation/>.
#
# The difference between "VIO" and "postprocess" categories is that the former is the "online" result
# of the given algorithm, while the latter allows updating past parts of the map. SLAM methods seem
# to prefer report their results after such postprocessing, which of course tends to improve the numbers.

import argparse
import json
import numpy as np
import os
import re
import subprocess
import shutil
import tempfile
from datetime import datetime
from benchmark.benchmark import getArgParser
from run import runBenchmark
from convert.output_to_tum import outputToTum

parser = argparse.ArgumentParser(description="Compare benchmark results against other VIO methods on popular datasets")
parser.add_argument("dataset", help="'euroc', 'tum', 'sensetime', or 'all'")
parser.add_argument("--vislamEvalBin", help="Path to bin/ folder of compiled `eval-vislam`. Required for -dataset=sensetime", default=None)
parser.add_argument("--runId", help="Skip benchmark run and use previous results from this folder", default=None)
parser.add_argument("--mono", help="Run in monocular mode", default=False, action="store_true")
parser.add_argument("--noSlam", help="Run without SLAM", default=False, action="store_true")
parser.add_argument("--raspberry", help="Run with Raspberry Pi 4 settings", default=False, action="store_true")
parser.add_argument("--postprocess", help="Improve final result by using SLAM techniques", default=False, action="store_true")
parser.add_argument("--frameTimes", help="Measures processing time for each benchmark", default=False, action="store_true")

DATE_FORMAT = "%Y-%m-%d_%H-%M-%S"
BENCHMARK_DIR = "target/benchmark-results"
OUR_METHOD_NAME = "Spectacular AI"
OUTPUT_DIR = "target/results"
OTHERS_RESULTS_DIR = "comparison_data"
SENSETIME_RAW = "../data/raw/sensetime"
DATASET_DIR = "../data/benchmark"
SET_DIR = "benchmark/sets"

def evalVislam(args, sequences, runId):
    assert(args.vislamEvalBin)
    tmpDir = tempfile.mkdtemp()
    metrics = []
    # `parse_args()` with list argument ignores `sys.argv`.
    tumArgs = getArgParser().parse_args([])
    tumArgs.header = False
    for sequence in sequences:
        # Convert JSONL to format `vislam-eval` understands.
        tumArgs.inFile = "{}/{}/output/{}.jsonl".format(BENCHMARK_DIR, runId, sequence)
        tumArgs.outFile = "{}/{}.txt".format(tmpDir, sequence)
        tumArgs.csv = False
        outputToTum(tumArgs)

        # Run `vislam-eval` and grab the APE result.
        cmd = "{}/accuracy {}/{} {}".format(args.vislamEvalBin, SENSETIME_RAW, sequence, tumArgs.outFile)
        output = subprocess.run(cmd, capture_output=True, shell=True, encoding="utf-8")
        if output.stderr.strip():
            print(output.stderr.strip())
        m = re.search("APE:\s+(\d+\.\d*)", output.stdout.strip())

        assert(m and m.group(1))
        metrics.append(float(m.group(1)))

    shutil.rmtree(tmpDir)
    return metrics

DATASETS = {
    "euroc": {
        "vioResults": "vio_basalt_euroc.csv",
        "postprocessResults": "orbslam3_euroc.csv",
        "benchmarkSet": "euroc",
        # Same as contents of the JSON pointed by `benchmarkSet`, but included to make sure order
        # isn't accidentally changed.
        "sets": [
            "euroc-mh-01-easy",
            "euroc-mh-02-easy",
            "euroc-mh-03-medium",
            "euroc-mh-04-difficult",
            "euroc-mh-05-difficult",
            "euroc-v1-01-easy",
            "euroc-v1-02-medium",
            "euroc-v1-03-difficult",
            "euroc-v2-01-easy",
            "euroc-v2-02-medium",
            "euroc-v2-03-difficult",
        ],
        "metricSet": "full_3d_align",
        "metricFormula": "RMSE",
        "vioParameters": "-maxSuccessfulVisualUpdates=20",
        # NOTE `-cameraTrailHanoiLength=4` omitted and given in `compute_paper_results.py` instead.
        "raspberryParameters": "-cameraTrailLength=6 -featureDetector=FAST -pyrLKMaxIter=8 -pyrLKWindowSize=13 -ransacMaxIters=30 -subPixMaxIter=0",
        "postprocessParameters": "-maxSuccessfulVisualUpdates=20 -localBAProblemSize=100 -adjacentSpaceSize=50",
    },
    # The TUM VI dataset has full ground truth only for the sequences confined to "the room"
    # where the tracking hardware is. As noted by the ORB-SLAM3 paper, in the other sequences
    # this amounts to the RMS ATE almost directly measuring the drift accumulated while out
    # of the room, which is rather different and maybe not as interesting metric.
    "tum": {
        # NOTE Some of the ORB-SLAM3 paper results are "postprocess" category results.
        "vioResults": "orbslam3_tum.csv",
        "postprocessResults": "orbslam3_tum.csv",
        "benchmarkSet": "tum-vi-room",
        "sets": [ "room1", "room2", "room3", "room4", "room5", "room6" ],
        "metricSet": "full_3d_align",
        "metricFormula": "RMSE",
        "vioParameters": "",
        "raspberryParameters": "-cameraTrailLength=6 -featureDetector=FAST -pyrLKMaxIter=8 -pyrLKWindowSize=13 -ransacMaxIters=30 -subPixMaxIter=0",
        "postprocessParameters": "-localBAProblemSize=100 -adjacentSpaceSize=50",
    },
    # The "APE" reported by the authors' benchmark software `eval-vislam` uses the same
    # formula as full_3d_align-RMSE but the results from our benchmark differ somewhat.
    # Probably the reason is that `eval-vislam` may ignore early part of the trajectory
    # (initialization time), the track alignment algorithm is different, and other subtle
    # factors.
    "sensetime": {
        "vioResults": "zjucvg_sensetime.csv",
        "postprocessResults": "zjucvg_sensetime.csv",
        "benchmarkSet": "sensetime-a",
        "sets": [ "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7" ],
        "metricSet": "full_3d_align",
        "metricFormula": "RMSE",
        # -targetFps=30 required for the completeness metric to work.
        "vioParameters": "-outputCameraPose -targetFps=30 -odometryPriorStrengthPosition=2000",
        "raspberryParameters": "-outputCameraPose -targetFps=30 -odometryPriorStrengthPosition=2000 -cameraTrailLength=6 -featureDetector=FAST -pyrLKMaxIter=8 -pyrLKWindowSize=13 -ransacMaxIters=30 -subPixMaxIter=0",
        "postprocessParameters": "-outputCameraPose -targetFps=30 -odometryPriorStrengthPosition=2000 -localBAProblemSize=100 -adjacentSpaceSize=50",
        "benchmarkFunction": evalVislam,
        "monoOnly": True,
    },
}

def runDataset(args, datasetName, dataset, extraParameters=""):
    isMono = args.mono or "monoOnly" in dataset
    monoForStereoSet = args.mono and not "monoOnly" in dataset
    if args.runId:
        runId = args.runId
    else:
        # `parse_args()` with list argument ignores `sys.argv`.
        runArgs = getArgParser().parse_args([])
        runArgs.copyBinary = True
        runArgs.postprocess = False
        runArgs.postprocessWithInterpolation = False
        runArgs.logLevel = 0
        # Name of `json` file which lists the datasets.
        runArgs.set = dataset["benchmarkSet"]
        runArgs.metricSet = dataset["metricSet"]
        runArgs.dir = DATASET_DIR
        runArgs.setDir = SET_DIR
        runArgs.frameTimes = args.frameTimes

        # Hand-tuned parameters that give good results on the dataset.
        if args.postprocess:
            assert("postprocessParameters" in dataset)
            assert("postprocessResults" in dataset)
            runArgs.params = dataset["postprocessParameters"]
            runArgs.postprocessWithInterpolation = True
        elif args.raspberry:
            assert("raspberryParameters" in dataset)
            assert("vioResults" in dataset)
            runArgs.params = dataset["raspberryParameters"]
            if isMono:
                runArgs.params += " -maxTracks=100"
            else:
                runArgs.params += " -maxTracks=70"
        else:
            assert("vioParameters" in dataset)
            assert("vioResults" in dataset)
            runArgs.params = dataset["vioParameters"]

        # Hack to allow overwriting parameters set in `vioParameters`.
        if "CLEAR" in extraParameters:
            runArgs.params = ""
            extraParameters = extraParameters.replace("CLEAR", "")

        if isMono:
            runArgs.params += " -useStereo=false"
        else:
            runArgs.params += " -useStereo"

        if args.noSlam:
            runArgs.params += ' -useSlam=false'
        else:
            runArgs.params += ' -useSlam'

        if extraParameters:
            runArgs.params += ' ' + extraParameters

        if args.frameTimes:
            runArgs.params += " -processingQueueSize=10"
            runArgs.threads = 1
            if not args.noSlam:
                runArgs.params += " -slamThread=true"

        runId = datetime.now().strftime(DATE_FORMAT)
        runArgs.runId = runId
        print("Using parameters:", runArgs.params)
        runBenchmark(runArgs)

    metrics = []
    frameTimes = []
    if "benchmarkFunction" in dataset:
        metrics = dataset["benchmarkFunction"](args, dataset["sets"], runId)
    else:
        formula = dataset["metricFormula"]
        for s in dataset["sets"]:
            with open("/".join([BENCHMARK_DIR, runId, "results", s + ".json"])) as f:
                results = json.load(f)
                metrics.append(results["methods"]["our"][formula])
                frameTimes.append(results["duration"] / results["frameCount"] * 1000) # s -> ms

    # Copy results from other methods and append our own.
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    results = dataset["postprocessResults"] if args.postprocess else dataset["vioResults"]
    src = "{}/{}".format(OTHERS_RESULTS_DIR, results)
    postfix = "-mono" if monoForStereoSet else ""
    name = "{}/{}_{}{}".format(OUTPUT_DIR, runId, datasetName, postfix)
    dst = "{}.csv".format(name)
    shutil.copyfile(src, dst)
    with open(dst, "a") as metricsFile, open("{}-frameTimes.csv".format(name), "a") as frameTimeFile:
        if monoForStereoSet:
            metricsFile.write("Monocular Inertial\n")
        metricsFile.write(OUR_METHOD_NAME )
        if monoForStereoSet:
            metricsFile.write(" MONO")
        if args.postprocess:
            metricsFile.write(" post-processed")
        metricsFile.write(" (our),")
        for metric in metrics:
            metricsFile.write("{:.3g},".format(metric))
        metricsFile.write("{:.3g}\n".format(np.mean(metrics)))
        if frameTimes:
            for frameTime in frameTimes:
                frameTimeFile.write("{:.3f},".format(frameTime))
            frameTimeFile.write("{:.3f}\n".format(np.mean(frameTimes)))
    for srcBase in ["figures.png", "figures-z-axis.png"]:
        src = "/".join([BENCHMARK_DIR, runId, srcBase])
        dst = "{}_{}".format(name, srcBase)
        shutil.copyfile(src, dst)

    return {"metrics": metrics, "frameTimes": frameTimes}

def main(args):
    # Crash early if forgot this.
    if args.dataset in ["sensetime", "all"]:
        assert(args.vislamEvalBin)

    if args.dataset == "all":
        for key, value in DATASETS.items():
            runDataset(args, key, value)
    else:
        runDataset(args, args.dataset, DATASETS[args.dataset])

if __name__ == "__main__":
    main(parser.parse_args())
