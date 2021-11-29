#!/usr/bin/env python

"""
See the script `compare_to_others.py` for more information and sources for the data.
"""

import argparse
from datetime import datetime
import os

from compare_to_others import DATASETS, parser as compareParser, runDataset

OUTPUT_DIR = "target/results"
DATE_FORMAT = "%Y-%m-%d_%H-%M-%S"

RESULTS = {
    "tum": {
        "header": "Method,R1,R2,R3,R4,R5,R6",
# VINS Mono,0.07,0.07,0.11,0.04,0.20,0.08
# ROVIO,0.16,0.33,0.15,0.09,0.12,0.05
        "online_stereo": """
OKVIS,0.06,0.11,0.07,0.03,0.07,0.04
BASALT,0.09,0.07,0.13,0.05,0.13,0.02
ORB-SLAM3,0.008,0.012,0.011,0.008,0.010,0.006
""",
    },
    "sensetime": {
        "header": "Method,A0,A1,A2,A3,A4,A5,A6,A7",
# PTAM,75.442,113.406,67.099,10.913,21.007,40.403,19.483,13.503
# ORB-SLAM2,96.777,95.379,69.486,15.310,10.061,29.653,12.145,5.832
# LSD-SLAM,105.963,221.643,310.963,199.445,155.692,249.644,49.805,38.673
# DSO,231.860,431.929,216.893,188.989,115.477,323.482,14.864,27.142
# MSCKF,156.018,294.091,102.657,44.493,114.845,82.885,66.001,105.492
        "online_mono": """
OKVIS,71.677,87.73,68.381,22.949,146.89,77.924,63.895,47.465
VINS-Mono,63.395,80.687,74.842,19.964,18.691,42.451,26.240,18.226
SenseSLAM {\\tiny v1.0},58.995,55.097,36.370,17.792,15.558,34.810,20.467,10.777
""",
    },
    "euroc": {
        "header": "Method,MH01,MH02,MH03,MH04,MH05,V101,V102,V103,V201,V202,V203",
        "online_mono": """
OKVIS,0.34,0.36,0.30,0.48,0.47,0.12,0.16,0.24,0.12,0.22,-
PIVO,-,-,-,-,-,0.82,-,0.72,0.11,0.24,0.51
VINS-Fusion,0.18,0.09,0.17,0.21,0.25,0.06,0.09,0.18,0.06,0.11,-
VI-DSO,0.06,0.04,0.12,0.13,0.12,0.06,0.07,0.10,0.04,0.06,-
""",
        "online_stereo": """
OKVIS,0.23,0.15,0.23,0.32,0.36,0.04,0.08,0.13,0.10,0.17,-
VINS-Fusion,0.24,0.18,0.23,0.39,0.19,0.10,0.10,0.11,0.12,0.10,-
BASALT,0.07,0.06,0.07,0.13,0.11,0.04,0.05,0.10,0.04,0.05,-
""",
        "postprocessed_mono": """
VINS-Mono,0.084,0.105,0.074,0.122,0.147,0.047,0.066,0.180,0.056,0.090,0.244
VI-DSO,0.062,0.044,0.117,0.132,0.121,0.059,0.067,0.096,0.040,0.062,0.174
ORB-SLAM3,0.032,0.053,0.033,0.099,0.071,0.043,0.016,0.025,0.041,0.015,0.037
""",
        "postprocessed_stereo": """
OKVIS,0.160,0.220,0.240,0.340,0.470,0.090,0.200,0.240,0.130,0.160,0.290
VINS-Fusion,0.166,0.152,0.125,0.280,0.284,0.076,0.069,0.114,0.066,0.091,0.096
BASALT,0.080,0.060,0.050,0.100,0.080,0.040,0.020,0.030,0.030,0.020,-
Kimera,0.080,0.090,0.110,0.150,0.240,0.050,0.110,0.120,0.070,0.100,0.190
ORB-SLAM3,0.037,0.031,0.026,0.059,0.086,0.037,0.014,0.023,0.037,0.014,0.029
""",
    },
}

parser = argparse.ArgumentParser()
parser.add_argument("--vislamEvalBin", help="Path to bin/ folder of compiled `eval-vislam`. Required to compute SenseTime results.", default=None)
parser.add_argument('dataset', choices=['euroc', 'tum', 'sensetime'])
parser.add_argument('--outputDir', default=None)
parser.add_argument("--frameTimes", help="Measures processing time for each benchmark (may sacrifice determinism)", action="store_true")
args = parser.parse_args()

def saveTable(results, datasetName, category, outputDir):
    if not category in RESULTS[datasetName]:
        return
    text = RESULTS[datasetName]["header"]
    text += RESULTS[datasetName][category]
    text += "Ours"
    precision = 3
    for m in results["metrics"]:
        text += (",{:.%dg}" % precision).format(m)
    fileName = "{}/{}-{}.csv".format(outputDir, datasetName, category.replace("_", "-"))
    with open(fileName, "w") as f:
        f.write(text)

def setupVariants(path, datasetName):
    with open(path, "w") as f:
        f.write(RESULTS[datasetName]["header"] + "\n")

def addVariant(text, results, variantsPath):
    ft = text
    for m in results["metrics"]:
        text += ",{:.3g}".format(m)
    text += "\n"

    for d in results["frameTimes"]:
        ft += ",{:.3g}".format(d)
    ft += "\n"

    # Append results individually because it takes so long to run them
    # all that it might be practical to split the work to multiple sessions.
    with open(variantsPath + ".csv", "a") as f:
        f.write(text)
    with open(variantsPath + "-frameTimes.csv", "a") as f:
        f.write(ft)

def runDatasetFormat(argsList, dataset, extraParameters="", frameTimes=False):
    datasetArgs = compareParser.parse_args(argsList)
    datasetArgs.frameTimes = args.frameTimes
    name = datasetArgs.dataset
    print("\n---\n{} {}\n".format(argsList, extraParameters))
    return runDataset(datasetArgs, name, dataset, extraParameters)

def main():
    if args.outputDir:
        outputDir = "{}/{}".format(OUTPUT_DIR, args.outputDir)
    else:
        outputDir = "{}/{}".format(OUTPUT_DIR, datetime.now().strftime(DATE_FORMAT))
    print("Saving results to:", outputDir)
    os.makedirs(outputDir, exist_ok=True)

    datasetName = args.dataset
    assert(datasetName in ["sensetime", "euroc", "tum"])
    if datasetName == 'sensetime':
        def withVislamEval(argsList, dataset, extraParameters="", frameTimes=False):
            a = ["--vislamEvalBin", args.vislamEvalBin]
            a.extend(argsList)
            return runDatasetFormat(a, dataset, extraParameters, frameTimes)
        f = withVislamEval
        stereo = False
    else:
        f = runDatasetFormat
        stereo = True

    disabledFeatures = [
        "-noiseProcessBAA=0",
        "-useHybridRansac=false",
        "-scoreVisualUpdateTracks=false",
        "-useVisualStationarity=false",
        "-trackSampling=ALL",
        "-cameraTrailHanoiLength=0",
    ]

    dataset = DATASETS[datasetName]

    variantsPath = "{}/{}-variants".format(outputDir, datasetName)
    setupVariants(variantsPath + ".csv", datasetName)
    setupVariants(variantsPath + "-frameTimes.csv", datasetName)

    computeParameterVariants = datasetName == 'euroc'
    if computeParameterVariants:
        parameterVariantsPath = "{}/{}-parameter-variants".format(outputDir, datasetName)
        setupVariants(parameterVariantsPath + ".csv", datasetName)
        setupVariants(parameterVariantsPath + "-frameTimes.csv", datasetName)

    results = f(["--noSlam", "--mono", datasetName], dataset, " ".join(disabledFeatures))
    addVariant("PIVO reimpl.", results, variantsPath)

    results = f(["--noSlam", "--mono", datasetName], dataset)
    addVariant("Mono VIO (Normal)", results, variantsPath)

    results = f(["--noSlam", "--mono", datasetName], dataset, "-noiseProcessBAA=0")
    addVariant("- (no acc. bias walk)", results, variantsPath)

    results = f(["--noSlam", "--mono", datasetName], dataset, "-useHybridRansac=false")
    addVariant("- (no RANSAC)", results, variantsPath)

    results = f(["--noSlam", "--mono", datasetName], dataset, "-scoreVisualUpdateTracks=false")
    addVariant("- (random track sel.)", results, variantsPath)

    results = f(["--noSlam", "--mono", datasetName], dataset, "-useVisualStationarity=false")
    addVariant("- (no stationarity)", results, variantsPath)

    results = f(["--noSlam", "--mono", datasetName], dataset, "-trackSampling=ALL")
    addVariant("- (re-use full tracks)", results, variantsPath)

    results = f(["--noSlam", "--mono", "--raspberry", datasetName], dataset, "-cameraTrailHanoiLength=4")
    addVariant("Mono VIO (Fast)", results, variantsPath)

    results = f(["--noSlam", "--mono", "--raspberry", datasetName], dataset, "-cameraTrailHanoiLength=0")
    addVariant("- (no Hanoi trail)", results, variantsPath)

    results = f(["--mono", datasetName], dataset)
    saveTable(results, datasetName, "online_mono", outputDir)
    addVariant("Mono SLAM (Normal)", results, variantsPath)

    results = f(["--mono", "--postprocess", datasetName], dataset)
    saveTable(results, datasetName, "postprocessed_mono", outputDir)
    addVariant("Mono SLAM (Post-pr.)", results, variantsPath)

    if stereo:
        results = f(["--noSlam", "--raspberry", datasetName], dataset, "-cameraTrailHanoiLength=4")
        addVariant("Stereo VIO (Fast)", results, variantsPath)

        results = f(["--noSlam", "--raspberry", datasetName], dataset, "-cameraTrailHanoiLength=0")
        addVariant("- (STEREO no Hanoi trail)", results, variantsPath)

        results = f(["--noSlam", datasetName], dataset)
        addVariant("Stereo VIO (Normal)", results, variantsPath)

        results = f(["--noSlam", datasetName], dataset, "-trackSampling=ALL")
        addVariant("- (STEREO re-use full tracks)", results, variantsPath)

        results = f([datasetName], dataset)
        saveTable(results, datasetName, "online_stereo", outputDir)
        addVariant("Stereo SLAM (Normal)", results, variantsPath)
        if computeParameterVariants:
            addVariant("Stereo SLAM (Normal)", results, parameterVariantsPath)

        results = f(["--postprocess", datasetName], dataset)
        saveTable(results, datasetName, "postprocessed_stereo", outputDir)
        addVariant("Stereo SLAM (Post-pr.)", results, variantsPath)

    if computeParameterVariants:
        assert(stereo)
        results = f([datasetName], dataset, "-featureDetector=FAST")
        addVariant("feature detector FAST", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-subPixMaxIter=0")
        addVariant("no subpixel adjustment", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-maxTracks=70")
        addVariant("max. features 70", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-maxTracks=100")
        addVariant("max. features 100", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-maxTracks=300")
        addVariant("max. features 300", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-pyrLKMaxIter=8")
        addVariant("max. itr. 8", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-pyrLKMaxIter=40")
        addVariant("max. itr. 40", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-pyrLKWindowSize=13")
        addVariant("window size 13", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-pyrLKWindowSize=51")
        addVariant("window size 51", results, parameterVariantsPath)

        # Assertion `(unsigned)parameters.odometry.cameraTrailLength > parameters.slam.keyframeCandidateInterval * (parameters.slam.delayIntervalMultiplier + 1)' limits cameraTrailLength to minimimum of ~17.
        results = f([datasetName], dataset, "-cameraTrailLength=10 -delayIntervalMultiplier=0")
        addVariant("n_a 10", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-cameraTrailLength=30")
        addVariant("n_a 30", results, parameterVariantsPath)

        results = f([datasetName], dataset, "CLEAR -maxSuccessfulVisualUpdates=5")
        addVariant("n_{target} 5", results, parameterVariantsPath)

        results = f([datasetName], dataset, "CLEAR -maxSuccessfulVisualUpdates=10")
        addVariant("n_{target} 10", results, parameterVariantsPath)

        results = f([datasetName], dataset, "CLEAR -maxVisualUpdates=30 -maxSuccessfulVisualUpdates=30")
        addVariant("n_{target} 30", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-cameraTrailHanoiLength=0")
        addVariant("n_{FIFO} 20", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-cameraTrailHanoiLength=6")
        addVariant("n_{FIFO} 14", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-localBAProblemSize=50")
        addVariant("n_{BA} 50", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-localBAProblemSize=100")
        addVariant("n_{BA} 100", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-adjacentSpaceSize=35")
        addVariant("n_{matching} 35", results, parameterVariantsPath)

        results = f([datasetName], dataset, "-adjacentSpaceSize=50")
        addVariant("n_{matching} 50", results, parameterVariantsPath)

if __name__ == "__main__":
    main()
