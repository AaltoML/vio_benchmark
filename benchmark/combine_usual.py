# Combine JSONL data from the "usual" directory hierarchy of simultaneous recordings.
#
# Example:
#
#   python vio_benchmark/benchmark/combine_usual.py data/benchmark/otaniemi/ttalo-08/realsense
#
# will create `data/benchmark/otaniemi/ttalo-08/realsense-combined/` with the basic
# data from the realsense folder and method results pulled from ttalo-08/[arengine|arkit|…].
#
# You need to run `vio_benchmark/benchmark/add_time_offset.py` prior to this script

import argparse
import os
import re
import shutil
import sys
from pathlib import Path

from .combine_jsonl import combineJSONL

parser = argparse.ArgumentParser()
parser.add_argument("target", help="Path to directory to create a modified copy of")
parser.add_argument("-arkit", help="Directory containing data.jsonl with ARKit tracking info")
parser.add_argument("-arcore", help="Directory containing data.jsonl with ARCore tracking info")
parser.add_argument("-arengine", help="Directory containing data.jsonl with AREngine tracking info")
parser.add_argument("-groundtruth", help="Directory containing data.jsonl with ground truth tracking info")
parser.add_argument("-realsense", help="Directory containing data.jsonl with Realsense tracking info")
parser.add_argument("-gps", help="Directory containing data.jsonl with GPS location info")

def main(args):
    def basePath(folder):
        return str(Path(args.target).parent / folder)

    targetName = os.path.basename(os.path.normpath(args.target))

    varsArgs = vars(args)
    for method in ["arkit", "arengine", "arcore", "realsense"]:
        if method != targetName and not varsArgs[method] and os.path.exists(basePath(method)):
            varsArgs[method] = method

    methods = []
    folders = []
    if args.arkit:
        methods.append("ARKit")
        folders.append(basePath(args.arkit))
    if args.arcore:
        methods.append("arcore")
        folders.append(basePath(args.arcore))
    if args.arengine:
        methods.append("arengine")
        folders.append(basePath(args.arengine))
    if args.groundtruth:
        methods.append("groundTruth")
        folders.append(basePath(args.groundtruth))
    if args.realsense:
        methods.append("realsense")
        folders.append(basePath(args.realsense))
    if args.gps:
        methods.append("gps")
        folders.append(basePath(args.gps))

    combined = basePath(targetName) + "-combined"
    combineJSONL(args.target, combined, folders, methods)

    for x in os.walk(args.target):
        for f in x[2]:
            if f == "data.jsonl":
                continue
            elif re.search("avi|mov|mp4", f):
                # Symlink large video files.
                os.symlink("../{}/{}".format(targetName, f), "{}/{}".format(combined, f))
            else:
                shutil.copyfile("{}/{}".format(args.target, f), "{}/{}".format(combined, f))

if __name__ == "__main__":
    main(parser.parse_args())
