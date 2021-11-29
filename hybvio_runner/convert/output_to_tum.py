# Convert from our JSONL (or old CSV) format to TUM format used by ZJU/SenseTime and UZH FPV online benchmarks:
# [t, px, py, pz, qx, qy, qz, qw] space separated

import argparse
import csv
import json

parser = argparse.ArgumentParser()
parser.add_argument("inFile")
parser.add_argument("outFile")
parser.add_argument('--no-header', dest='header', action='store_false')
parser.add_argument('--csv', action='store_true')
parser.set_defaults(header=True, csv=False)

def csvToTum(inFile, outFile):
    csvreader = csv.reader(inFile, delimiter=',')
    for row in csvreader:
        # Change order from wxyz to xyzw and conjugate the unit quaternion to get inverse.
        q = [-float(row[9]), -float(row[10]), -float(row[11]), float(row[8])]
        outFile.write("{} {} {} {} {} {} {} {}\n"
            .format(row[0], row[2], row[3], row[4], q[0], q[1], q[2], q[3]))

def jsonlToTum(inFile, outFile):
    for line in inFile.readlines():
        try:
            entry = json.loads(line)
        except:
            print("Ignoring bad JSONL line:", line)
            continue
        if "position" in entry:
            outFile.write("{} {} {} {} {} {} {} {}\n".format(
                entry["time"],
                entry["position"]["x"],
                entry["position"]["y"],
                entry["position"]["z"],
                # conjugate to get inverse
                -entry["orientation"]["x"],
                -entry["orientation"]["y"],
                -entry["orientation"]["z"],
                entry["orientation"]["w"],
            ))

def outputToTum(args):
    with open(args.inFile, 'r') as inFile, open(args.outFile, 'w') as outFile:
        if args.header:
            outFile.write("# timestamp_s tx ty tz qx qy qz qw\n")
        if args.csv:
            csvToTum(inFile, outFile)
        else:
            jsonlToTum(inFile, outFile)

if __name__ == "__main__":
    args = parser.parse_args()
    outputToTum(args)
