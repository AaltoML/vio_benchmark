# Convert specialized RTK-GPS JSONL format to generic JSONL.

import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("input")
args = parser.parse_args()

output = []
with open(args.input) as f:
    for line in f.readlines():
        rtkgps = json.loads(line)
        output.append({
            "rtkgps": {
                "latitude": rtkgps["lat"],
                "longitude": rtkgps["lon"],
                "altitude": rtkgps["altitude"],
                "accuracy": rtkgps["accuracy"],
                "verticalAccuracy": rtkgps["verticalAccuracy"],
                "gpsTime": rtkgps["time"],
            },
            "time": rtkgps["time"],
        })

for line in output:
    print(json.dumps(line, separators=(',', ':')))
