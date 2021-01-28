import argparse
import json
import os
import shutil
import subprocess

parser = argparse.ArgumentParser(description="Semi automated tool for adding timeOffsets")
parser.add_argument("dir", help="Directory containing datasets you wish to augment with timeOffset data")


def getFps(videoFile):
    fpsDiv = subprocess.run("ffprobe -v error -select_streams v -of default=noprint_wrappers=1:nokey=1 -show_entries stream=avg_frame_rate {}".format(videoFile), capture_output=True, shell=True, encoding="utf-8").stdout.strip()
    fps = eval(fpsDiv) # ffprobe out has format like "60/1", evaluate the division.
    return fps

def processFolder(folder):
    timeOffset = None
    frameOffset = None
    infoJson = folder + "/info.json"
    if os.path.exists(infoJson):
        with open(infoJson) as f:
            data = json.load(f)
            timeOffset = data.get("timeOffset")
            frameOffset = data.get("frameOffset")

    if timeOffset or frameOffset:
        print("Time offset for {} already exists, skipping".format(folder))
        return

    tmpFolder = folder + "/tmp"
    if os.path.exists(tmpFolder):
        shutil.rmtree(tmpFolder)
    os.mkdir(tmpFolder)

    videoFile = subprocess.run("find {} | grep -E \"data\.[avi|mp4|mov]\"".format(folder), capture_output=True, shell=True, encoding="utf-8").stdout.strip()
    if not videoFile:
        raise Exception("Could not find video file in {}.".format(folder))

    # Sample images for every video over roughly the same period of time by skipping frames.
    skip = int(round(getFps(videoFile) / 15))
    skipFilter = ""
    if skip > 1:
        skipFilter = "select=\"not(mod(n\,{}))\",".format(skip)
    else:
        skip = 1

    print("Converting video to image sequence...")
    # Save space with grayscale and scaling.
    subprocess.run("ffmpeg -hide_banner -loglevel panic -t 999 -r {} -i {} -vf {}format=gray,scale=-2:640 -r 1 \"{}/tmp/$filename%03d.png\"".format(skip, videoFile, skipFilter, folder), shell=True)
    print("Go to folder {} and find the frame with visual cue in it, indexing should start from 1.".format(tmpFolder))
    frameOffset = int(input("Input frame number of the cue: "))
    # Convert to zero index with -1.
    frameOffset = (frameOffset - 1) * skip

    cueTime = None
    dataJsonl = folder + "/data.jsonl"
    if os.path.exists(dataJsonl):
        with open(dataJsonl) as f:
            for line in f.readlines():
                entry = json.loads(line)
                frames = entry.get("frames")
                if frames:
                    if entry.get("number") == frameOffset:
                        cueTime = entry.get("time")
                        break
        if not cueTime:
            raise Exception("Couldn't find frame number {} from data.jsonl".format(frameOffset))

    data = None
    if os.path.exists(infoJson):
        with open(infoJson) as f:
            data = json.load(f)
    else:
        data = {}

    if cueTime:
        data["timeOffset"] = cueTime
    data["frameOffset"] = frameOffset

    with open(infoJson, "w") as f:
        f.write(json.dumps(data, indent=4, sort_keys=True))

    print("Updated {} timeOffset to n={}, {} seconds".format(infoJson, frameOffset, cueTime))

    # Cleanup
    if os.path.exists(tmpFolder):
        shutil.rmtree(tmpFolder)


def main(args):
    for x in sorted(os.walk(args.dir)):
        for file in x[2]:
            if file == "info.json":
                processFolder(x[0])
    print("Done!")


if __name__ == "__main__":
    main(parser.parse_args())
