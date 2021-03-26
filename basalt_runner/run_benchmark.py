# Run benchmarks with Basalt

import subprocess
from benchmark.benchmark import benchmark, getArgParser
from pathlib import Path
import shutil

def main():
    parser = getArgParser()
    parser.add_argument("-calib", help="Camera calibration file")
    parser.add_argument("-config", help="Algorithm configuration file")
    parser.add_argument('-postprocess', help="Include post processed trajectory", action='store_true')
    args = parser.parse_args()

    def setupFn(args, outputDir):
        pass

    def tearDownFn(args, outputDir):
        pass

    def vioTrackingFn(args, benchmark, outputDir, outputFile, slamMap):
        logFile = "{}/{}/{}.txt".format(outputDir, "logs", benchmark.name)
        margFolder = "{}/{}/{}/".format(outputDir, "margdata", benchmark.name)
        margArg = ""
        if args.postprocess:
            Path(margFolder).mkdir(parents=True, exist_ok=True)
            margArg = " --marg-data " + margFolder
        subprocess.check_call("./target/basalt_runner"
                + " --dataset-path " + benchmark.dir
                + " --save-trajectory " + outputFile
                + " --config-path " + args.config
                + " --cam-calib " + args.calib
                + margArg
                + " > " + logFile + " 2>&1", shell=True)
        if args.postprocess:
            mapFn = outputFile.rsplit(".")[0] + "_map.jsonl"
            subprocess.check_call("./target/basalt_mapper"
                    + " --show-gui 0"
                    + " --config-path " + args.config
                    + " --cam-calib " + args.calib
                    + margArg
                    + " --output-path " + mapFn
                    + " > " + logFile + " 2>&1", shell=True)
            shutil.rmtree(margFolder)

    benchmark(args, vioTrackingFn, setupFn, tearDownFn)

    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    main()
