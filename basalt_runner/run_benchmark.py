# Run benchmarks with Basalt

import subprocess
from benchmark.benchmark import benchmark, getArgParser


def main():
    parser = getArgParser()
    parser.add_argument("-calib", help="Camera calibration file")
    parser.add_argument("-config", help="Algorithm configuration file")
    args = parser.parse_args()

    def setupFn(args, outputDir):
        pass

    def tearDownFn(args, outputDir):
        pass

    def vioTrackingFn(args, benchmark, outputDir, outputFile, slamMap):
        subprocess.check_call("./target/basalt_runner"
                + " --dataset-path " + benchmark.dir
                + " --save-trajectory " + outputFile
                + " --config-path " + args.config
                + " --cam-calib " + args.calib, shell=True)

    benchmark(args, vioTrackingFn, setupFn, tearDownFn)

    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    main()
