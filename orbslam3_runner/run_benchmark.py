# Run benchmarks with ORB-SLAM3

import subprocess
from benchmark.benchmark import benchmark, getArgParser


def main():
    parser = getArgParser()

    parser.add_argument("-vocab", help="ORB vocabulary file", required=True)
    parser.add_argument("-config", help="Algorithm configuration file", required=True)
    args = parser.parse_args()

    def setupFn(args, outputDir):
        pass

    def tearDownFn(args, outputDir):
        pass

    def vioTrackingFn(args, benchmark, outputDir, outputFile, slamMap):
        logFile = "{}/{}/{}.txt".format(outputDir, "logs", benchmark.name)
        subprocess.check_call("./target/orbslam3_runner"
                + " " + args.vocab
                + " " + args.config
                + " " + benchmark.dir
                + " " + outputFile
                + " > " + logFile + " 2>&1", shell=True)

    benchmark(args, vioTrackingFn, setupFn, tearDownFn)

    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    main()
