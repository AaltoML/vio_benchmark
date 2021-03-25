# Run benchmarks with LARVIO

import subprocess
from benchmark.benchmark import benchmark, getArgParser


def main():
    parser = getArgParser()
    parser.add_argument("-config", help="Algorithm configuration file")
    args = parser.parse_args()

    def setupFn(args, outputDir):
        pass

    def tearDownFn(args, outputDir):
        pass

    def vioTrackingFn(args, benchmark, outputDir, outputFile, slamMap):
        logFile = "{}/{}/{}.txt".format(outputDir, "logs", benchmark.name)
        subprocess.check_call("./target/larvio_runner"
                + " " + benchmark.dir
                + " " + args.config
                + " " + outputFile
                + " novisu" # Don't want visualization for benchmark runs, visu enables it
                + " > " + logFile + " 2>&1", shell=True)

    benchmark(args, vioTrackingFn, setupFn, tearDownFn)

    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    main()
