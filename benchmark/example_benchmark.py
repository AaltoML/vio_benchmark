# Example usage of benchmark.py library

import subprocess
from vio_benchmark.benchmark.benchmark import benchmark, getArgParser


def main():
    parser = getArgParser()
    parser.add_argument("-customArgumentsHere", help="Add any custom arguments you might need")
    args = parser.parse_args()

    def setupFn(args, outputDir):
        print("Any setup you need while knowing output direcotry")

    def tearDownFn(args, outputDir):
        print("Tear down after benchmark has ran successfully")

    def vioTrackingFn(args, benchmark, outputDir, outputFile, slamMap):
        subprocess.check_call("your_binary_here"
                + " -input=" + benchmark.dir
                + " -ouptut=" + outputFile
                + " -custom=" + args.customArgumentsHere, shell=True)

    benchmark(args, vioTrackingFn, setupFn, tearDownFn)

    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    main()
