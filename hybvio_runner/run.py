# Odometry benchmark, see run.py -h for help

import subprocess
import pathlib
import os
import tempfile

from benchmark.benchmark import benchmark, getArgParser
from postprocessing import postprocessMergeSlamAndVioOutput, convertSlamOutput

def runBenchmark(args):
    # Override
    args.setDir = "benchmark/sets"
    args.output = "target/benchmark-results"

    binary = "./HybVIO/target/main"
    subprocess.run(["cd", "target", "&&", "make", "-j4"], shell=True)

    def setupFn(args, outputDir):
        pass

    def tearDownFn(args, outputDir):
        pass

    def vioTrackingFn(args, benchmark, outputDir, outputFile, slamMap):
        mainFlags = "-displayVideo=false -v={} -vocabularyPath=HybVIO/data/orb_vocab.dbow2".format(args.logLevel)
        if args.params: mainFlags += " " + args.params
        allFlags = mainFlags if not benchmark.params else mainFlags + " " + benchmark.params
        logFile = "{}/{}/{}.txt".format(outputDir, "logs", benchmark.name)

        if args.postprocessWithInterpolation:
            tmpfile = tempfile.NamedTemporaryFile(delete=False, suffix='.jsonl').name
            vioOutFile = tmpfile
        else:
            vioOutFile = outputFile

        cmd = (binary
            + " -i=" + benchmark.dir
            + " -o=" + vioOutFile
            + " -slamMapPosesPath=" + slamMap
            + " " + allFlags
            + " > " + logFile + " 2>&1")
        task = subprocess.Popen(cmd, shell=True, encoding="utf-8")
        if task.wait() != 0 or not pathlib.Path(outputFile).exists():
            print("---\nFailed running {}, printing VIO logs `{}`:".format(benchmark.dir, logFile))
            with open(logFile, "r") as f:
                for line in f: print("    " + line.strip())
            print("---")
            return False

        if args.postprocessWithInterpolation:
            postprocessMergeSlamAndVioOutput(vioOutFile, slamMap, outputFile)
            os.remove(tmpfile)
        elif args.postprocess:
            convertSlamOutput(slamMap, outputFile)

    benchmark(args, vioTrackingFn, setupFn, tearDownFn)

    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    parser = getArgParser()
    parser.add_argument("-logLevel", help="Log level for algorithm (-2,-1,...,9)", default=0)
    parser.add_argument('--postprocess', action='store_true')
    parser.add_argument('--postprocessWithInterpolation', action='store_true')
    args = parser.parse_args()

    runBenchmark(args)
