# Odometry benchmark, see run.py -h for help

import subprocess
import os
import socket
import re
import tempfile
from benchmark.benchmark import benchmark, getArgParser
from postprocessing import postprocessMergeSlamAndVioOutput, convertSlamOutput

def runBenchmark(args):
    # Override
    args.setDir = "benchmark/sets"
    args.output = "target/benchmark-results"

    binary = ["./HybVIO/target/main"]
    runner = ""
    if args.parallelBenchmark:
        # `srun` here allows tracking resource usage of the following command on
        # a cluster. The `--input none` is needed inside a loop that uses `read`,
        # see eg <https://stackoverflow.com/a/13800476>.
        runner="srun --input none "
    else:
        subprocess.run(["cd", "target", "&&", "make", "-j4"], shell=True)

    def setupFn(args, outputDir):
        if args.copyBinary:
            subprocess.run(["cp", "-n", "./HybVIO/target/main", outputDir])
            binary[0] = "./{}/main".format(outputDir)
        if hasattr(args, 'memoryLimit') and args.memoryLimit:
            # Using --user here to avoid giving password might, without errors, not actually apply limits
            binary[0] = "systemd-run --scope -p MemoryLimit={}M {}".format(args.memoryLimit, binary[0])

    def tearDownFn(args, outputDir):
        if args.copyBinary:
            os.remove(binary[0])

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

        subprocess.check_call(runner + binary[0]
                + " -i=" + benchmark.dir
                + " -o=" + vioOutFile
                + " -slamMapPosesPath=" + slamMap
                + " " + allFlags
                + " > " + logFile + " 2>&1", shell=True)

        if args.postprocessWithInterpolation:
            postprocessMergeSlamAndVioOutput(vioOutFile, slamMap, outputFile)
            os.remove(tmpfile)
        elif args.postprocess:
            convertSlamOutput(slamMap, outputFile)

    benchmark(args, vioTrackingFn, setupFn, tearDownFn)

    print("Everything ran successfully! Bye!")


if __name__ == "__main__":
    parser = getArgParser()
    parser.add_argument("-copyBinary", help="Copies binary before using it, copy is removed after benchmark ends", action="store_true")
    parser.add_argument("-logLevel", help="Log level for algorithm (-2,-1,...,9)", default=0)
    parser.add_argument('--postprocess', action='store_true')
    parser.add_argument('--postprocessWithInterpolation', action='store_true')
    parser.add_argument('-memoryLimit', help="Memory limit in megabytes", default=None)
    args = parser.parse_args()

    runBenchmark(args)
