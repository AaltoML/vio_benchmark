# vio_benchmark

Tools for benchmarking different Visual-Inertial Odometry solutions

These are best used in together with other tools supporting same JSONL format defined in [json-recorder](https://github.com/AaltoVision/jsonl-recorder):
* [u-blox C099-F9P data collection app](https://github.com/AaltoML/u-blox-capture)
* [Realsense data collection app](https://github.com/AaltoVision/realsense-capture)
* [Android data collection app](https://github.com/AaltoML/android-viotester)


# benchmark.py

## Usage

`benchmark.py` is a library for running multiple parallel benchmarks. It's not a stand alone script. See `benchmark/example_benchmark.py` on how to use it.

## Running benchmarks sets

To run benchmark sets, use the option `-set name_of_the_set`. The benchmark sets are define in json files in current directory or in given `-setDir <folder>`.

```
python benchmark/example_benchmark.py -set example
```

Following example benchmark set will run each dataset in "benchmarks" array once for every entry in "parameterSets", meaning you will get 6 x 2 = 12 benchmarks ran in total. You can use same dataset with different parameters as seen with the last 4 datasets. "parameterSets" is optional. All "params" flags are appended. If you run a benchmark with flags `-set example -params "-something=true"`, the full arguments for dataset euroc-mh-01-vR-01 and first parameter set would be `-something=true -customParam1=0.0 -customParam2=0.01`.

```
{
    "benchmarks": [
        {"folder": "advio-01"},
        {"folder": "advio-02"},
        {"folder": "euroc-mh-01-easy", "params": "-customParam1=0.01", "name": "euroc-mh-01-vR-01"},
        {"folder": "euroc-mh-01-easy", "params": "-customParam1=0.05", "name": "euroc-mh-01-vR-05"},
        {"folder": "euroc-mh-02-easy", "params": "-customParam1=0.01", "name": "euroc-mh-02-vR-01"},
        {"folder": "euroc-mh-02-easy", "params": "-customParam1=0.05", "name": "euroc-mh-02-vR-05"}
    ],
    "parameterSets": [
        {"params": "-customParam2=0.0", "name": "filterOff"},
        {"params": "-customParam2=0.1", "name": "filterOn"}
    ]
}
```
