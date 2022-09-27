# Simple CLI to ORB-SLAM3

Simple CLI tool to run [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) with standard JSONL formats presented in this repo.

ORB-SLAM3 version used is a fork that has added Mac support and binary ORB Vocabulary format support for improved startup times, neither impacts the algorithm itself.

## Build

You don't need to build ORB_SLAM3 separately, it's included in the step 3.

1. Install dependencies using your favorite package manager: `pangolin opencv@3 eigen boost`.
2. Build included dependencies: `./build_dependencies.sh`
3. Build with `mkdir target && cd target && cmake .. && make -j8`.

## ORB_SLAM3 Runner

First run will create a binary version of ORB vocabulary for MUCH faster loading.

Run the benchmark, for example, asuming your `EuroC` data is in `./data` directory in converted format:

```
python run_benchmark.py euroc-mh-01-easy \
	-dir ./data \
	-vocab ./ORB_SLAM3/Vocabulary \
	-config ./ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
	-metricSet full_3d_align \
	-compare groundTruth,slam \
	-threads 1
```

To run full EuroC set:

```
python run_benchmark.py -set euroc \
	-dir ./data \
	-setDir ../benchmark/sets \
	-vocab ./ORB_SLAM3/Vocabulary \
	-config ./ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
	-metricSet full_3d_align \
	-compare groundTruth,slam \
	-exportAlignedData \
	-threads 1
```

## Running in Docker

Building the version of ORB_SLAM3 used in the HybVIO paper can be challenging due to required old library versions. A `Dockerfile` is included in this directory to help in the task. Rough usage guide follows.

Build the image (once):
```
cd vio_benchmark/orbslam3_runner
docker build -t orbslam3 .
```

Run an example benchmark:
```
cd vio_benchmark
mkdir output
docker run -v $(pwd)/data:/data -v $(pwd)/output:/vio_benchmark/orbslam3_runner/output -it orbslam3 bash docker_orbslam_run.sh
```

Finally, browse the output files and figures in the local `output/` directory.

## License

Some files in this folder are copied/modified from ORB_SLAM3 and are licensed under GNU GENERAL PUBLIC LICENSE 3, see `ORB_SLAM3/LICENSE`.
