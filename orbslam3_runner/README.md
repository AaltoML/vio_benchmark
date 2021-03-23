# Simple CLI to Basalt

Simple CLI tool to run [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) with standard JSONL formats presented in this repo.

## Build

You don't need to build ORB_SLAM3 separately, it's included in the step 3.

1. Install dependencies using your favorite package manager: `pangolin opencv@3 eigen boost`.
2. Build included dependencies: `./build_dependencies.sh`
3. Build with `mkdir target && cd target && cmake .. && make -j8`.

## ORB_SLAM3 Runner

First run will create a binary version of ORB vocabulary for MUCH faster loading.

Run the benchmark, for example, asuming your `EuroC` data is in `~/euroc_benchmarks` directory in converted format:

```
	python run_benchmark.py euroc-mh-01-easy \
		-dir ~/euroc_benchmarks/ \
		-vocab ./ORB_SLAM3/Vocabulary \
		-config ./ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
		-metricSet full_3d_align \
		-thread 1
```

## License

Some files in this folder are copied/modified from ORB_SLAM3 and are licensed under GNU GENERAL PUBLIC LICENSE 3, see `ORB_SLAM3/LICENSE`.
