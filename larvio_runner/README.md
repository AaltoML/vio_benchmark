# Simple CLI to LARVIO

Simple CLI tool to run [LARVIO](https://github.com/PetWorm/LARVIO) with standard JSONL formats presetnted in this repo.

## Build

You don't need to build LARVIO separately, it's included in the step 2.

1. Install dependencies using your favorite package manager: `Eigen`, `Boost`, `Suitesparse`, `Ceres`, `Pangolin` and `OpenCV`(v4)
2. Build with `mkdir target && cd target && cmake .. && make -j8`.

## LARVIO Runner

Run the benchmark, for example, asuming your `EuroC` data is in `~/euroc_benchmarks` directory in converted format:

```
	python run_benchmark.py euroc-mh-01-easy \
		-dir ~/euroc_benchmarks/ \
		-config ./LARVIO/config/euroc.yaml \
		-metricSet full_3d_align \
		-compare groundTruth \
		-thread 1
```

## License

Some files in this folder are copied/modified from LARVIO and are licensed under various licenses, see `LARVIO/licenses` folder.
