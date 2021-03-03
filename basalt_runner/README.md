# Simple CLI to Basalt

Simple CLI tool to run [Basalt](https://vision.in.tum.de/research/vslam/basalt) with standard JSONL formats presetnted in this repo.

## How to run?

1. Go to `basalt-mirror` and build it as instructed
2. Then build the runner `mkdir target && cd target && cmake .. && make j8`
3. Finally run the benchmark, for example `python run_benchmark.py euroc-mh-01-easy -dir ~/euroc_benchmarks/ -calib ./basalt-mirror/data/euroc_ds_calib.json -config ./basalt-mirror/data/euroc_config.json`, asuming your `EuroC` data is in `~/euroc_benchmarks` directory.

## License

Some files in this folder are copied/modified from Basalt and are licensed under BSD 3-Clause License, see `basalt-mirror/LICENSE`.
