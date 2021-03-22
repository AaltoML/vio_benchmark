# Simple CLI to Basalt

Simple CLI tool to run [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) with standard JSONL formats presented in this repo.

## Build

You don't need to build ORB_SLAM3 separately, it's included in the step 3.

1. Install dependencies: `pangolin`, `opencv@3`, `eigen`, and `boost`.
2. Build included dependencies: `./build_dependencies.sh`
3. Build with `mkdir target && cd target && cmake .. && make -j8`.

## ORB_SLAM3 Runner

TODO

## License

Some files in this folder are copied/modified from ORB_SLAM3 and are licensed under GNU GENERAL PUBLIC LICENSE 3, see `ORB_SLAM3/LICENSE`.
