# HybVIO runner

This module can be used to run benchmarks and reproduce the numbers published in [HybVIO: Pushing the Limits of Real-time Visual-inertial Odometry](https://arxiv.org/abs/2106.11857).

## Building HybVIO

See also the `README` in the `HybVIO` repository (submodule in this directory). This has been tested only on Linux.

1. Clone the `HybVIO` subdirectory of this repository with the `--recursive` option (this will take a while)

2. Install dependencies, including `CMake`, `glfw` and `ffmpeg`. Some lists:

* **Arch Linux:** clang, cmake, ffmpeg, glfw, gtk3
* **Debian Stretch:** clang, libc++-dev, libgtk2.0-dev, libgstreamer1.0-dev, libvtk6-dev, libavresample-dev.
* **Raspberry Pi/Raspbian:** (Pi 4, 8 GiB): libglfw3-dev and libglfw3 (for accelerated arrays) and libglew-dev and libxkbcommon-dev (for pangolin, still had problems). Also started off with the Debian setup above.
* **Ubuntu 21.04**: Debian setup plus: libavcodec-dev libswscale-dev libavformat-dev (OpenCV ffmpeg support)

3. Build rest of the dependencies:
``` bash
cd HybVIO/3rdparty/mobile-cv-suite && ./scripts/build.sh
```

4. In order to be able to use the SLAM module you must download the ORB vocabulary:
``` bash
cd HybVIO
./src/slam/download_orb_vocab.sh
```

5. Then, to build the main and test binaries, perform the standard CMake routine:
``` bash
cd HybVIO
mkdir target && cd target
cmake -DBUILD_VISUALIZATIONS=ON -DUSE_SLAM=ON ..
# or if not using clang by default:
# CC=clang CXX=clang++ cmake [...]
make -j6
```

6. Now the `target` folder should contain the binaries `main` and `run-tests`.

## Benchmarking EuRoC

To run benchmarks on EuRoC dataset and reproduce numbers in the paper https://arxiv.org/abs/2106.11857:

* Complete the building section above
* To download the dataset and convert it to correct format, from the root of this repository, run (this will take a while and require about 40GB of disk space):
``` bash
python convert/euroc_to_benchmark.py
# Optionally, remove the large temporary files no longer needed:
rm -r data/raw/euroc
```

* Finally, reproduce the paper benchmarks with `compute_paper_results.py` in `hybvio_runner` dir (`cd hybvio_runner`). This will take a while if you run all configuration variations. You can modify the script to omit benchmarks you don't need. Scripts assume first EuRoC benchmark data file can be found at `hybvio_runner/../data/benchmark/euroc-mh-01-easy/data.jsonl` etc., that is, the `data/` folder/symlink should be at the repository root.
``` bash
cd hybvio_runner
python compute_paper_results.py euroc
```

* Alternatively you can just run the EuRoC once with configuration of your choosing with `compare_to_others.py`, use `-h` flag to see the available options:
``` bash
python compare_to_others.py euroc
```

* Results for benchmarks are placed in `./target/benchmark-results/` and `./target/results/` folders with current time as folder name.

## Benchmarking TUM VI and SenseTime

Like with EuRoC above, however the conversion scripts `tum_vi_to_benchmark.py` and `sensetime_to_benchmark.py` include manual download steps. See the scripts for details.

Benchmarking the SenseTime data further requires building the dataset authors' own benchmark tool [eval-vislam](https://github.com/zju3dv/eval-vislam), and pointing to the directory with built binaries using `--vislamEvalBin`.

``` bash
cd hybvio_runner
python compute_paper_results.py tum
python compute_paper_results.py --vislamEvalBin path/to/eval-vislam/bin sensetime
```

## Reproducing ORB_SLAM3 results

See `orbslam3_runner/README.md` in this repository. Using the runner on (EuRoC) data will produce an output folder with images of the position tracks, and a subfolder with the track coordinates in JSONL format.
