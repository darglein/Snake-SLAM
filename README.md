# Snake-SLAM

*Abstract.*
Snake-SLAM is a scalable visual inertial SLAM system for autonomous navigation in low-power aerial devices.
The tracking front-end features map reuse, loop closing, relocalization, and supports monocular, stereo, and RGBD input.
The keyframes are reduced by a graph-based simplification approach and further refined using a novel deferred mapping stage to ensure a sparse yet accurate global map.
The optimization back-end decouples IMU state estimation from visual bundle adjustment and solves them separately in two simplified sub problems.
This greatly reduces computational complexity and allows Snake-SLAM to use a larger local window size than existing SLAM methods.
Our system implements a novel multi-stage VI initialization scheme, which uses gyroscope data to detect visual outliers and recovers metric velocity, gravity, and scale.
We evaluate Snake-SLAM on the EuRoC dataset and show that it outperforms all other approaches in efficiency while also achieving state-of-the-art tracking accuracy.

## Prerequisites 
Download and install the following libraries before continuing with the build instructions.
 * [CMake 3.18+](https://github.com/Kitware/CMake)
 * [OpenCV 4](https://github.com/opencv/opencv)
 * [CUDA 11.1+ (optional)](https://developer.nvidia.com/cuda-downloads)
 
## Build Instructions (Ubuntu 20.04, Cuda 11.1)

```bash
cd Snake-SLAM
git submodule update --init --recursive

export CXX=clang++-10
export CUDAHOSTCXX=g++-9

mkdir build
cd build
cmake ..
make -j8
```

## Run Snake-SLAM on the EuRoC dataset

 * Open the file configs/euroc.ini
 * Update the line [Dataset] -> dir=XX 
 * For example: `dir = /ssd2/slam/euroc/MH_01/mav0`
 * Run Snake-SLAM with

```bash
cd Snake-SLAM
./build/bin/snake_slam configs/euroc.ini
```

## Notes on Efficiency

At the time of release, Snake-SLAM is the most efficient VI-SLAM system.
This can be validated by adjusting the `Dataset-playback_fps` in the config file.

By default we have set `async = false` in the config file and which disables all non-deterministic parallelism.
Maximum performance will be only be achieved by setting `async = true`. However this can reduce tracking accuracy, if
the `playback_fps` is unreasonably high. Our recommendation therefore is:

**Debugging, Testing, Development**
```ini
async = false
playback_fps = 200
```
**Deployment, Real-world Usage**
```ini
async = true
playback_fps = 30
```

## Additional Information

 * All trajectories used in the paper are included here in `euroc_mono_25_runs_each.zip` and `euroc_stereo_25_runs_each.zip`
 * Feel free to use these trajectories for further analysis or recompute them using the config file `configs/euroc.ini`
 * The Snake-SLAM source code is released under the **MIT License**. You are allowed to use the code in commercial or non-commercial projects, however a reference to this repository and the respective paper (see below) must be made. 

## Publication

Rückert, Darius, and Marc Stamminger. "Snake-SLAM: Efficient Global Visual Inertial SLAM using Decoupled Nonlinear Optimization." Proceedings of the 2021 International Conference on Unmanned Aircraft Systems (ICUAS), Athen 2021.

https://ieeexplore.ieee.org/abstract/document/9476760

```
@INPROCEEDINGS{9476760,
  author={Rückert, Darius and Stamminger, Marc},
  booktitle={2021 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Snake-SLAM: Efficient Global Visual Inertial SLAM using Decoupled Nonlinear Optimization}, 
  year={2021},
  volume={},
  number={},
  pages={219-228},
  doi={10.1109/ICUAS51884.2021.9476760}}
```
