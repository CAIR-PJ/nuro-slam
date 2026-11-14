# Building Nuro-SLAM

## Scope

This document covers the experimental native integration of
CMax, EROAM, EMBA and EPBA on macOS Apple Silicon.

Compilation and test success do not establish SLAM accuracy
or numerical equivalence with upstream implementations.

## Development Environment

- macOS ARM64 / Apple Silicon
- ROS 2 Jazzy via Homebrew
- AppleClang with C++17 support
- CMake and Ninja
- Python 3.11
- Eigen and OpenCV
- Algorithm-specific dependencies

Activate the locally configured ROS 2 environment before
running ROS-dependent builds.

## Algorithm Locations

| Algorithm | Source directory |
|---|---|
| CMax | `algorithms/single-robot/event-based/cmax/` |
| EROAM | `algorithms/single-robot/event-based/eroam/` |
| EMBA | `algorithms/single-robot/event-based/emba/` |
| EPBA | `algorithms/single-robot/event-based/epba/` |

## Isolated Build Procedure

Build each algorithm in a separate directory outside the
repository. The general CMake sequence is:

1. Configure using `cmake -S <source> -B <build> -G Ninja`.
2. Compile using `cmake --build <build>`.
3. Run registered tests using
   `ctest --test-dir <build> --output-on-failure`.

Algorithm-specific CMake options and dependencies may differ.
Consult the relevant CMakeLists.txt and algorithm README.

## Verified Integration Results

| Algorithm | Current verification |
|---|---|
| CMax | Native build and offline event processing completed |
| EROAM | Native build and 50,000-event execution completed |
| EMBA | Native build and CTest execution passed |
| EPBA | Native build and 4/4 CTest checks passed |

EROAM currently registers no CTest tests.

The bounded EROAM run generated a TUM trajectory but did
not generate a panorama because the calculated loop width
was non-positive.

The integrated EPBA package remains an input-validation
stage; complete optimizer verification is pending.

## Reproducibility Requirements

Record the Git revision, compiler and dependency versions,
build configuration, dataset checksums, camera calibration,
algorithm parameters, logs, output checksums and metrics.

## Limitations

Ground-truth comparison, numerical parity, extended dataset
testing, profiling and multi-robot integration remain pending.

The original research workspace must remain unchanged.
