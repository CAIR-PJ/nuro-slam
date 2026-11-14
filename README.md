# Nuro-SLAM

**Neuromorphic SLAM Research Platform**

Nuro-SLAM is a research-oriented repository for event-based
simultaneous localization and mapping (SLAM), neuromorphic
computing, reproducible benchmarking, and collaborative
multi-robot mapping.

The repository integrates existing event-based SLAM
implementations alongside infrastructure for future
neuromorphic and multi-robot research.

## Integrated Algorithms

| Algorithm | Description | Current Validation |
|---|---|---|
| CMax | Contrast-maximization-based event-camera SLAM | Native build and offline execution completed |
| EROAM | Event-based rotational odometry and mapping | Native build and bounded offline execution completed |
| EMBA | Event-based mosaicing bundle adjustment | Native build and CTest execution completed |
| EPBA | Event-based photometric bundle adjustment | Native build and 4/4 CTest checks passed |

Source implementations are located under:

`algorithms/single-robot/event-based/`

Each algorithm retains its own documentation and applicable
upstream notices.

## Development Environment

The initial native integration and validation environment is:

- macOS on Apple Silicon (ARM64)
- ROS 2 Jazzy through Homebrew
- AppleClang and C++17
- CMake and Ninja
- Python 3.11
- OpenCV and Eigen

Additional platforms are research targets, not yet validated
by the results summarized here.

## Current Validation Status

### CMax

Native compilation and offline processing have been completed
using the converted Playroom event dataset.

### EROAM

The native executable successfully processed a bounded sample
of 50,000 events in 100 packets.

A TUM-format trajectory file was generated.

Panorama generation was skipped during this short run because
the calculated loop width was non-positive. This requires
further investigation.

### EMBA

The native C++ implementation and ROS 2 wrapper compiled
successfully. CTest execution completed successfully.

### EPBA

Native compilation completed successfully. Four CTest checks
passed, covering CLI availability, timing, camera functionality,
and core warping functionality.

## Research Validation Limitations

Successful compilation or execution does not independently
establish SLAM accuracy.

The following remain open research-validation activities:

- Ground-truth trajectory comparison
- Numerical parity against original implementations
- Cross-algorithm benchmarking under controlled conditions
- Runtime, memory, and energy profiling
- Multi-robot and neuromorphic hardware integration
- Extended dataset and mapping validation

Results should distinguish implementation verification from
algorithmic accuracy and scientific reproducibility.

## Repository Structure

- `algorithms/` — SLAM algorithm implementations and ports
- `slam/` — SLAM-specific documentation and organization
- `core/` — Common research component areas
- `neuromorphic/` — Neuromorphic research modules
- `multi-robot/` — Collaborative SLAM research
- `datasets/` — Dataset documentation and tooling
- `benchmarks/` — Benchmark definitions and evaluation
- `experiments/` — Experimental configurations and reports
- `platforms/` — Platform-specific development information
- `docs/` — Architecture, research and project documentation

Some directories represent planned research modules rather
than completed software implementations.

## Data and Generated Artifacts

Large raw datasets, event streams, ROS bags, build directories,
runtime logs, and temporary release-audit outputs are excluded
from Git through `.gitignore`.

The original research workspace is maintained separately from
this integration repository.

## Development Status

**Experimental research software — active development.**

The current milestone is the native integration of CMax,
EROAM, EMBA, and EPBA on Apple Silicon.

Future milestones include reproducible evaluation, common
algorithm interfaces, neuromorphic acceleration, and
collaborative multi-robot SLAM.

## Attribution

The integrated algorithms originate from independent research
projects. Their original authorship, citations, and applicable
licenses should be consulted in their respective directories.

Nuro-SLAM integration does not imply authorship of the
original algorithms.
