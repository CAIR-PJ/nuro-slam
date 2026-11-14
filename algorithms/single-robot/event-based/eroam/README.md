# EROAM → ROS 2 Jazzy (Apple Silicon ARM64) — source migration candidate

This is an **uncompiled, unexecuted source-level port** based on the uploaded original EROAM archive. It retains the original ES-ICP optimization, spherical event processing, RDM and ikd-Tree implementation in `src/eroam_ros2/src/eroam.hpp` and `thirdparty/`. It has not been validated against the original implementation or on the target Mac. **Do not use results as scientific benchmarks before validation.**

## Layout

`src/eroam_ros2/` is a separate Ament CMake package. `src/eroam_ros2/src/eroam.hpp` is a minimally adapted original algorithm. `include/eroam_port/ros2_adapter.hpp` maps original ROS 1 calls to ROS 2 rclcpp publishers/parameters and implements the `dv_ros_msgs` event structure locally for offline processing. `src/eroam_run_ros2.cpp` reads `CMAXEV1` events and runs the original processing function. The original algorithm destructor saves its TUM trajectory and panoramic PNG/JPEG on normal EOF. No ROS 1 runtime or dv_ros_msgs installation is required for offline mode.

**Limitations:** Only offline event input is implemented. ROS 2 live EventArray subscription is not implemented, and no ROS 2 custom EventArray message is generated. Original pose/path publishers are created but the original algorithm does not publish pose/path messages in its processing function; trajectory is written to TUM. Point cloud outputs are published through ROS 2 native PointCloud2 serialization. Map output is the original panorama; a PCD export is not implemented. OpenCV 5 uses a local iterative Brown–Conrady undistortion implementation instead of `cv::undistortPoints`, requiring numerical parity validation.

## Mac build (does not modify existing installations)

Copy this directory into `~/neuromorphic_slam_ws/ports/eroam_ros2_port`, then:

```zsh
cd "$HOME/neuromorphic_slam_ws/ports/eroam_ros2_port"
set +u
source "$HOME/robotics2/robotics_env.zsh"
set +u
bash src/eroam_ros2/tools/build_macos.sh
source install/setup.zsh
ros2 pkg executables eroam_ros2
```

Homebrew dependencies: `pcl`, `eigen`, `opencv`, `tbb`, `glog`, `gflags`, `yaml-cpp`; ROS 2 packages `rclcpp`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `ament_cmake`. This package does **not** install any of these automatically. If compilation fails, preserve the first compiler error and fix source compatibility rather than changing the working ROS 2 installation.

## Offline Playroom

The CMax port's converter previously produced `converted/playroom.events` in CMAXEV1 format. Verify the file with:

```zsh
python3 src/eroam_ros2/tools/check_stream.py "$HOME/neuromorphic_slam_ws/ports/cmax_ros2_port/converted/playroom.events"
```

**Camera calibration must match the actual dataset.** The included launch uses *illustrative* 128×128 intrinsics (`fx=fy=90, cx=cy=64, zero distortion`), which are **not verified for Playroom**. Replace the values in `src/eroam_ros2/launch/offline.launch.py` with those from the original Playroom calibration before scientific runs. The original EROAM launch uses 240×180 for a different dataset; do not reuse it for Playroom.

```zsh
mkdir -p results/playroom_run01
ros2 launch eroam_ros2 offline.launch.py \
  events:="$HOME/neuromorphic_slam_ws/ports/cmax_ros2_port/converted/playroom.events" \
  results:="$PWD/results/playroom_run01" \
  packet_size:=500
```

`packet_size` defines how many events are passed to EROAM per `EventArray`. Original EROAM has frame-level thresholds and samples from each packet; changing packet size changes results. To compare fairly, preserve packet boundaries from original `dv_ros_msgs` or define and report a consistent packetization policy. **The CMAXEV1 stream discards original packet boundaries**, so exact equivalence to the original ROS bag processing is not guaranteed.

## Test status

- Uploaded original archive inspected and copied: **yes**.
- Source-level migration generated: **yes**.
- ROS 2 Jazzy/Apple Silicon compilation: **not tested (no ROS 2 toolchain here)**.
- Offline Playroom run: **not tested**.
- Numerical parity (ES-ICP, undistortion, trajectories): **not tested**.
- Pose/path live publication: **not implemented by original processing loop**.

## Original source provenance

Original repository: https://github.com/wlxing1901/eroam ; local checkout commit reported by user: `01d48b0`. The ZIP did not contain `.git`, so the source tree's exact commit cannot be cryptographically verified from the archive alone. Keep upstream license and attribution intact; inspect upstream licensing before redistribution.

## Nuro-SLAM Integration Validation

**Environment:** macOS Apple Silicon ARM64, native ROS 2 Jazzy.

- Native compilation: completed.
- Offline execution: completed using 50,000 Playroom events.
- Packet size: 500 events.
- Packets processed: 100.
- Spherical ICP processing: executed.
- TUM trajectory: generated.
- Panorama: not generated in the bounded test.
- Numerical parity and ground-truth accuracy: not established.

The panorama writer reported a non-positive loop width.
The cause remains under investigation.

The earlier source-migration status above reflects the
original imported documentation and is superseded by this
integration-validation section.
