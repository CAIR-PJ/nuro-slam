# CMax-SLAM: experimental native ROS 2 Jazzy source port

**Status: source port supplied; full ROS 2 build and Apple Silicon execution are unverified.** This workspace contains the actual upstream numerical implementation, not a substitute SLAM algorithm. It is not an established upstream ROS 2 release and has not reproduced a published result yet.

Target: macOS ARM64, Homebrew Jazzy at `/opt/homebrew/opt/ros2-jazzy`, RViz2. No ROS 1 runtime, Docker, VM, or Ubuntu is used. Gazebo Harmonic 8.14 can remain installed; recorded ECRot replay does not use Gazebo or generate events from Gazebo images.

## Contents

- `src/cmax_slam_ros2`: complete ament_cmake package, generated Event/EventArray messages, C++ frontend/backend, native ROS 2 node, launch files, RViz configuration.
- `tools/convert_ecrot.py`: pure Python ROS bag V2 reader using `rosbags`; no `rosbag`, `rospy`, or ROS 1 installation required.
- `tools/build_macos.sh`: read-only preflight followed by isolated colcon build.
- `tools/validate_outputs.py`: checks trajectory/map structure after a run.
- `docs/UPSTREAM_TO_ROS2.patch`: exact source changes relative to upstream.
- `docs/SOURCE_MANIFEST.json`: upstream and port source hashes, third-party preservation audit.
- `docs/ecrot_calibration_original`: calibration files copied unchanged from ECRot, with provenance in `docs/PROVENANCE.md`.
- `tests`: runnable converter and timestamp arithmetic tests.

## 1. Extract beside your existing experiment

Extract this archive into a **new** directory, for example `~/neuromorphic_slam_ws/ports/cmax_ros2_port`. Do not overwrite your original `software/cmax_slam` checkout and do not move the ROS 1 packages into this workspace's `src` directory.

Run the following commands from the extracted `cmax_ros2_port` directory. Build and dataset preparation use different Python environments deliberately: the converter only needs pure Python libraries; the ROS build must use the interpreter matching your installed ROS 2 extensions.

## 2. Prepare the offline converter

```bash
python3 -m venv .venv-converter
.venv-converter/bin/python -m pip install -r tools/requirements.txt
.venv-converter/bin/python -m unittest discover -s tests -v
```

This installs converter dependencies only into the new virtual environment. It does not change Homebrew or ROS Python packages.

Inspect both recordings before conversion:

```bash
.venv-converter/bin/python tools/convert_ecrot.py \
  ~/neuromorphic_slam_ws/experiments/ecrot_comparison/datasets/Playroom_events.bag --inspect

.venv-converter/bin/python tools/convert_ecrot.py \
  ~/neuromorphic_slam_ws/experiments/ecrot_comparison/datasets/Victory_Column_events.bag --inspect
```

The converter selects the EventArray topic only when exactly one topic is unambiguous; otherwise provide `--topic /actual/topic`. It reads custom message definitions embedded in the bag. It checks every selected event's coordinates, timestamps, dimensions and ordering. It refuses to silently sort or thin events. It reads uncompressed, BZ2 and LZ4 chunks through `rosbags`; the included integration tests currently cover uncompressed fixtures only.

Convert Playroom using its DVS128 calibration:

```bash
.venv-converter/bin/python tools/convert_ecrot.py \
  ~/neuromorphic_slam_ws/experiments/ecrot_comparison/datasets/Playroom_events.bag \
  --calibration docs/ecrot_calibration_original/synth_data/DVS128-synthetic.yaml \
  --repair-projection-from-k \
  --output converted/playroom.events
```

**Why the explicit repair?** The ECRot DVS128 calibration supplied in the repository has projection last row `[0, 0, 0, 1]`, rather than the valid pinhole row `[0, 0, 1, 0]`. This option constructs `P = [K | 0]` for this zero-baseline monocular camera, preserves the original calibration file, and records the modification in the conversion manifest. This is a documented calibration adjustment, not a verified reproduction of the publisher's exact processing pipeline. Remove the option when using an independently verified corrected calibration.

Convert Victory Column using the motorized-mount DVX calibration:

```bash
.venv-converter/bin/python tools/convert_ecrot.py \
  ~/neuromorphic_slam_ws/experiments/ecrot_comparison/datasets/Victory_Column_events.bag \
  --calibration docs/ecrot_calibration_original/real_data/DXA-motorized_mount.yaml \
  --output converted/victory.events
```

If the bag dimensions disagree with the selected calibration, conversion stops. Investigate the exact dataset variant; do not force the dimensions or invent intrinsics. A single unambiguous full-resolution CameraInfo topic can also be used by omitting `--calibration`.

Each conversion produces:

| File | Meaning |
| --- | --- |
| `*.events` | 24-byte header plus packed 13-byte events: x/y uint16, timestamp int64 ns, polarity uint8; little endian |
| `*.events.camera.yaml` | ROS 2 camera parameters derived from the actual calibration |
| `*.events.json` | Source bag/calibration hashes, topic inventory, event count and exact time bounds |

Event timestamps remain absolute, integer nanoseconds. Bag arrival timestamps do not replace event timestamps. IMU and ground-truth topics are inventoried but not converted: CMax-SLAM uses event-only rotational estimation. Keep the original bag for later evaluation.

## 3. Build with the existing Homebrew ROS 2

Use a fresh terminal with your **working native Jazzy environment sourced**, then:

```bash
cd ~/neuromorphic_slam_ws/ports/cmax_ros2_port
bash tools/build_macos.sh
source install/setup.zsh
```

The script attempts `/opt/homebrew/opt/ros2-jazzy/setup.bash` or `local_setup.bash`; if your distribution uses another setup location, source your known working environment first. It does not install or upgrade anything.

The preflight checks `rclpy`, `ament_package`, `colcon_core`, `em`, `lark`, `numpy`, ROS 2 packages, and native tools from one Python interpreter. Your previously reported `python3`/colcon at `/Library/Frameworks/Python.framework/Versions/3.11` may differ from the interpreter used by Homebrew ROS. If imports fail, select the already-installed matching interpreter:

```bash
ROS_PYTHON=/absolute/path/to/the/ROS-compatible/python3 bash tools/build_macos.sh
```

Do not select the converter virtual environment for the ROS build. A successful converter install does not establish ROS Python ABI compatibility.

### Native library requirements

| Library | Handling in this port |
| --- | --- |
| OpenCV 5 | Selected by CMake from the existing Homebrew prefix; source compatibility unverified |
| Eigen 5 | Uses `Eigen3::Eigen` CMake target from the installed Eigen package; compatibility with bundled older Sophus/Basalt unverified |
| GSL | **Required**: original frontend/backend optimizers use GSL BFGS routines |
| glog | Required for original logging/check macros |
| Ceres 2.2 | Not linked: the active original implementation uses GSL |
| Boost, PCL, gflags | Not needed by this port |
| Basalt/Sophus/cereal | Bundled upstream headers retained with their licenses |
| cv_bridge/image_geometry | Native ROS 2 packages; must use the **same OpenCV ABI** as this executable |

The preflight writes `preflight.json` and inspects `cv_bridge`/`image_geometry` with `otool`. It rejects dependencies on OpenCV dylib names absent from the selected Homebrew OpenCV. This catches common OpenCV 4/5 mismatches; it does not prove full ABI compatibility. Do not mix libraries or force linker flags to bypass it. A mismatched vision_opencv overlay needs a separate build against the same OpenCV before this package can be validated.

If a dependency is missing, preflight stops. It never reinstalls ROS or changes your existing libraries. If CMake cannot find the current OpenCV or Eigen config, supply their existing configuration directories as extra `-DOpenCV_DIR=...` / `-DEigen3_DIR=...` arguments to the build script. Do not switch library versions silently.

No `-fopenmp`, `-march=native`, x86 flags, Catkin, `roscpp`, `rosbag`, or ROS 1 headers are used. Upstream has no OpenMP pragmas in the compiled algorithm files.

## 4. Run Playroom

From the built workspace:

```bash
source install/setup.zsh
ros2 launch cmax_slam_ros2 offline.launch.py \
  events:="$PWD/converted/playroom.events" \
  profile:=synthetic \
  results:="$PWD/results/playroom_run01"
```

In another terminal, source the same Jazzy environment and this workspace, then:

```bash
ros2 topic echo /cmax/status --once --qos-durability transient_local
```

Wait for `eof=true` and a positive `optimized_windows` count. The application stays alive so late RViz subscribers can see the last map and orientation. **Press Ctrl-C in the launch terminal after EOF to save the final map.** Allow graceful shutdown; force-killing the process can prevent final map saving. Use a fresh results directory for each run.

Victory Column:

```bash
ros2 launch cmax_slam_ros2 offline.launch.py \
  events:="$PWD/converted/victory.events" \
  profile:=motorized_mount \
  results:="$PWD/results/victory_run01"
```

Profiles copy upstream `ecrot_synth.launch` / `ecrot_mount.launch` numerical parameters. They use relative ROS 2 input topics and disable the optional FOV drawing overlay. The synthetic upstream launch demonstrates Bicycle rather than a documented Playroom tuning; the supplied profile is an initial execution configuration, **not a claim of published Playroom settings**. Parameter changes for a comparison must be recorded.

RViz starts automatically. `rviz:=false` runs processing without it. RViz shows the panoramic image, local motion compensation and camera orientation. All positions are fixed at zero by the rotational model; a `Path` display cannot show a meaningful translational route.

## 5. Validate saved outputs

After graceful shutdown:

```bash
.venv-converter/bin/python tools/validate_outputs.py \
  results/playroom_run01 converted/playroom.events.json
```

Expected outputs:

- `trajectory.tum`: `timestamp tx ty tz qx qy qz qw`, with zero translation and camera-to-world rotation. Samples are exported at 100 Hz for each finalized outgoing backend stride. Timestamp formatting preserves integer nanoseconds.
- `panorama.raw.yml.gz`: raw floating-point accumulated global IWE `IG`, saved on normal destruction.
- `panorama.png`: normalized inverse-grayscale view of the final raw IWE, without gamma correction. The live ROS panorama uses upstream gamma correction.

The validator checks strictly increasing timestamps within the recording, finite unit quaternions, zero translation, PNG dimensions with 2:1 aspect ratio and the raw map's presence. These checks do **not** prove estimator accuracy. The final partial frontend packet/backend window is not padded or forced through optimization; the estimated trajectory legitimately ends before the recording.

ROS interfaces (namespace `/cmax`):

| Topic | Type / meaning |
| --- | --- |
| `/cmax/events` | `cmax_slam_ros2/msg/EventArray`, topic-mode input |
| `/cmax/camera_info` | `sensor_msgs/msg/CameraInfo`, transient-local calibration input |
| `/cmax/angular_velocity` | `geometry_msgs/msg/TwistStamped`, rad/s; upstream published degrees/s, so only publication units are corrected |
| `/cmax/pose` | Optimized orientation, zero position; transient local |
| `/cmax/path` | Recent optimized pose samples, visualization capped at 10,000; file remains complete |
| `/cmax/pano_map`, `/cmax/local_iwe` | `sensor_msgs/msg/Image`, reliable transient local |
| `/tf` | `cmax_world → cmax_camera_optical`, event timestamps, optical x-right/y-down/z-forward convention |
| `/cmax/status` | Offline completion counts, reliable transient local |

Offline mode directly feeds converted events into the ROS 2 node and does not republish each input event over DDS. It uses recorded timestamps for all algorithm outputs and does not publish `/clock`. Keep `use_sim_time=false`. Historical TF stamps may appear stale in wall-clock tools; the latched Pose/Image RViz displays avoid depending on a current-time TF lookup.

Topic mode is available through `ros2 launch cmax_slam_ros2 live.launch.py`. Publish a reliable transient-local CameraInfo first, then ordered EventArray messages on `/cmax/events`. All numerical parameters are startup-only declared ROS 2 parameters. Topic-mode transport loss/performance has not been tested; use direct offline ingestion for the first reproducible experiment.

## 6. What was preserved and what changed

Upstream revision: `12342de85cf46b81fc45211714853ee0233284a3`.

Preserved: local/global contrast objectives and analytical derivatives, GSL optimization routines and termination settings, event warping/projection, SO(3) spline evaluation/fitting, angular-velocity integration, sliding-window optimization and panoramic accumulation. No SNN, new estimator or replacement map model was introduced.

Port changes: ROS 2 publishers/subscribers, declared parameters, custom event messages, integer timestamp value types, explicit timestamp-to-message conversion, ament build, added pose/TF/file outputs, deterministic serialized frontend/backend scheduling, startup/input guards, safe iterator stepping and one existing map-visit mask boundary check. The separate source patch records these changes. The original unsafe detached backend thread is removed. Completed windows are processed with the original readiness criterion after each sampled event. Serial scheduling changes runtime and may change floating-point/order-sensitive behavior; numerical parity requires actual comparison runs.

The world-to-camera convention is not guessed: upstream `warpEventToMap` explicitly computes `R * bearing_camera` to obtain the world ray, so exported quaternions represent camera-to-world rotation.

## 7. Tested versus unverified

Tested here:

- Seven integration tests create and decode actual ROS bag V2 fixtures using embedded custom event definitions.
- Exact event coordinates, polarity, integer timestamps, count and binary header round-trip.
- Embedded ROS 1 CameraInfo extraction, missing calibration, mismatched dimensions, invalid projection row, out-of-order rejection and no-overwrite behavior.
- Timestamp arithmetic/map-key semantics compiled and executed with a host C++17 compiler.
- Python compilation, shell syntax, XML/YAML parsing and upstream numerical-source/third-party hash audit.

**Not tested here:** full colcon/C++ compilation against ROS 2, macOS ARM64 linking, OpenCV 5/Eigen 5 compatibility, RViz runtime, either of your real recordings, accuracy, performance or parity with the published implementation. This execution host has no ROS 2 or native macOS libraries. The supplied source port is ready for a first native build attempt, not a certified working build.

For a scientific validation, preserve the bag/conversion/build manifests, obtain the publisher's correct ground-truth rotations and sequence-specific parameters, compare matched SO(3) angular errors after a documented frame/gauge alignment, and inspect map quality. Do not treat zero-position TUM output as full 6-DoF SLAM or report ordinary translation ATE for this model.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Python cannot import rclpy | Interpreter mismatch; use the Python that matches existing Jazzy |
| GSL not found | Confirm it exists in Homebrew; this optimizer requires it |
| OpenCV ABI rejection | Inspect preflight dylib paths; align a separate ROS vision overlay with the selected OpenCV |
| Eigen/Sophus compilation errors | Capture exact compiler error; Eigen 5 compatibility is unverified, do not globally downgrade |
| No map / zero windows | Verify calibration, selected event topic, event rate and packet/window settings |
| Empty output map files | Wait for EOF, then Ctrl-C gracefully; verify writable results directory |
| No visualized rotation | Use the latched camera Pose display; the model has no translation |
| Gazebo is not opening | Expected: recorded ECRot replay does not launch Gazebo |

Upstream: https://github.com/tub-rip/cmax_slam ; dataset: https://github.com/tub-rip/ECRot . Cite the original authors and identify this experimental port and its changes when reporting results.
