# EMBA native C++17 / ROS 2 Jazzy rewrite

**Experimental source port. The full EMBA executable, ROS 2 wrapper and macOS ARM64 build have not been compiled or executed here. Numerical parity with upstream is unverified.** The ZIP is source-complete, including the original solver code, bundled dependencies, adapters, tests and pristine mathematical reference files.

Reference: https://github.com/tub-rip/emba at **11225df786a65c059f113a903349c1d3f8f68829**. This is an offline event-mosaicing bundle-adjustment backend: it refines an existing SO(3) trajectory and a panorama gradient map. It does not independently estimate a trajectory from events alone and is not full 6-DoF SLAM.

## What is implemented

- Separate `src/emba_ros2` ament_cmake package; no CMax-SLAM or EROAM files are modified.
- ROS-independent `emba_core` with upstream LEGM residuals, analytical Jacobians, normal equations, Schur/LDLT or CG solve, regularization, IRLS, LM updates and SO(3) splines.
- Native `CMAXEV1` reader with dimensions, count, byte-length, timestamp-order, coordinate and polarity checks.
- Signed int64 nanosecond time types; exact decimal/scientific TUM timestamp parsing.
- Calibration adapter using native OpenCV, with no ROS cv_bridge/image_geometry dependency or OpenCV ABI mixing through those packages.
- TUM quaternion adapter, optional explicit SO(3) seed resampling, gradient-map loading and reproducible map initialization.
- Standalone `emba_cli` plus thin `rclcpp` offline wrapper; ROS output publication does not enter the numerical solver.
- Explicit single-thread configuration: no `-fopenmp`, OpenMP target, FFTW threads/MPI library or direct BLAS linkage. Eigen parallelism is disabled, and OpenCV threading/OpenCL are disabled at entry.
- Final sampled trajectory, raw gradient maps, reconstructed panorama, LM trace, optional normal-equation dumps and RViz configuration.

## 1. Extract into a NEW directory

Recommended extraction parent: `~/neuromorphic_slam_ws/rewrites/`. The ZIP contains `emba_ros2_port/` beneath it. This avoids overwriting your existing `~/neuromorphic_slam_ws/ports/emba_ros2_port` working copy.

```zsh
cd ~/neuromorphic_slam_ws/rewrites/emba_ros2_port
```

This workspace is independent. Keep existing CMax-SLAM, EROAM and upstream EMBA checkouts where they are.

## 2. Run the dependency-light tests

For the Python tools only, use an isolated environment:

```zsh
python3 -m venv .venv-tools
.venv-tools/bin/python -m pip install -r tools/requirements.txt
.venv-tools/bin/python -m unittest discover -s tests -v
```

To compile/run the standard-library and bundled SO(3) tests, use a terminal whose `python3` already has NumPy/PyYAML, or activate that tools environment first:

```zsh
source .venv-tools/bin/activate
bash tools/test_adapters.sh
deactivate
```

The SO(3) header test intentionally uses the bundled Eigen copy to test the bundled Sophus/Basalt implementation without changing your Homebrew installation. This does **not** test Eigen 5 compatibility.

## 3. Prepare the camera calibration

The application reads a small checked `EMBACAL1` text file. Generate it from either standard ROS camera YAML or the existing CMAX `*.events.camera.yaml` parameter file:

```zsh
.venv-tools/bin/python tools/prepare_calibration.py \
  /absolute/path/playroom.events.camera.yaml \
  /absolute/path/playroom.calib
```

A corrected CMAX calibration file does not need a repair flag. Alternatively, the original upstream Playroom YAML is included:

```zsh
.venv-tools/bin/python tools/prepare_calibration.py \
  docs/upstream_calibration/DVS-playroom.yaml \
  /absolute/path/playroom.calib \
  --repair-projection-from-k
```

**The repair is explicit:** upstream's Playroom YAML has projection last row `[0,0,0,1]`. The option constructs monocular `P=[K|0]`, retaining the original file and recording both matrices and their hashes in `playroom.calib.json`. This adjustment is not a verified reproduction of the publisher's calibration processing.

The baseline supports full-resolution monocular pinhole cameras, identity rectification `R`, canonical zero-skew `K`, and OpenCV `plumb_bob`/`rational_polynomial` distortion. It rejects unsupported models rather than inventing calibration. Coordinates must match CMAXEV1 dimensions. CameraInfo ROI/binning and fisheye models are not implemented.

## 4. Build with your existing native Jazzy environment

In a fresh terminal, source the environment you already use:

```zsh
source ~/robotics2/robotics_env.zsh
cd ~/neuromorphic_slam_ws/rewrites/emba_ros2_port
bash tools/build_macos.sh
source install/setup.zsh
colcon test --packages-select emba_ros2
colcon test-result --verbose
```

The build script targets `/opt/homebrew/opt/ros2-jazzy` and `/opt/homebrew`, uses Apple Clang `/usr/bin/clang++`, and does not install, upgrade, unlink or relink packages. It requires the existing `eigen`, `opencv`, `fftw`, `boost`, `glog` and ROS 2 packages. The ROS-compatible Python must import `rclpy`, `ament_package`, `colcon_core`, `em` and `lark`. Your converter/tools Python may be a different interpreter.

If your default Python does not match Homebrew ROS, select the already-installed compatible interpreter:

```zsh
EMBA_ROS_PYTHON=/absolute/path/to/compatible/python3 bash tools/build_macos.sh
```

Do not use the tools virtual environment as the ROS interpreter unless it actually contains the matching ROS extensions. A Python mismatch must be resolved before Ament can build.

Target dependencies are C++17, installed Eigen through `Eigen3::Eigen`, bundled Sophus/Basalt/cereal, OpenCV 5, serial FFTW3, Boost headers and glog. Ceres, Catkin, `roscpp`, `dvs_msgs`, ROS 1 bag APIs, `cv_bridge`, `image_geometry`, and `image_transport` are absent from the compiled port.

**OpenCV 5 and Eigen 5 source compatibility are unverified.** CMake reports the selected versions and libraries. If configuration cannot find their existing package configs, pass explicit `-DOpenCV_DIR=...` / `-DEigen3_DIR=...` paths to the build script. Do not downgrade or replace your working installation automatically.

The original Poisson reconstruction source carries GPL-2.0-or-later notices alongside the repository's MIT license. These notices, the GPL text and third-party licenses are retained; the combined deliverable is not represented as exclusively MIT licensed.

### Standalone build without ROS dependencies

This builds the same core and CLI, useful for isolating numerical compilation from ROS integration:

```zsh
cmake -S src/emba_ros2 -B build-native \
  -DEMBA_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
  -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build-native -j 4
ctest --test-dir build-native --output-on-failure
```

This still requires OpenCV/Eigen/FFTW/Boost/glog. It is a native executable, not a VM or a ROS 1 runtime.

## 5. Run a first refinement

You need three real inputs:

1. Existing CMAXEV1 event file, with magic `CMAXEV1\n`, little-endian width/height/count and records `<HHqB` (x, y, int64 timestamp ns, polarity).
2. The generated `.calib` file.
3. CMax-SLAM or EROAM initial trajectory in TUM order: `timestamp tx ty tz qx qy qz qw`, camera-to-world optical rotation, zero translation.

No event dataset or fabricated initial trajectory is bundled. This reader consumes the existing converted recording directly. It does not reopen ROS 1 bags or depend on another port's source package.

```zsh
ros2 run emba_ros2 emba_cli \
  --events /absolute/path/playroom.events \
  --calibration /absolute/path/playroom.calib \
  --trajectory /absolute/path/cmax_trajectory.tum \
  --output "$PWD/results/playroom_cmax_seed_run01"
```

Or use `./build-native/emba_cli` after the standalone build. Choose a new output directory per run; nonempty directories are rejected.

By default the run uses the common time interval covered by events and initial poses, trims its end to a whole number of knot intervals, and records the chosen bounds in `run.txt`. For a controlled reproduction, specify the exact **absolute event-clock nanoseconds**, not seconds:

```zsh
ros2 run emba_ros2 emba_cli \
  --events /absolute/path/playroom.events \
  --calibration /absolute/path/playroom.calib \
  --trajectory /absolute/path/cmax_trajectory.tum \
  --start-ns 100000000 --stop-ns 2400000000 \
  --dt-knots 0.05 \
  --output "$PWD/results/playroom_explicit_run01"
```

The example bounds mean 0.1–2.4 seconds **only if your recording and seed share a zero-based clock**. Confirm actual timestamps first. The program refuses an interval not covered by both inputs. `--trajectory-offset-ns` applies an explicitly known clock offset; it does not estimate synchronization or align frames automatically.

Internally, the exact start timestamp is subtracted from event/pose timestamps before spline fitting. Output adds it back using int64 arithmetic. This keeps recorded timestamps exact and avoids epoch-scale floating-point conditioning in the legacy spline-fit implementation.

### Initial-trajectory density

The original fitting code excludes knot endpoints and requires at least two interior poses per knot segment. A 100 Hz seed with 0.05 s knots usually has enough samples; the program checks every segment. It does not extrapolate missing motion.

If you deliberately want to densify a sparse seed by SO(3) interpolation, add:

```text
--seed-step-ns 1000000
```

That explicitly interpolates at 1 ms using `R0 * exp(u * log(R0^-1 * R1))` and records the setting. It adds no new motion information and changes the fitted initialization; match this preprocessing in any upstream parity comparison. It is disabled by default.

### Initial map and execution profile

The default is a 256×128 panorama with no supplied gradient map. The analytical solver then starts from seeded random gradient noise, following upstream's no-map branch, with a deterministic seed of 1. CG starts from zero gradients. The median-filter initialization is retained.

**These defaults are an execution profile, not published Playroom settings.** The upstream Playroom launch uses an available initial map. To reproduce that case, supply the corresponding original gradient maps and exact height:

```text
--gx /absolute/path/Gx.bin --gy /absolute/path/Gy.bin --pano-height HEIGHT
```

`Gx.bin` and `Gy.bin` must be row-major little-endian float64 maps of exactly `2*HEIGHT*HEIGHT` pixels each. A CMax event-count panorama PNG or IWE is **not** interchangeable with an EMBA gradient map. No unvalidated image-to-gradient initializer is silently inserted.

The CLI accepts `--help` for solver, regularizer, IRLS, subsampling and diagnostics options. The map size and full BA interval can make the dense pose-map cross block large; start with the supplied modest map resolution before scaling. Higher memory demand is intrinsic to this implementation's normal-equation representation.

## 6. ROS 2 and RViz

```zsh
ros2 launch emba_ros2 offline.launch.py \
  events:=/absolute/path/playroom.events \
  calibration:=/absolute/path/playroom.calib \
  trajectory:=/absolute/path/cmax_trajectory.tum \
  output:="$PWD/results/playroom_ros2_run01"
```

Optional launch arguments: `start_ns`, `stop_ns`, `seed_step_ns`, `trajectory_offset_ns`, `gx`, `gy`, `rviz`. Numerical defaults are in `config/serial.yaml`. The node declares its numerical parameters at startup.

The wrapper runs offline BA, then publishes final results and stays alive for RViz; it does not stream intermediate optimization states. Inputs are files, not event topics. Outputs are reliable/transient-local:

| Topic | Content |
| --- | --- |
| `/emba/panorama` | Reconstructed intensity panorama, mono8 |
| `/emba/gradient_x`, `/emba/gradient_y` | Raw 64FC1 gradient maps |
| `/emba/path` | Optimized rotation samples; all positions fixed at zero |
| `/emba/orientation` | Last published rotation for RViz axes |
| `/emba/status` | Completion message and results path |

RViz uses `emba_world`. No `/clock` or TF is fabricated. `use_sim_time` is unnecessary for this recorded-data backend. The visualization path may be decimated to at most roughly 10,000 samples; the saved trajectory remains complete. Since positions are zero, a line-path plot does not show useful rotational motion.

Gazebo Harmonic 8.14 can remain installed and is not involved in this ECRot refinement. Simulated event generation is outside this deliverable.

## 7. Outputs and validation

- `trajectory.tum`: optimized spline sampled at 1 ms, with exact restored timestamps and double-precision quaternions. The terminal spline endpoint is excluded because upstream spline evaluation uses a half-open interval.
- `Gx.bin`, `Gy.bin`: raw final float64 gradient maps.
- `panorama.png`: upstream Poisson reconstruction with robust display normalization; display only, not a quantitative map metric.
- `run.txt`: actual interval/origin, input paths, seed, solver/model settings and selected event count.
- `solver_trace.csv`: iteration, lambda, incumbent cost, candidate cost, accepted flag.
- `final_results/iterations.txt`: upstream-style textual progress.
- With `--diagnostics 1`: evolving images and `normal_*.txt` dumps of residuals and regularized normal equations **before gauge removal**. These can be large and affect runtime.

```zsh
.venv-tools/bin/python tools/validate_run.py results/playroom_cmax_seed_run01
```

This checks timestamp ordering/range, quaternion norms, zero translation, map dimensions, image structure and LM acceptance-trace consistency. It does not establish accuracy, convergence quality or equivalence to upstream. Preserve SHA-256 hashes of all input files, compiler/library versions and build logs with each experiment.

## 8. Preservation, changes and parity

Read `docs/NUMERICAL_PARITY.md` and `docs/UPSTREAM_TO_NATIVE.patch` before reporting results. Pristine source/header files are supplied in `docs/upstream_reference`, and bundled third-party bytes are audited in `docs/SOURCE_MANIFEST.json`.

Preserved: LEGM event-pair model, residual signs/polarity, analytical spatial/pose derivatives, active pixel selection, L2 regularization, robust weights, LM diagonal damping and acceptance rules, Schur and CG implementations, first-pose gauge fixing, left SO(3) updates and upstream spline fitting.

Explicit changes: ROS-free orchestration and adapters; exact clock-origin rebasing; input rejection instead of unsafe accesses; initialized unused pose index; rejected-candidate memory cleanup; initialized residual return value; defined Poisson boundary storage; deterministic random seed; serial runtime; precise sampled TUM output and optional diagnostic traces.

Preserved upstream quirks include integer division in the hardcoded 100-event mini-batch count (the last partial batch is ignored), stride-100 event-window selection with safety clamping, CG settings (100 iterations / 1e-6 tolerance), and the LM `iter <= max_num_iter` loop. Other mini-batch sizes are rejected rather than claiming that the unused upstream parameter works.

The baseline stops if a rounded warped event would index outside the panorama at a seam/pole, instead of silently wrapping/clamping and changing the model. Full-seam robustness remains a limitation. A future seam fix must be validated and disclosed separately.

Required parity work: match events/order/subsampling, calibration, seed trajectory preprocessing, map initialization, map resolution and all parameters; compare bearing rays, spline evaluations and Jacobians, residual vectors/inlier counts, robust weights, active sets, normal-equation blocks, solved increments, accepted LM steps and final SO(3)/gradient maps. See the numerical-parity document for exact stages and suggested tolerances. Source preservation alone does not prove numerical equivalence.

## 9. Tested and unverified

**Executed on this Linux x86_64 host:** 22 compiled standard-library adapter checks; bundled Eigen/Sophus/Basalt interpolation and finite-difference spline-Jacobian test (maximum difference about 2.22e-9); six Python calibration/dump-tool tests; adapter-only CMake/CTest build; Python/shell/XML/YAML checks and upstream-source hash audit. Test logs are included in `test_results/`.

**Not executed here:** full core/CLI build, full numerical-invariant test executable, ROS 2 wrapper/launch/RViz, Apple Silicon linking, installed Eigen 5/OpenCV 5 compatibility, actual Playroom/Victory Column runs, energy/runtime comparisons, or upstream numerical parity. The host lacks the required ROS/OpenCV/FFTW development environment. No successful full build or scientific reproduction is claimed.

To check the OpenMP issue on your Mac after building, inspect the actual executable and loaded libraries with `otool -L` and a diagnostic run using `DYLD_PRINT_LIBRARIES=1`. This port introduces no second runtime itself, but transitive Homebrew library loading still needs verification. `KMP_DUPLICATE_LIB_OK` is not used, and a thread-count request alone is not evidence that the process uses one worker.
