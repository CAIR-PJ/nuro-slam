# Approach B: local camera and image adapters

This revision removes direct CMax-SLAM dependencies on ROS 2 `cv_bridge` and `image_geometry` without changing the GSL optimization objective, frontend event warping, or backend trajectory optimizer.

## Source modifications

- `include/cmax_port/camera_adapter.hpp`: implements the *used subset* of `PinholeCameraModel`: `fromCameraInfo`, `fullResolution`, `fullIntrinsicMatrix`, `rectifyPoint`, and `projectPixelTo3dRay`. Rectification uses OpenCV `undistortPoints(K,D,R,P)`. Rectified ray uses P intrinsics. Supports `plumb_bob`, `rational_polynomial`, and empty distortion model only. No ROS 1/2 image_geometry dependency.
- `include/cmax_port/image_adapter.hpp`: converts `mono8` or `bgr8` OpenCV images to `sensor_msgs::msg::Image` with row-wise copy and proper step, stamp and frame. No cv_bridge dependency.
- `src/cmax_slam.cpp`, `src/frontend/ang_vel_estimator.*`, `src/backend/pose_graph_optimizer.cpp`, `include/cmax_slam.h`, and `include/backend/event_pano_warper.h`: call local adapters.
- `CMakeLists.txt`, `package.xml`: remove cv_bridge and image_geometry from package dependencies.
- `tools/preflight.py`: no longer rejects the unrelated installed `libimage_geometry`/`libcv_bridge` OpenCV linkage; continues checking native libraries, ROS 2 packages, and GSL.

## Impact and limitations

- GSL optimizers and event warping equations are unchanged by this revision.
- Camera ray generation can differ from image_geometry in distortion/rectification edge cases, so compare bearing vectors against a reference before asserting numerical parity. OpenCV 5 and Eigen 5 compatibility remain unverified.
- Image publication is output-only and should not alter estimates, but may change publication overhead.
- The macOS ARM64 ROS 2 build has **not** been executed here. This is a source-level modification, not a verified executable.
- The preflight can still fail if GSL is not installed. Install it separately using `brew install gsl` if acceptable; do not upgrade or relink the ROS 2 installation.
- This avoids **direct** cv_bridge/image_geometry linkage only; a ROS 2 transitive dependency could still load incompatible OpenCV libraries.

## Native commands

```zsh
cd ~/neuromorphic_slam_ws/ports/cmax_ros2_port
set +u
source "$HOME/robotics2/robotics_env.zsh"
set +u
brew list --versions gsl || brew install gsl
bash tools/build_macos.sh
source install/setup.zsh
ros2 pkg executables cmax_slam_ros2
```

Stop at the first compiler/linker error and share the first diagnostic. Do not symlink OpenCV 4 to OpenCV 5 or modify your working ROS installation.
