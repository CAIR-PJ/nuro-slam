# OpenCV 5 compatibility patch

This update addresses the CMake configure error `Could NOT find OpenCV (missing: calib3d)` with Homebrew OpenCV 5.0.0.

- CMakeLists.txt: request the `calib` module instead of the removed `calib3d` module.
- camera_adapter.hpp: include `opencv2/calib.hpp` when available; fall back to `opencv2/calib3d.hpp` for OpenCV 4.
- `cv::undistortPoints()` and all camera matrices and numerical calls are unchanged.

This patch has not been compiled on the user's macOS system. If OpenCV 5 has further API changes, the compiler output will identify them.
