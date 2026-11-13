#pragma once

#include "native/types.hpp"

#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>

namespace native {

class PinholeCameraModel {
public:
  void fromCameraInfo(const CameraInfo& c) {
    if (c.width == 0 || c.height == 0)
      throw std::invalid_argument("Invalid camera dimensions");

    if (c.K[0] <= 0 || c.K[4] <= 0 ||
        !std::isfinite(c.K[0]) || !std::isfinite(c.K[4]))
      throw std::invalid_argument("Invalid focal length");

    if (c.distortion_model != "plumb_bob")
      throw std::invalid_argument("Unsupported distortion model");

    for (double d : c.D)
      if (!std::isfinite(d) || std::abs(d) > 1e-12)
        throw std::invalid_argument(
          "Native baseline requires zero distortion");

    for (size_t i = 0; i < 9; ++i) {
      double expected = (i % 4 == 0) ? 1.0 : 0.0;
      if (!std::isfinite(c.R[i]) ||
          std::abs(c.R[i] - expected) > 1e-12)
        throw std::invalid_argument(
          "Native baseline requires identity rectification");
    }

    if (std::abs(c.K[0] - c.P[0]) > 1e-9 ||
        std::abs(c.K[4] - c.P[5]) > 1e-9 ||
        std::abs(c.K[2] - c.P[2]) > 1e-9 ||
        std::abs(c.K[5] - c.P[6]) > 1e-9)
      throw std::invalid_argument(
        "Native baseline requires matching K and P");

    width_ = static_cast<int>(c.width);
    height_ = static_cast<int>(c.height);

    fx_ = c.P[0];
    fy_ = c.P[5];
    cx_ = c.P[2];
    cy_ = c.P[6];
  }

  cv::Size fullResolution() const {
    return cv::Size(width_, height_);
  }

  cv::Point2d rectifyPoint(const cv::Point2d& p) const {
    return p;
  }

  cv::Point3d projectPixelTo3dRay(
      const cv::Point2d& p) const {
    if (width_ <= 0 || height_ <= 0)
      throw std::runtime_error("Camera not initialized");

    return cv::Point3d(
      (p.x - cx_) / fx_,
      (p.y - cy_) / fy_,
      1.0);
  }

private:
  int width_ = 0;
  int height_ = 0;

  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
};

}
