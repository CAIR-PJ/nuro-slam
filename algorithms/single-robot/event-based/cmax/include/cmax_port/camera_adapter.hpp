#pragma once
// Local replacement for the subset of cmax_port::PinholeCameraModel used by CMax-SLAM.
// Keep camera geometry explicit and independent of ROS image_geometry's OpenCV ABI.
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace cmax_port {
class PinholeCameraModel {
public:
  void fromCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
    if (!info) throw std::invalid_argument("null CameraInfo");
    if (!info->width || !info->height || info->k[0] <= 0 || info->k[4] <= 0 ||
        info->p[0] <= 0 || info->p[5] <= 0)
      throw std::invalid_argument("invalid CameraInfo intrinsics");
    if (info->distortion_model != "plumb_bob" &&
        info->distortion_model != "rational_polynomial" &&
        info->distortion_model != "")
      throw std::invalid_argument("unsupported distortion model: " + info->distortion_model);
    resolution_ = cv::Size(static_cast<int>(info->width), static_cast<int>(info->height));
    K_ = cv::Matx33d(info->k[0],info->k[1],info->k[2],
                     info->k[3],info->k[4],info->k[5],
                     info->k[6],info->k[7],info->k[8]);
    R_ = cv::Matx33d(info->r[0],info->r[1],info->r[2],
                     info->r[3],info->r[4],info->r[5],
                     info->r[6],info->r[7],info->r[8]);
    P_ = cv::Matx34d(info->p[0],info->p[1],info->p[2],info->p[3],
                     info->p[4],info->p[5],info->p[6],info->p[7],
                     info->p[8],info->p[9],info->p[10],info->p[11]);
    D_.clear();
    for (double d : info->d) D_.push_back(d);
    // Supported OpenCV plumb-bob (4/5) and rational polynomial (8) models.
    // Reject higher-order thin-prism/tilt terms rather than silently ignoring them.
    if (!(D_.empty() || D_.size() == 4 || D_.size() == 5 || D_.size() == 8))
      throw std::invalid_argument("unsupported distortion coefficient count");
    if (info->distortion_model == "rational_polynomial" && D_.size() != 8)
      throw std::invalid_argument("rational_polynomial requires 8 coefficients");
    if (std::abs(K_(0,0)) < 1e-12 || std::abs(K_(1,1)) < 1e-12)
      throw std::invalid_argument("invalid intrinsic focal lengths");
    // CameraInfo from the converter should already contain a valid pinhole P.
    if (std::abs(P_(2,2)) < 1e-12) throw std::invalid_argument("invalid projection P(2,2)");
  }
  cv::Size fullResolution() const { return resolution_; }
  cv::Matx33d fullIntrinsicMatrix() const { return K_; }
  cv::Point2d rectifyPoint(const cv::Point2d& raw) const {
    // Inverse pinhole projection with optional skew (OpenCV's common K has zero skew).
    const double yd = (raw.y - K_(1,2)) / K_(1,1);
    const double xd = (raw.x - K_(0,2) - K_(0,1)*yd) / K_(0,0);
    double x = xd, y = yd;
    if (!D_.empty()) {
      const auto d = [this](size_t i) { return i < D_.size() ? D_[i] : 0.0; };
      const double k1=d(0), k2=d(1), p1=d(2), p2=d(3);
      const double k3=d(4), k4=d(5), k5=d(6), k6=d(7);
      // OpenCV's default undistortPoints uses five fixed-point iterations.
      for (int i = 0; i < 5; ++i) {
        const double r2=x*x+y*y, r4=r2*r2, r6=r4*r2;
        const double numerator=1.0+k1*r2+k2*r4+k3*r6;
        const double denominator=1.0+k4*r2+k5*r4+k6*r6;
        if (std::abs(numerator) < 1e-15 || std::abs(denominator) < 1e-15)
          throw std::runtime_error("degenerate camera distortion polynomial");
        const double invRadial=denominator/numerator;
        const double dx=2.0*p1*x*y+p2*(r2+2.0*x*x);
        const double dy=p1*(r2+2.0*y*y)+2.0*p2*x*y;
        x=(xd-dx)*invRadial;
        y=(yd-dy)*invRadial;
      }
    }
    const cv::Vec3d rotated=R_*cv::Vec3d(x,y,1.0);
    if (std::abs(rotated[2]) < 1e-15)
      throw std::runtime_error("rectified ray has zero depth");
    const double xr=rotated[0]/rotated[2], yr=rotated[1]/rotated[2];
    // OpenCV undistortPoints uses the left 3x3 portion of P for projection;
    // P's translation column is not applied during rectification.
    const double u=P_(0,0)*xr+P_(0,1)*yr+P_(0,2);
    const double v=P_(1,0)*xr+P_(1,1)*yr+P_(1,2);
    const double w=P_(2,0)*xr+P_(2,1)*yr+P_(2,2);
    if (std::abs(w) < 1e-15) throw std::runtime_error("invalid projection depth");
    return cv::Point2d(u/w,v/w);
  }
  cv::Point3d projectPixelTo3dRay(const cv::Point2d& rectified) const {
    // image_geometry projects using the rectified P intrinsics; unit z ray.
    const double x=(rectified.x-P_(0,2)-P_(0,3))/P_(0,0);
    const double y=(rectified.y-P_(1,2)-P_(1,3)-P_(1,0)*x)/P_(1,1);
    return cv::Point3d(x,y,1.0);
  }
private:
  cv::Size resolution_{};
  cv::Matx33d K_{cv::Matx33d::eye()};
  cv::Matx33d R_{cv::Matx33d::eye()};
  cv::Matx34d P_{cv::Matx34d::zeros()};
  std::vector<double> D_;
};
} // namespace cmax_port
