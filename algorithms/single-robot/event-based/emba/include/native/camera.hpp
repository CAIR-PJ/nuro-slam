#pragma once
#include "native/io.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
namespace native {
// No ROS image_geometry/cv_bridge dylib: avoids mixing OpenCV 4 ROS binaries with OpenCV 5.
class PinholeCameraModel {
  CameraInfo c_;bool distorted_=false;
public:
  void fromCameraInfo(const CameraInfo& c) {validate_calibration(c);c_=c;distorted_=std::any_of(c.D.begin(),c.D.end(),[](double x){return x!=0;});}
  cv::Size fullResolution()const{return cv::Size(c_.width,c_.height);}
  cv::Point2d rectifyPoint(const cv::Point2d& uv)const {
    if(!distorted_)return uv;
    cv::Matx33d K,R,P;
    for(int i=0;i<9;++i){K.val[i]=c_.K[i];R.val[i]=c_.R[i];P.val[i]=c_.P[(i/3)*4+i%3];}
    std::vector<cv::Point2d> src{uv},dst;
    cv::undistortPoints(src,dst,K,c_.D,R,P);return dst.at(0);
  }
  cv::Point3d projectPixelTo3dRay(const cv::Point2d& p)const {
    return cv::Point3d((p.x-c_.P[2]-c_.P[3])/c_.P[0],(p.y-c_.P[6]-c_.P[7])/c_.P[5],1.0);
  }
};
}
