#include "emba/model.h"
#include "native/camera.hpp"
#include <iostream>
#include <glog/logging.h>
#include <opencv2/core/ocl.hpp>
void require(bool b,const char* label){if(!b)throw std::runtime_error(label);}
int main(int argc,char** argv){
  google::InitGoogleLogging(argv[0]);cv::setNumThreads(1);cv::ocl::setUseOpenCL(false);Eigen::setNbThreads(1);
  try {
    native::CameraInfo c;c.width=128;c.height=128;c.distortion_model="plumb_bob";
    c.K={91.,0,64,0,91,64,0,0,1};c.R={1,0,0,0,1,0,0,0,1};c.P={91.,0,64,0,0,91,64,0,0,0,1,0};c.D.assign(5,0.);
    native::PinholeCameraModel camera;camera.fromCameraInfo(c);auto ray=camera.projectPixelTo3dRay(camera.rectifyPoint({64,64}));
    require(cv::norm(ray-cv::Point3d(0,0,1))<1e-14,"pinhole center bearing");
    dvs::EventWarper warper;warper.initialize(c,128,64);auto R=Sophus::SO3d::exp(Eigen::Vector3d(.03,.05,.02));cv::Matx23d J;
    auto pm=warper.warpEventToMap({70,60},R,&J);(void)pm;
    for(int j=0;j<3;++j){Eigen::Vector3d d=Eigen::Vector3d::Zero();d[j]=1e-7;
      auto plus=warper.warpEventToMap({70,60},Sophus::SO3d::exp(d)*R,nullptr);
      auto minus=warper.warpEventToMap({70,60},Sophus::SO3d::exp(-d)*R,nullptr);
      require(std::abs((plus.x-minus.x)/2e-7-J(0,j))<1e-5,"warper x derivative");
      require(std::abs((plus.y-minus.y)/2e-7-J(1,j))<1e-5,"warper y derivative");}
    TrajectorySettings settings;settings.t_beg=native::Time(0);settings.t_end=native::Time(.05);settings.dt_knots=.05;
    LinearTrajectory trajectory(settings);trajectory.pushbackCtrlPoses({Sophus::SO3d(),Sophus::SO3d()});
    EMBA::LEGM model(c,.45,128,64);EMBA::EventPacket events;
    for(int i=0;i<200;++i){native::Event e;e.x=64;e.y=64;e.ts=native::Time::fromNSec((i+1)*100000);e.polarity=true;events.push_back(e);}
    auto gx=cv::Mat::zeros(64,128,CV_64FC1),gy=cv::Mat::zeros(64,128,CV_64FC1);cv::Mat counts=cv::Mat::zeros(64,128,CV_32SC1);
    auto residual=model.evaluateDataError(&trajectory,gx,gy,events,true,counts);
    require(residual.size()==199,"static fixture residual count");require((residual.array()-.45).abs().maxCoeff()<1e-12,"static fixture residual value");
    EMBA::VecXd ep(2);ep<<.1,.4;
    require(std::abs(model.evaluateRobustDataCost(ep,"huber",.2)-.065)<1e-12,"Huber objective");
    auto cauchy=.5/.2*(std::log1p(.2*.01)+std::log1p(.2*.16));
    require(std::abs(model.evaluateRobustDataCost(ep,"cauchy",.2)-cauchy)<1e-12,"Cauchy objective");
    EMBA::MatXd A11=5*EMBA::MatXd::Identity(3,3),A12(3,2);A12<<.1,.2,.3,.1,.2,.1;
    std::vector<EMBA::Mat2d> A22(1);A22[0]<<3,.1,.1,4;
    EMBA::VecXd b1=EMBA::VecXd::Ones(3),b2=EMBA::VecXd::Constant(2,.2),x1,x2;
    model.solveNormalEq(A11,A12,A22,b1,b2,.001,x1,x2);
    Eigen::MatrixXd full(5,5);full<<A11,A12,A12.transpose(),A22[0];full.diagonal()*=1.001;
    Eigen::VectorXd rhs(5);rhs<<b1,b2;Eigen::VectorXd exact=full.ldlt().solve(rhs);
    require((x1-exact.head(3)).norm()+(x2-exact.tail(2)).norm()<1e-11,"Schur solve versus dense damped system");
    model.solveNormalEqCG(A11,A12,A22,b1,b2,.001,x1,x2);
    require((x1-exact.head(3)).norm()+(x2-exact.tail(2)).norm()<1e-5,"CG solve versus dense damped system");
    std::cout<<"Numeric invariants passed (not an upstream parity comparison)\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
  google::ShutdownGoogleLogging();return 0;
}
