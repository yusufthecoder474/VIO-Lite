#include "vio/core.hpp"
#include <stdexcept>
#include <cmath>
namespace vio {
Mat expSO3(const Vec& w) {
 double a=w.norm(); if(a<1e-12)return Mat::Identity(); return Eigen::AngleAxisd(a,w/a).toRotationMatrix();
}
Initialization initialize(const std::vector<Imu>& samples,const Options&o) {
 Initialization init;init.status="initialization_not_stationary";
 if(samples.size()<30 || double(samples.back().ns-samples.front().ns)*1e-9 < o.init_seconds-.01) {init.status="initialization_too_short";return init;}
 Vec g=Vec::Zero(),a=Vec::Zero();
 for(size_t i=0;i<samples.size();i++){
  const auto& s=samples[i];
  if(!s.gyro.allFinite()||!s.accel.allFinite()||(i&&s.ns<=samples[i-1].ns)){init.status="invalid_initialization_data";return init;}
  g+=s.gyro;a+=s.accel;
 }g/=samples.size();a/=samples.size();
 double av=0,gv=0; for(auto&s:samples){av+=(s.accel-a).squaredNorm();gv+=(s.gyro-g).squaredNorm();}
 av=std::sqrt(av/samples.size());gv=std::sqrt(gv/samples.size());
 Vec corrected=a-o.accel_bias;
 // A quiet interval is necessary, not proof of zero acceleration or zero angular rate.
 if(av>.15||gv>.02||(g-o.gyro_bias).norm()>.1||std::abs(corrected.norm()-9.81)>.35)return init;
 init.R_WB=Eigen::Quaterniond::FromTwoVectors(corrected.normalized(),Vec::UnitZ()).toRotationMatrix();
 init.gyro_bias=g;init.valid=true;init.status="stationary_assumption_accepted";return init;
}
void integrate(State&s,const std::vector<Imu>& data,const Options&o) {
 if(data.size()<2)throw std::runtime_error("Empty integration interval");
 for(size_t i=1;i<data.size();i++) {
  double dt=double(data[i].ns-data[i-1].ns)*1e-9;
  if(dt<=0||dt>o.max_imu_gap||!data[i].gyro.allFinite()||!data[i].accel.allFinite()||!data[i-1].gyro.allFinite()||!data[i-1].accel.allFinite())throw std::runtime_error("Invalid IMU step");
  Vec w=.5*(data[i-1].gyro+data[i].gyro)-o.gyro_bias;
  Vec f=.5*(data[i-1].accel+data[i].accel)-o.accel_bias;
  Mat mid=s.R_WB*expSO3(w*dt*.5);
  Vec a=mid*f+Vec(0,0,-9.81);
  s.p+=s.v*dt+.5*a*dt*dt;s.v+=a*dt;s.R_WB=s.R_WB*expSO3(w*dt);
 }
}
}
