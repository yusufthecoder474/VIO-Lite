#include "vio/core.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
namespace vio {
Scale fitScale(const std::vector<ScalePoint>& pts,const Options&o) {
 Scale out;out.status="window_too_short";
 if(pts.size()<6)return out;
 // q_i*s - v0*t_i = integrated_acceleration_i + camera_lever_i.
 // Remove t's component from q to test scale observability independently of velocity.
 size_t n=pts.size();Eigen::MatrixXd A(3*n,4);Eigen::VectorXd b(3*n);
 double tt=0;Vec tq=Vec::Zero();
 for(size_t i=0;i<n;i++) {
  const auto&p=pts[i];
  if(!std::isfinite(p.t)||p.t<0||!p.visual.allFinite()||!p.inertial.allFinite()||!p.lever.allFinite()||(i&&p.t<=pts[i-1].t)){out.status="invalid_window";return out;}
  if(i==0&&(std::abs(p.t)>1e-9||p.visual.norm()>1e-8||p.inertial.norm()>1e-8||p.lever.norm()>1e-8)){out.status="window_origin_required";return out;}
  A.block<3,1>(3*i,0)=p.visual;A.block<3,3>(3*i,1)=-p.t*Mat::Identity();
  b.segment<3>(3*i)=p.inertial+p.lever;tt+=p.t*p.t;tq+=p.t*p.visual;
 }
 if(tt<1e-8){out.status="degenerate_time";return out;}
 double energy=0;for(auto&p:pts)energy+=(p.visual-p.t*tq/tt).squaredNorm();
 out.excitation=std::sqrt(energy/n);
 if(out.excitation<o.min_excitation){out.status="insufficient_excitation";return out;}
 auto solve=[&](const Eigen::MatrixXd& X,const Eigen::VectorXd& y,Eigen::VectorXd& result) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(X,Eigen::ComputeThinU|Eigen::ComputeThinV);
  auto singular=svd.singularValues();
  if(singular.size()<4||singular[3]<1e-8||singular[0]/singular[3]>1e5)return false;
  result=svd.solve(y);return result.allFinite();
 };
 Eigen::VectorXd solution;
 if(!solve(A,b,solution)){out.status="ill_conditioned";return out;}
 // One robust refit: discard gross residuals, keep at least 70% and six samples.
 std::vector<double> errors;for(size_t i=0;i<n;i++)errors.push_back((A.block(3*i,0,3,4)*solution-b.segment(3*i,3)).norm());
 auto sorted=errors;std::sort(sorted.begin(),sorted.end());double cut=std::max(.02,3*sorted[n/2]);
 std::vector<size_t> keep;for(size_t i=0;i<n;i++)if(errors[i]<=cut)keep.push_back(i);
 if(keep.size()<6||keep.size()<.7*n){out.status="too_many_outliers";return out;}
 Eigen::MatrixXd X(3*keep.size(),4);Eigen::VectorXd y(3*keep.size());
 for(size_t j=0;j<keep.size();j++){X.block(3*j,0,3,4)=A.block(3*keep[j],0,3,4);y.segment(3*j,3)=b.segment(3*keep[j],3);}
 if(!solve(X,y,solution)){out.status="ill_conditioned_refit";return out;}
 out.value=solution[0];out.used=(int)keep.size();
 double rmse=(X*solution-y).norm()/std::sqrt(double(keep.size()));
 double magnitude=y.norm()/std::sqrt(double(keep.size()));out.residual=rmse/std::max(.02,magnitude);
 if(out.value<=0||out.value>100){out.status="nonpositive_or_extreme_scale";return out;}
 if(out.residual>o.max_scale_residual){out.status="inconsistent_inertial_motion";return out;}
 out.valid=true;out.status="accepted";return out;
}
}
