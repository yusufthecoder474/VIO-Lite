#include "vio/core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
namespace vio {
Motion track(const cv::Mat&a,const cv::Mat&b,const Calibration&c,const Options&o) {
 Motion out;
 if(a.empty()||b.empty()||a.type()!=CV_8UC1||b.type()!=CV_8UC1||a.size()!=b.size()||a.cols!=c.width||a.rows!=c.height){out.status="invalid_image";return out;}
 try {
 std::vector<cv::Point2f> p,q,back; std::vector<uchar> good,reverse;std::vector<float> err;
 cv::goodFeaturesToTrack(a,p,1200,.01,8);
 if(p.size()<size_t(o.min_matches)){out.status="few_features";return out;}
 cv::calcOpticalFlowPyrLK(a,b,p,q,good,err,cv::Size(21,21),3);
 cv::calcOpticalFlowPyrLK(b,a,q,back,reverse,err,cv::Size(21,21),3);
 std::vector<cv::Point2f> x,y;std::vector<double> flow;
 for(size_t i=0;i<p.size();i++)if(good[i]&&reverse[i]&&cv::norm(p[i]-back[i])<1&&q[i].x>=0&&q[i].y>=0&&q[i].x<b.cols&&q[i].y<b.rows){x.push_back(p[i]);y.push_back(q[i]);flow.push_back(cv::norm(p[i]-q[i]));}
 out.matches=(int)x.size(); if(out.matches<o.min_matches){out.status="few_matches";return out;}
 cv::undistortPoints(x,x,c.K,c.distortion);cv::undistortPoints(y,y,c.K,c.distortion);
 cv::Mat mask;double focal=.5*(c.K.at<double>(0,0)+c.K.at<double>(1,1));
 cv::Mat E=cv::findEssentialMat(x,y,1.0,cv::Point2d(0,0),cv::RANSAC,.999,1.0/focal,mask);
 if(E.empty()||E.rows!=3||E.cols!=3){out.status="essential_failed";return out;}
 cv::Mat R,t;out.inliers=cv::recoverPose(E,x,y,cv::Mat::eye(3,3,CV_64F),R,t,1000.0,mask);
 if(out.inliers<o.min_matches||out.inliers<double(out.matches)*.35){out.status="few_inliers";return out;}
 for(int r=0;r<3;r++){for(int k=0;k<3;k++)out.R_21(r,k)=R.at<double>(r,k);out.t_21[r]=t.at<double>(r);}
 // Rotation-compensated angular disparity avoids treating pure rotation as translation.
 flow.clear();
 for(size_t i=0;i<x.size();i++)if(mask.at<uchar>((int)i)) {
  Vec u=out.R_21*Vec(x[i].x,x[i].y,1),v(y[i].x,y[i].y,1);
  double cosine=std::clamp(u.normalized().dot(v.normalized()),-1.,1.);
  flow.push_back(std::acos(cosine)*focal);
 }
 if(flow.empty()){out.status="no_parallax";return out;}
 std::nth_element(flow.begin(),flow.begin()+flow.size()/2,flow.end());out.parallax=flow[flow.size()/2];
 if(out.parallax<o.min_parallax){out.status="low_parallax";return out;}
 if(!out.R_21.allFinite()||!out.t_21.allFinite()||std::abs(out.R_21.determinant()-1)>1e-5||std::abs(out.t_21.norm()-1)>1e-5){out.status="invalid_transform";return out;}
 out.valid=true;out.status="tracked";
 }catch(const cv::Exception&){out.status="opencv_failure";}
 return out;
}
void compose(Mat&R,Vec&p,const Motion&m) {
 if(!m.valid)return;
 Mat next=R*m.R_21.transpose();p-=next*m.t_21;R=next;
}
}
