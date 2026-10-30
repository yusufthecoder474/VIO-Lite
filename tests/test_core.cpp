#include "vio/core.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <chrono>
using namespace vio;
void require(bool v,const std::string&s){if(!v)throw std::runtime_error(s);}
template<class F> void rejects(F f){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}require(rejected,"expected rejection");}
void file(const std::filesystem::path&p,const std::string&s){std::ofstream f(p);f<<s;require(bool(f),"fixture write");}
std::string calibration(const std::string&transform="1,0,0,0.1,0,1,0,0,0,0,1,0,0,0,0,1") {
 return "camera_model: pinhole\ndistortion_model: radial-tangential\nintrinsics: [400.,400.,320.,240.]\ndistortion_coefficients: [0.,0.,0.,0.]\nresolution: [640,480]\nT_BS:\n  rows: 4\n  cols: 4\n  data: ["+transform+"]\n";
}
int main(int argc,char**argv){
 if(argc!=2)return 2;
 auto root=std::filesystem::temp_directory_path()/("vio-test-"+std::string(argv[1])+"-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
 std::filesystem::create_directories(root/"cam0/data");std::filesystem::create_directories(root/"imu0");
 try {
 std::string test=argv[1];Options o;
 file(root/"camera.yaml",calibration());auto c=loadCalibration(root/"camera.yaml");
 if(test=="parsing") {
  file(root/"imu0/data.csv","# timestamp,wx,wy,wz,ax,ay,az\n1000000000,0,0,0,0,0,9.81\n1005000000,1,2,3,4,5,6\n");
  auto v=loadImu(root);require(v.size()==2&&v[1].gyro.y()==2&&v[0].accel.z()==9.81,"IMU parse values");
  file(root/"cam0/data/a.png","");file(root/"cam0/data.csv","#timestamp,filename\n1000000000,a.png\n");
  require(loadImages(root).size()==1,"image parse");std::filesystem::remove(root/"cam0/data/a.png");rejects([&]{loadImages(root);});
 }else if(test=="ordering") {
  file(root/"imu0/data.csv","20,0,0,0,0,0,9.81\n10,0,0,0,0,0,9.81\n");auto v=loadImu(root);require(v[0].ns==10,"sort");
  file(root/"imu0/data.csv","10,0,0,0,0,0,9.81\n10,0,0,0,0,0,9.81\n");rejects([&]{loadImu(root);});
 }else if(test=="interpolation") {
  std::vector<Imu> v{{1000000000,Vec::Zero(),Vec::Zero()},{1010000000,Vec(2,4,6),Vec(4,6,8)}};
  require((interpolate(v,1005000000).gyro-Vec(1,2,3)).norm()<1e-10,"linear interpolation");
  auto slice=interval(v,1002000000,1008000000,.02);require(slice.size()==2&&slice[0].ns==1002000000,"clipped endpoints");
  rejects([&]{interpolate(v,999999999);});rejects([&]{interval(v,1002000000,1008000000,.005);});
 }else if(test=="invalid") {
  for(auto s:{"10,nan,0,0,0,0,9.81\n20,0,0,0,0,0,9.81\n","10,0,0\n","-1,0,0,0,0,0,9.81\n","10x,0,0,0,0,0,9.81\n"}){file(root/"imu0/data.csv",s);rejects([&]{loadImu(root);});}
  file(root/"cam0/data.csv","10,../bad.png\n");rejects([&]{loadImages(root);});
 }else if(test=="calibration") {
  require(c.width==640&&std::abs(c.t_BC.x()-.1)<1e-12&&c.K.at<double>(0,0)==400,"calibration values");
  file(root/"bad.yaml",calibration("2,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1"));rejects([&]{loadCalibration(root/"bad.yaml");});
  file(root/"bad.yaml","camera_model: fisheye\n");rejects([&]{loadCalibration(root/"bad.yaml");});
 }else if(test=="tracking") {
  cv::Mat blank=cv::Mat::zeros(480,640,CV_8U);require(!track(blank,blank,c,o).valid,"blank rejected");
  require(track(cv::Mat(),blank,c,o).status=="invalid_image","empty rejected");
  // Render nonplanar synthetic 3D points into a translated camera; exercise actual LK/RANSAC.
  cv::Mat a=blank.clone(),b=blank.clone(),small=blank.clone();cv::RNG rng(1337);
  for(int i=0;i<500;i++) {
   double x=rng.uniform(-2.,2.),y=rng.uniform(-1.4,1.4),z=rng.uniform(4.,10.);
   cv::Point p(cvRound(400*x/z+320),cvRound(400*y/z+240));
   cv::Point q(cvRound(400*(x-.18)/z+320),cvRound(400*y/z+240));
   cv::Point near(cvRound(400*(x-.075)/z+320),cvRound(400*y/z+240));
   int shade=rng.uniform(100,256);cv::rectangle(small,cv::Rect(near.x-2,near.y-2,5,5),cv::Scalar(shade),-1);cv::rectangle(a,cv::Rect(p.x-2,p.y-2,5,5),cv::Scalar(shade),-1);cv::rectangle(b,cv::Rect(q.x-2,q.y-2,5,5),cv::Scalar(shade),-1);
  }
  cv::setRNGSeed(42);auto m=track(a,b,c,o);require(m.valid,"synthetic translation failed: "+m.status);require(m.t_21.x()<-.9,"translation direction");
  auto short_step=track(a,small,c,o);require(short_step.valid,"small-baseline translation failed: "+short_step.status);
  auto same=track(a,a,c,o);require(!same.valid,"zero baseline rejected");
 }else if(test=="composition") {
  Mat R=expSO3(Vec(0,0,.4));Vec p(1,2,3);Mat oldR=R;Vec old=p;Motion m;m.valid=true;m.R_21=expSO3(Vec(0,.2,0));m.t_21=Vec::UnitX();
  compose(R,p,m);require((R-oldR*m.R_21.transpose()).norm()<1e-12,"rotation inverse");require((p-(old-R*m.t_21)).norm()<1e-12,"translation inverse");
 }else if(test=="scale") {
  std::vector<ScalePoint> pts;Vec velocity(.3,-.2,.1);
  for(int i=0;i<30;i++){double t=i*.1;Vec q(.4*t*t,.03*t*t*t,0),lever(.001*t,0,0);pts.push_back({t,q,2.5*q-velocity*t-lever,lever});}
  auto fit=fitScale(pts,o);require(fit.valid&&std::abs(fit.value-2.5)<1e-9,"scale and initial velocity fitted");
  pts[12].inertial+=Vec(4,4,4);fit=fitScale(pts,o);require(fit.valid&&std::abs(fit.value-2.5)<1e-8&&fit.used<30,"robust scale fit");
 }else if(test=="degenerate") {
  std::vector<ScalePoint> pts;
  for(int i=0;i<30;i++){double t=.1*i;pts.push_back({t,Vec(t,0,0),Vec(2*t,0,0),Vec::Zero()});}
  require(!fitScale(pts,o).valid,"constant velocity unobservable");
  for(auto&p:pts){p.visual.x()=p.t*p.t;p.inertial.x()=-2*p.t*p.t;}
  require(!fitScale(pts,o).valid,"negative scale rejected");pts[3].t=0;require(!fitScale(pts,o).valid,"time disorder rejected");
 }else if(test=="csv") {
  Row r;r.ns=1000000000;r.status="origin";r.scale.status="window_warming_up";writeRows(root/"out.csv",{r});
  std::ifstream f(root/"out.csv");std::string line;std::getline(f,line);require(std::count(line.begin(),line.end(),',')==20,"header columns");std::getline(f,line);require(std::count(line.begin(),line.end(),',')==20,"row columns");require(line.find("nan")==std::string::npos,"no nan sentinel");
 }else if(test=="imu") {
  State s;std::vector<Imu> v;for(int i=0;i<=100;i++)v.push_back({1000000000+Stamp(i)*10000000,Vec::Zero(),Vec(2,0,9.81)});
  integrate(s,v,o);require(std::abs(s.p.x()-1)<1e-10&&std::abs(s.v.x()-2)<1e-10&&std::abs(s.p.z())<1e-10,"constant acceleration gravity removal");
  s=State();for(auto&p:v){p.gyro=Vec(0,0,1);p.accel=Vec(0,0,9.81);}integrate(s,v,o);
  require((s.R_WB-expSO3(Vec(0,0,1))).norm()<1e-10,"gyroscope integration");
 }else if(test=="initialization") {
  std::vector<Imu> v;for(int i=0;i<=100;i++)v.push_back({1000000000+Stamp(i)*5000000,Vec(.01,0,0),Vec(0,9.81,0)});
  auto init=initialize(v,o);require(init.valid&&(init.R_WB*Vec(0,9.81,0)-Vec(0,0,9.81)).norm()<1e-8,"gravity alignment");
  v[1].accel=Vec(10,0,0);require(!initialize(v,o).valid,"dynamic initialization rejected");
 }else throw std::runtime_error("unknown test");
 std::filesystem::remove_all(root);std::cout<<test<<": PASS\n";return 0;
 }catch(const std::exception&e){std::cerr<<argv[1]<<": FAIL: "<<e.what()<<'\n';std::filesystem::remove_all(root);return 1;}
}
