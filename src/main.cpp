#include "vio/core.hpp"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <deque>
#include <stdexcept>
using namespace vio;
namespace {
void checkImuFrame(const std::filesystem::path&path) {
 std::ifstream f(path);if(!f)throw std::runtime_error("Missing imu0/sensor.yaml");std::string s((std::istreambuf_iterator<char>(f)),{});
 if(s.rfind("%YAML",0)!=0)s="%YAML:1.0\n---\n"+s;
 cv::FileStorage fs(s,cv::FileStorage::READ|cv::FileStorage::MEMORY);
 std::vector<double> data;auto t=fs["T_BS"];t["data"]>>data;
 if(data.size()!=16||(int)t["rows"]!=4||(int)t["cols"]!=4)throw std::runtime_error("Invalid IMU extrinsics");
 for(int i=0;i<16;i++)if(!std::isfinite(data[i])||std::abs(data[i]-(i%5==0?1.:0.))>1e-7)
 throw std::runtime_error("This prototype requires EuRoC body frame to equal imu0 frame (identity imu0 T_BS)");
}
}
int main(int argc,char**argv) {
 try {
 if(argc==2&&std::string(argv[1])=="--help") {
  std::cout<<"vio_lite --dataset PATH/mav0 --output out [--config config/default.yaml] [--start-frame 0] [--max-frames 300]\n";return 0;
 }
 std::filesystem::path root,out="output",config="config/default.yaml";int start=0,limit=300;
 for(int i=1;i<argc;i+=2) {
  if(i+1>=argc)throw std::runtime_error("Options require values; use --help");std::string key=argv[i],v=argv[i+1];
  if(key=="--dataset")root=v;else if(key=="--output")out=v;else if(key=="--config")config=v;
  else if(key=="--start-frame"||key=="--max-frames") {size_t used;int n=std::stoi(v,&used);if(used!=v.size())throw std::runtime_error("Invalid integer");if(key=="--start-frame")start=n;else limit=n;}
  else throw std::runtime_error("Unknown option "+key);
 }
 if(root.empty()||start<0||limit<2)throw std::runtime_error("Dataset required; start >=0 and max-frames >=2");
 cv::setNumThreads(1);cv::setRNGSeed(42);
 Options options=loadOptions(config);Calibration cam=loadCalibration(root/"cam0/sensor.yaml");checkImuFrame(root/"imu0/sensor.yaml");
 auto images=loadImages(root);auto imu=loadImu(root);
 if(size_t(start)>=images.size())throw std::runtime_error("Start frame out of range");
 Stamp init_start=images[start].ns,init_end=init_start+Stamp(options.init_seconds*1e9);
 auto init_samples=interval(imu,init_start,init_end,options.max_imu_gap);auto init=initialize(init_samples,options);
 // Initialization consumes a prefix, never a future interval used by the estimator.
 while(size_t(start)<images.size()&&images[start].ns<init_end)++start;
 if(size_t(start+1)>=images.size())throw std::runtime_error("No images after initialization");
 std::filesystem::create_directories(out);
 State state;state.R_WB=init.R_WB;if(init.valid)options.gyro_bias=init.gyro_bias;
 if(images[start].ns>init_end&&init.valid)integrate(state,interval(imu,init_end,images[start].ns,options.max_imu_gap),options);
 Mat R_WC=state.R_WB*cam.R_BC;Vec raw=Vec::Zero();
 cv::Mat previous=cv::imread(images[start].path.string(),cv::IMREAD_GRAYSCALE);
 if(previous.empty()||previous.cols!=cam.width||previous.rows!=cam.height)throw std::runtime_error("Invalid first image");
 struct Point {Stamp ns;Vec raw,p,v;Mat R;};std::deque<Point> window;
 window.push_back({images[start].ns,raw,state.p,state.v,state.R_WB});
 std::vector<Row> rows;Row first;first.ns=images[start].ns;first.status="origin";first.scale.status=init.status;rows.push_back(first);
 std::vector<double> scales;bool connected=true,imu_ok=init.valid;int failures=0,tracked=0,valid_windows=0;
 auto wall=std::chrono::steady_clock::now();
 int end=start+int(std::min(images.size()-size_t(start),size_t(limit)));
 for(int i=start+1;i<end;i++) {
  auto tick=std::chrono::steady_clock::now();Row row;row.ns=images[i].ns;
  cv::Mat current=cv::imread(images[i].path.string(),cv::IMREAD_GRAYSCALE);
  Motion motion=track(previous,current,cam,options);row.tracking=motion.valid;row.status=motion.status;
  row.matches=motion.matches;row.inliers=motion.inliers;row.parallax=motion.parallax;
  Mat lastR=state.R_WB;
  if(imu_ok) {
   try {integrate(state,interval(imu,images[i-1].ns,images[i].ns,options.max_imu_gap),options);}
   catch(const std::exception&){imu_ok=false;window.clear();row.scale.status="imu_gap";}
  }
  if(motion.valid) {
   compose(R_WC,raw,motion);++tracked;
   // Camera rotation from the IMU must agree with the visual relative rotation.
   Mat gyroR=cam.R_BC.transpose()*state.R_WB.transpose()*lastR*cam.R_BC;
   double rotation_error=Eigen::AngleAxisd(gyroR*motion.R_21.transpose()).angle();
   if(imu_ok&&rotation_error<.15) {
    window.push_back({row.ns,raw,state.p,state.v,state.R_WB});
    while(window.size()>size_t(options.window))window.pop_front();
    if(window.size()==size_t(options.window)) {
     auto origin=window.front();std::vector<ScalePoint> pts;
     for(auto&p:window) {
      double t=double(p.ns-origin.ns)*1e-9;
      pts.push_back({t,p.raw-origin.raw,p.p-origin.p-origin.v*t,(p.R-origin.R)*cam.t_BC});
     }
     row.scale=fitScale(pts,options);
     if(row.scale.valid){scales.push_back(row.scale.value);++valid_windows;}
    }else row.scale.status="window_warming_up";
   }else {window.clear();if(imu_ok)row.scale.status="gyro_visual_rotation_disagreement";}
  }else {++failures;connected=false;window.clear();}
  row.raw=raw;row.connected=connected;
  if(!init.valid)row.scale.status=init.status;else if(!imu_ok)row.scale.status="imu_unavailable";
  if(row.scale.status.empty())row.scale.status="tracking_unavailable";
  row.runtime_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();rows.push_back(row);previous=current;
 }
 double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-wall).count();
 double scale=0,spread=0;std::string metric_status="insufficient_scale_windows";
 // A conservative retrospective constant scale; accepted windows are correlated, not independent trials.
 if(scales.size()>=3) {
  std::sort(scales.begin(),scales.end());scale=scales[scales.size()/2];std::vector<double> deviations;
  for(double s:scales)deviations.push_back(std::abs(s-scale));std::sort(deviations.begin(),deviations.end());spread=deviations[deviations.size()/2]/scale;
  metric_status=spread>.25?"unstable_scale":(!connected?"disconnected_visual_trajectory":(!imu_ok?"imu_unavailable":"experimental_metric_estimate"));
  if(metric_status=="experimental_metric_estimate")for(auto&r:rows){r.metric_valid=true;r.metric=r.raw*scale;}
 }
 writeRows(out/"trajectory.csv",rows);
 std::ofstream summary(out/"run_summary.csv");summary<<std::setprecision(17);
 summary<<"processed_frames,valid_tracking_frames,failed_tracking_frames,valid_scale_windows,elapsed_seconds,fps,initialization_status,metric_status,global_scale,relative_scale_mad,initialization_frames_skipped\n";
 summary<<rows.size()<<','<<tracked<<','<<failures<<','<<valid_windows<<','<<elapsed<<','<<(rows.size()-1)/elapsed<<','<<init.status<<','<<metric_status<<',';
 if(metric_status=="experimental_metric_estimate")summary<<scale;
 summary<<','<<spread<<','<<(start-int(std::lower_bound(images.begin(),images.end(),init_start,[](const Image&a,Stamp t){return a.ns<t;})-images.begin()))<<'\n';
 if(!summary)throw std::runtime_error("Summary write failed");
 std::cout<<"Processed "<<rows.size()<<" frames; tracked "<<tracked<<"; failures "<<failures<<"; scale windows "<<valid_windows<<"; "<<metric_status<<"\n";
 return 0;
 }catch(const std::exception&e){std::cerr<<"VIO-Lite: "<<e.what()<<'\n';return 1;}
}
