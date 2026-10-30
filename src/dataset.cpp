#include "vio/core.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <cmath>
namespace vio {
namespace {
std::vector<std::vector<std::string>> csv(const std::filesystem::path& path, size_t cols) {
 std::ifstream f(path); if(!f) throw std::runtime_error("Cannot open "+path.string());
 std::vector<std::vector<std::string>> rows; std::string line;
 while(std::getline(f,line)) {
  if(line.empty() || line[0]=='#' || line=="\r") continue;
  std::stringstream ss(line); std::string cell; std::vector<std::string> r;
  while(std::getline(ss,cell,',')) { if(!cell.empty()&&cell.back()=='\r') cell.pop_back(); r.push_back(cell); }
  if(r.size()!=cols) throw std::runtime_error("Wrong CSV column count in "+path.string());
  rows.push_back(r);
 } if(rows.empty()) throw std::runtime_error("Empty CSV "+path.string()); return rows;
}
Stamp stamp(const std::string& s) {
 size_t n=0; auto t=std::stoll(s,&n); if(n!=s.size()||t<=0) throw std::runtime_error("Invalid nanosecond timestamp"); return t;
}
double number(const std::string& s) {
 size_t n=0; double d=std::stod(s,&n); if(n!=s.size()||!std::isfinite(d)) throw std::runtime_error("Invalid sensor number"); return d;
}
template<class T> void sortValidate(std::vector<T>& values) {
 std::sort(values.begin(),values.end(),[](const T&a,const T&b){return a.ns<b.ns;});
 for(size_t i=1;i<values.size();++i) if(values[i].ns==values[i-1].ns) throw std::runtime_error("Duplicate timestamp");
}
// EuRoC sensor.yaml uses plain YAML 1.0. OpenCV requires its own header.
cv::FileStorage yaml(const std::filesystem::path& path) {
 std::ifstream f(path); if(!f) throw std::runtime_error("Cannot open calibration/config "+path.string());
 std::stringstream text; text<<f.rdbuf(); std::string s=text.str();
 if(s.rfind("%YAML",0)!=0) s="%YAML:1.0\n---\n"+s;
 cv::FileStorage fs(s,cv::FileStorage::READ|cv::FileStorage::MEMORY);
 if(!fs.isOpened()) throw std::runtime_error("Invalid YAML");
 return fs;
}
Vec vec(const cv::FileNode& n) {
 if(n.type()!=cv::FileNode::SEQ||n.size()!=3) throw std::runtime_error("Expected three bias values");
 Vec v; for(int i=0;i<3;i++) {v[i]=(double)n[i]; if(!std::isfinite(v[i])) throw std::runtime_error("Invalid bias");} return v;
}
}
Options loadOptions(const std::filesystem::path& path) {
 auto fs=yaml(path); Options o;
 auto d=[&](const char* key,double& v){if(!fs[key].empty())v=(double)fs[key];};
 d("init_seconds",o.init_seconds); d("max_imu_gap",o.max_imu_gap); d("min_parallax",o.min_parallax);
 d("max_scale_residual",o.max_scale_residual); d("min_excitation",o.min_excitation);
 if(!fs["window"].empty())o.window=(int)fs["window"];
 if(!fs["min_matches"].empty())o.min_matches=(int)fs["min_matches"];
 if(!fs["gyro_bias"].empty())o.gyro_bias=vec(fs["gyro_bias"]);
 if(!fs["accel_bias"].empty())o.accel_bias=vec(fs["accel_bias"]);
 if(!std::isfinite(o.init_seconds)||o.init_seconds<.2||o.init_seconds>60||!std::isfinite(o.max_imu_gap)||o.max_imu_gap<=0 ||
 !std::isfinite(o.min_parallax)||o.min_parallax<=0||!std::isfinite(o.max_scale_residual)||o.max_scale_residual<=0||
 !std::isfinite(o.min_excitation)||o.min_excitation<=0||o.window<6||o.window>500||o.min_matches<8)
 throw std::runtime_error("Invalid options");
 return o;
}
Calibration loadCalibration(const std::filesystem::path& path) {
 auto fs=yaml(path); Calibration c;
 std::string model,dist; fs["camera_model"]>>model; fs["distortion_model"]>>dist;
 if(model!="pinhole"||dist!="radial-tangential")throw std::runtime_error("Only EuRoC pinhole radtan cameras supported");
 std::vector<double> intr,coeff; std::vector<int> res;
 fs["intrinsics"]>>intr; fs["distortion_coefficients"]>>coeff; fs["resolution"]>>res;
 if(intr.size()!=4||coeff.size()!=4||res.size()!=2)throw std::runtime_error("Incomplete camera calibration");
 for(double v:intr)if(!std::isfinite(v))throw std::runtime_error("Nonfinite intrinsics");
 for(double v:coeff)if(!std::isfinite(v))throw std::runtime_error("Nonfinite distortion");
 if(intr[0]<=0||intr[1]<=0||res[0]<=0||res[1]<=0)throw std::runtime_error("Invalid intrinsics/resolution");
 c.K=(cv::Mat_<double>(3,3)<<intr[0],0,intr[2],0,intr[1],intr[3],0,0,1);
 c.distortion=cv::Mat(coeff).clone(); c.width=res[0];c.height=res[1];
 auto t=fs["T_BS"]; std::vector<double> data; t["data"]>>data;
 if((int)t["rows"]!=4||(int)t["cols"]!=4||data.size()!=16)throw std::runtime_error("T_BS must be 4x4");
 for(double v:data)if(!std::isfinite(v))throw std::runtime_error("Nonfinite T_BS");
 for(int r=0;r<3;r++){for(int col=0;col<3;col++)c.R_BC(r,col)=data[4*r+col]; c.t_BC[r]=data[4*r+3];}
 if((c.R_BC.transpose()*c.R_BC-Mat::Identity()).norm()>1e-5||std::abs(c.R_BC.determinant()-1)>1e-5||
 std::abs(data[12])+std::abs(data[13])+std::abs(data[14])+std::abs(data[15]-1)>1e-8)
 throw std::runtime_error("T_BS is not a rigid transform");
 return c;
}
std::vector<Image> loadImages(const std::filesystem::path& root) {
 std::vector<Image> out;
 for(auto&r:csv(root/"cam0/data.csv",2)) {
  std::filesystem::path rel(r[1]);
  if(rel.is_absolute()||rel.has_parent_path())throw std::runtime_error("Image filename must be a basename");
  auto p=root/"cam0/data"/rel;
  if(!std::filesystem::is_regular_file(p))throw std::runtime_error("Missing image "+p.string());
  out.push_back({stamp(r[0]),p});
 } sortValidate(out); return out;
}
std::vector<Imu> loadImu(const std::filesystem::path& root) {
 std::vector<Imu> out;
 for(auto&r:csv(root/"imu0/data.csv",7))out.push_back({stamp(r[0]),Vec(number(r[1]),number(r[2]),number(r[3])),Vec(number(r[4]),number(r[5]),number(r[6]))});
 sortValidate(out); if(out.size()<2)throw std::runtime_error("Need at least two IMU samples");return out;
}
Imu interpolate(const std::vector<Imu>& data,Stamp ns) {
 if(data.size()<2||ns<data.front().ns||ns>data.back().ns)throw std::runtime_error("IMU does not bracket timestamp");
 auto b=std::lower_bound(data.begin(),data.end(),ns,[](const Imu&a,Stamp t){return a.ns<t;});
 if(b->ns==ns)return *b;
 auto a=b-1;double u=double(ns-a->ns)/double(b->ns-a->ns);
 return {ns,a->gyro*(1-u)+b->gyro*u,a->accel*(1-u)+b->accel*u};
}
std::vector<Imu> interval(const std::vector<Imu>& data,Stamp a,Stamp b,double max_gap) {
 if(b<=a)throw std::runtime_error("Nonpositive image interval");
 auto first=interpolate(data,a),last=interpolate(data,b);
 auto lo=std::lower_bound(data.begin(),data.end(),a,[](const Imu&x,Stamp t){return x.ns<t;});
 if(lo!=data.begin()&&lo->ns!=a)--lo;
 for(auto i=lo;i+1!=data.end()&&i->ns<b;++i)
  if(double((i+1)->ns-i->ns)*1e-9>max_gap)throw std::runtime_error("IMU synchronization gap");
 std::vector<Imu> out{first};
 for(auto it=std::upper_bound(data.begin(),data.end(),a,[](Stamp t,const Imu&x){return t<x.ns;});it!=data.end()&&it->ns<b;++it)out.push_back(*it);
 out.push_back(last); return out;
}
}
