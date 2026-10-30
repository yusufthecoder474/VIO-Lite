#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <cstdint>

namespace vio {
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Stamp = std::int64_t;
struct Image { Stamp ns; std::filesystem::path path; };
struct Imu { Stamp ns; Vec gyro, accel; };
struct Calibration {
  cv::Mat K, distortion;
  Mat R_BC=Mat::Identity(); Vec t_BC=Vec::Zero();
  int width=0, height=0;
};
struct Options {
  double init_seconds=.5, max_imu_gap=.02, min_parallax=1.0;
  double max_scale_residual=.35, min_excitation=.02;
  int window=20, min_matches=20;
  Vec gyro_bias=Vec::Zero(), accel_bias=Vec::Zero();
};
Options loadOptions(const std::filesystem::path&);
Calibration loadCalibration(const std::filesystem::path&);
std::vector<Image> loadImages(const std::filesystem::path& root);
std::vector<Imu> loadImu(const std::filesystem::path& root);
Imu interpolate(const std::vector<Imu>&, Stamp);
std::vector<Imu> interval(const std::vector<Imu>&, Stamp, Stamp, double max_gap);
Mat expSO3(const Vec&);
struct State { Mat R_WB=Mat::Identity(); Vec p=Vec::Zero(), v=Vec::Zero(); };
struct Initialization { bool valid=false; Mat R_WB=Mat::Identity(); Vec gyro_bias=Vec::Zero(); std::string status; };
Initialization initialize(const std::vector<Imu>&, const Options&);
void integrate(State&, const std::vector<Imu>&, const Options&);
struct Motion {
  bool valid=false; Mat R_21=Mat::Identity(); Vec t_21=Vec::Zero();
  int matches=0, inliers=0; double parallax=0; std::string status="not_run";
};
Motion track(const cv::Mat&,const cv::Mat&, const Calibration&,const Options&);
void compose(Mat& R_WC, Vec& p_WC, const Motion&);
struct ScalePoint { double t; Vec visual, inertial, lever; };
struct Scale { bool valid=false; double value=0, residual=0, excitation=0; int used=0; std::string status; };
Scale fitScale(const std::vector<ScalePoint>&, const Options&);
struct Row {
  Stamp ns=0; Vec raw=Vec::Zero(), metric=Vec::Zero();
  bool tracking=false, metric_valid=false, connected=true;
  int matches=0,inliers=0; double parallax=0, runtime_ms=0;
  Scale scale; std::string status;
};
void writeRows(const std::filesystem::path&,const std::vector<Row>&);
}
