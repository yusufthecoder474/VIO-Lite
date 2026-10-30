#include "vio/core.hpp"
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace vio {
void writeRows(const std::filesystem::path& path,const std::vector<Row>& rows) {
 std::ofstream f(path);if(!f)throw std::runtime_error("Cannot create "+path.string());f<<std::setprecision(17);
 f<<"timestamp_ns,raw_x,raw_y,raw_z,metric_x,metric_y,metric_z,metric_valid,tracking_valid,connected,matches,inliers,parallax_px,runtime_ms,scale,scale_valid,scale_residual,scale_excitation,scale_used,status,scale_status\n";
 for(auto&r:rows) {
  if(!r.raw.allFinite()||!std::isfinite(r.runtime_ms)||r.runtime_ms<0||(r.metric_valid&&!r.metric.allFinite()))throw std::runtime_error("Invalid output row");
  f<<r.ns<<','<<r.raw.x()<<','<<r.raw.y()<<','<<r.raw.z()<<',';
  if(r.metric_valid)f<<r.metric.x()<<','<<r.metric.y()<<','<<r.metric.z();else f<<",,";
  f<<','<<r.metric_valid<<','<<r.tracking<<','<<r.connected<<','<<r.matches<<','<<r.inliers<<','<<r.parallax<<','<<r.runtime_ms<<',';
  if(r.scale.valid)f<<r.scale.value;
  f<<','<<r.scale.valid<<','<<r.scale.residual<<','<<r.scale.excitation<<','<<r.scale.used<<','<<r.status<<','<<r.scale.status<<'\n';
 }
 if(!f)throw std::runtime_error("CSV write failed");
}
}
