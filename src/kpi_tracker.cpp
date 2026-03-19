#include "nav2_ground_consistency_costmap_plugin/kpi_tracker.hpp"
#include <iomanip>
#include "rclcpp/rclcpp.hpp"

namespace nav2_ground_consistency_costmap_plugin
{

KPITracker::KPITracker(const std::string & output_path)
: output_path_(output_path), file_(output_path, std::ios::app)
{
  if (!file_.is_open()) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("KPITracker"),
      "Failed to open KPI file: " << output_path);
  } else {
    writeHeader();
  }
}

KPITracker::~KPITracker()
{
  if (file_.is_open()) {
    file_.close();
  }
}

void KPITracker::writeHeader()
{
  std::lock_guard<std::mutex> lock(file_mutex_);
  if (header_written_) {
    return;
  }

  file_ << "timestamp,update_latency_ms,cells_updated,cells_decayed,"
        << "total_ground_cells,total_nonground_cells,memory_usage_mb,"
        << "ground_points,nonground_points\n";
  file_.flush();
  header_written_ = true;
}

void KPITracker::recordSnapshot(const KPISnapshot & snapshot)
{
  if (!file_.is_open()) {
    return;
  }

  std::lock_guard<std::mutex> lock(file_mutex_);

  auto time_t = std::chrono::system_clock::to_time_t(snapshot.timestamp);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    snapshot.timestamp.time_since_epoch()) %
    1000;

  file_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
        << "." << std::setfill('0') << std::setw(3) << ms.count() << ","
        << std::fixed << std::setprecision(3) << snapshot.update_latency_ms << ","
        << snapshot.cells_updated << ","
        << snapshot.cells_decayed << ","
        << snapshot.total_ground_cells << ","
        << snapshot.total_nonground_cells << ","
        << snapshot.memory_usage_mb << ","
        << snapshot.ground_points_processed << ","
        << snapshot.nonground_points_processed << "\n";

  file_.flush();
}

void KPITracker::startUpdateTimer()
{
  update_start_ = std::chrono::high_resolution_clock::now();
}

double KPITracker::stopUpdateTimer()
{
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    end - update_start_);
  return duration.count();
}

}  // namespace nav2_ground_consistency_costmap_plugin
