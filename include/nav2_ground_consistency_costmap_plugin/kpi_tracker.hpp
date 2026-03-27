#pragma once

#include <chrono>
#include <fstream>
#include <string>
#include <mutex>

namespace nav2_ground_consistency_costmap_plugin
{

struct KPISnapshot
{
  std::chrono::system_clock::time_point timestamp;
  double total_cycle_latency_ms{0.0};    // Total time: updateBounds + updateCosts
  uint32_t cells_updated{0};             // Number of cells modified
  uint32_t cells_decayed{0};             // Number of cells that decayed
  size_t total_ground_cells{0};          // Total ground score map size
  size_t total_nonground_cells{0};       // Total nonground score map size
  double memory_usage_mb{0.0};           // Estimated memory usage
  uint32_t ground_points_processed{0};   // Points in last update
  uint32_t nonground_points_processed{0};
};

class KPITracker
{
public:
  explicit KPITracker(const std::string & output_path = "/tmp/costmap_kpi.csv");
  ~KPITracker();

  void recordSnapshot(const KPISnapshot & snapshot);
  void startTimer();                    // Start cycle timer (call in updateBounds)
  double getTotalTime();                // Get total time since startTimer (call in updateCosts)

  KPISnapshot & currentSnapshot() { return current_snapshot_; }

private:
  void writeHeader();

  std::string output_path_;
  std::ofstream file_;
  std::mutex file_mutex_;
  KPISnapshot current_snapshot_;
  std::chrono::high_resolution_clock::time_point cycle_start_;
  bool header_written_{false};
};

}  // namespace nav2_ground_consistency_costmap_plugin
