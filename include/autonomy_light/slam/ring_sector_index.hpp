#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "autonomy_light/slam/keyframe.hpp"

namespace autonomy_light::slam
{

struct PlaceCandidate
{
  std::size_t keyframe_id{};
  double distance{};
};

// Independent ring/sector place recognition. This is intentionally not a copy
// of the CC-BY-NC-SA Scan Context++ implementation.
class RingSectorIndex final
{
public:
  RingSectorIndex(int rings, int sectors, double max_radius);

  void add(const Cloud & cloud);
  [[nodiscard]] std::optional<PlaceCandidate> findCandidate(
    const Cloud & cloud, std::size_t exclude_recent, std::size_t candidate_count,
    double max_distance) const;

private:
  struct Entry
  {
    Eigen::MatrixXf descriptor;
    Eigen::VectorXf ring_key;
  };

  [[nodiscard]] Entry makeEntry(const Cloud & cloud) const;
  [[nodiscard]] double descriptorDistance(const Entry & lhs, const Entry & rhs) const;

  int rings_;
  int sectors_;
  double max_radius_;
  std::vector<Entry> entries_;
};

}  // namespace autonomy_light::slam
