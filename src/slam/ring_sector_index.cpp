#include "autonomy_light/slam/ring_sector_index.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pcl/common/point_tests.h>

namespace autonomy_light::slam
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

RingSectorIndex::RingSectorIndex(const int rings, const int sectors, const double max_radius)
: rings_(rings), sectors_(sectors), max_radius_(max_radius)
{
  if (rings_ <= 0 || sectors_ <= 0 || max_radius_ <= 0.0) {
    throw std::invalid_argument("ring-sector dimensions must be positive");
  }
}

void RingSectorIndex::add(const Cloud & cloud)
{
  entries_.push_back(makeEntry(cloud));
}

std::optional<PlaceCandidate> RingSectorIndex::findCandidate(
  const Cloud & cloud, const std::size_t exclude_recent, const std::size_t candidate_count,
  const double max_distance) const
{
  if (candidate_count == 0U || entries_.size() <= exclude_recent) {
    return std::nullopt;
  }
  const Entry query = makeEntry(cloud);
  const std::size_t limit = entries_.size() - exclude_recent;
  std::vector<std::pair<float, std::size_t>> ring_candidates;
  ring_candidates.reserve(limit);
  for (std::size_t id = 0; id < limit; ++id) {
    ring_candidates.emplace_back((entries_[id].ring_key - query.ring_key).squaredNorm(), id);
  }
  const std::size_t count = std::min(candidate_count, ring_candidates.size());
  std::partial_sort(
    ring_candidates.begin(), ring_candidates.begin() + static_cast<std::ptrdiff_t>(count),
    ring_candidates.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.first < rhs.first;});

  PlaceCandidate best{0U, std::numeric_limits<double>::infinity()};
  for (std::size_t rank = 0; rank < count; ++rank) {
    const std::size_t id = ring_candidates[rank].second;
    const double distance = descriptorDistance(entries_[id], query);
    if (distance < best.distance) {
      best = {id, distance};
    }
  }
  return best.distance <= max_distance ? std::optional<PlaceCandidate>(best) : std::nullopt;
}

RingSectorIndex::Entry RingSectorIndex::makeEntry(const Cloud & cloud) const
{
  Entry entry{Eigen::MatrixXf::Zero(rings_, sectors_), Eigen::VectorXf::Zero(rings_)};
  for (const auto & point : cloud.points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const double radius = std::hypot(point.x, point.y);
    if (radius < 0.1 || radius >= max_radius_) {
      continue;
    }
    const int ring = std::min(rings_ - 1, static_cast<int>(radius / max_radius_ * rings_));
    const double angle = std::atan2(point.y, point.x) + kPi;
    const int sector = std::min(sectors_ - 1, static_cast<int>(angle / (2.0 * kPi) * sectors_));
    entry.descriptor(ring, sector) = std::max(entry.descriptor(ring, sector), point.z + 2.0F);
  }
  entry.ring_key = entry.descriptor.rowwise().mean();
  return entry;
}

double RingSectorIndex::descriptorDistance(const Entry & lhs, const Entry & rhs) const
{
  double best = std::numeric_limits<double>::infinity();
  for (int shift = 0; shift < sectors_; ++shift) {
    double cosine_sum = 0.0;
    int compared = 0;
    for (int ring = 0; ring < rings_; ++ring) {
      const Eigen::VectorXf lhs_row = lhs.descriptor.row(ring).transpose();
      Eigen::VectorXf rhs_row(sectors_);
      for (int sector = 0; sector < sectors_; ++sector) {
        rhs_row(sector) = rhs.descriptor(ring, (sector + shift) % sectors_);
      }
      const double denominator = lhs_row.norm() * rhs_row.norm();
      if (denominator > 1e-6) {
        cosine_sum += lhs_row.dot(rhs_row) / denominator;
        ++compared;
      }
    }
    if (compared > 0) {
      best = std::min(best, 1.0 - cosine_sum / static_cast<double>(compared));
    }
  }
  return std::isfinite(best) ? best : 1.0;
}

}  // namespace autonomy_light::slam
