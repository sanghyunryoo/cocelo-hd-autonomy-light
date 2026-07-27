#include "autonomy_light/elevation/fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy_light::elevation
{

GridFusion::GridFusion(FusionConfig config)
: config_(config)
{
}

ElevationGrid GridFusion::fuse(const ElevationGrid & live, const ElevationGrid & prior) const
{
  if (live.size() != prior.size()) {
    return {};
  }
  ElevationGrid result(live.size());
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto & live_cell = live[index];
    const auto & prior_cell = prior[index];
    const bool live_reliable = live_cell.valid() && live_cell.support >= config_.live_min_support;
    if (!live_reliable) {
      result[index] = prior_cell;
      continue;
    }
    if (!prior_cell.valid()) {
      result[index] = live_cell;
      continue;
    }
    const double live_variance = confidenceVariance(live_cell, true);
    const double prior_variance = confidenceVariance(prior_cell, false);
    const double innovation = static_cast<double>(live_cell.z) - prior_cell.z;
    const double gate = config_.agreement_sigma * std::sqrt(live_variance + prior_variance);
    if (std::abs(innovation) > gate) {
      result[index] = live_cell;
      continue;
    }
    const double live_weight = 1.0 / live_variance;
    const double prior_weight = 1.0 / prior_variance;
    result[index].z = static_cast<float>((live_weight * live_cell.z + prior_weight * prior_cell.z) /
      (live_weight + prior_weight));
    result[index].variance = static_cast<float>(1.0 / (live_weight + prior_weight));
    result[index].support = live_cell.support + prior_cell.support;
  }
  return result;
}

double GridFusion::confidenceVariance(const GridCell & cell, const bool live) const
{
  const double floor = live ? config_.live_variance_floor : config_.prior_variance_floor;
  const double support = static_cast<double>(std::max<std::size_t>(cell.support, 1U));
  const double measured = std::isfinite(cell.variance) ? cell.variance : floor;
  return std::max(measured / support, floor);
}

}  // namespace autonomy_light::elevation
