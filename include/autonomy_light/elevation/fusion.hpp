#pragma once

#include "autonomy_light/elevation/grid.hpp"

namespace autonomy_light::elevation
{

struct FusionConfig
{
  std::size_t live_min_support{3U};
  double live_variance_floor{0.000025};
  double prior_variance_floor{0.0004};
  double agreement_sigma{2.5};
};

class GridFusion final
{
public:
  explicit GridFusion(FusionConfig config);
  [[nodiscard]] ElevationGrid fuse(const ElevationGrid & live, const ElevationGrid & prior) const;

private:
  [[nodiscard]] double confidenceVariance(const GridCell & cell, bool live) const;

  FusionConfig config_;
};

}  // namespace autonomy_light::elevation
