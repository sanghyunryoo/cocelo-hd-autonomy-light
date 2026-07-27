#pragma once

#include <vector>

#include "autonomy_light/elevation/grid.hpp"

namespace autonomy_light::elevation
{

struct GridFilterConfig
{
  int min_points_per_cell{3};
  double support_band{0.025};
  int isolated_radius{2};
  int isolated_min_neighbors{5};
  double isolated_support_height_diff{0.02};
  double isolated_outlier_height_diff{0.025};
  int hole_radius{2};
  int hole_min_neighbors{5};
  double hole_max_height_diff{0.02};
  int bilateral_radius{2};
  double bilateral_sigma_spatial{1.3};
  double bilateral_sigma_height{0.012};
  double bilateral_max_height_diff{0.025};
  int bilateral_passes{2};
};

class GridBuilder final
{
public:
  GridBuilder(GridGeometry geometry, GridFilterConfig filter_config);

  [[nodiscard]] ElevationGrid build(const std::vector<Eigen::Vector3f> & points) const;
  [[nodiscard]] const GridGeometry & geometry() const {return geometry_;}

private:
  [[nodiscard]] GridCell selectHighestSupported(std::vector<float> & samples) const;
  void filter(ElevationGrid & grid) const;

  GridGeometry geometry_;
  GridFilterConfig config_;
};

}  // namespace autonomy_light::elevation
