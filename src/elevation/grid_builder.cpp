#include "autonomy_light/elevation/grid_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy_light::elevation
{

namespace
{
std::vector<float> neighbors(
  const ElevationGrid & grid, const int row, const int column, const int radius,
  const std::size_t width, const std::size_t height)
{
  std::vector<float> result;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int neighbor_row = row + dy;
      const int neighbor_column = column + dx;
      if ((dx == 0 && dy == 0) || neighbor_row < 0 || neighbor_column < 0 ||
        neighbor_row >= static_cast<int>(height) || neighbor_column >= static_cast<int>(width))
      {
        continue;
      }
      const auto & cell = grid[static_cast<std::size_t>(neighbor_row) * width + neighbor_column];
      if (cell.valid()) {
        result.push_back(cell.z);
      }
    }
  }
  return result;
}

float median(std::vector<float> values)
{
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}
}  // namespace

GridBuilder::GridBuilder(GridGeometry geometry, GridFilterConfig filter_config)
: geometry_(geometry), config_(filter_config)
{
}

ElevationGrid GridBuilder::build(const std::vector<Eigen::Vector3f> & points) const
{
  std::vector<std::vector<float>> samples(geometry_.size());
  for (const auto & point : points) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()) ||
      !geometry_.contains(point))
    {
      continue;
    }
    samples[geometry_.index(point)].push_back(point.z());
  }

  ElevationGrid grid(geometry_.size());
  for (std::size_t index = 0; index < samples.size(); ++index) {
    grid[index] = selectHighestSupported(samples[index]);
  }
  filter(grid);
  return grid;
}

GridCell GridBuilder::selectHighestSupported(std::vector<float> & samples) const
{
  if (static_cast<int>(samples.size()) < config_.min_points_per_cell) {
    return {};
  }
  std::sort(samples.begin(), samples.end());
  for (std::size_t end = samples.size(); end > 0U; --end) {
    const float upper = samples[end - 1U];
    const auto begin = std::lower_bound(samples.begin(), samples.begin() + end, upper - config_.support_band);
    const auto support = static_cast<std::size_t>(std::distance(begin, samples.begin() + end));
    if (static_cast<int>(support) < config_.min_points_per_cell) {
      continue;
    }
    float sum = 0.0F;
    for (auto value = begin; value != samples.begin() + end; ++value) {
      sum += *value;
    }
    const float mean = sum / static_cast<float>(support);
    float squared_error = 0.0F;
    for (auto value = begin; value != samples.begin() + end; ++value) {
      const float error = *value - mean;
      squared_error += error * error;
    }
    return {mean, squared_error / static_cast<float>(support), support};
  }
  return {};
}

void GridBuilder::filter(ElevationGrid & grid) const
{
  const std::size_t width = geometry_.width();
  const std::size_t height = geometry_.height();
  if (config_.isolated_radius > 0 && config_.isolated_min_neighbors > 0) {
    auto filtered = grid;
    for (int row = 0; row < static_cast<int>(height); ++row) {
      for (int column = 0; column < static_cast<int>(width); ++column) {
        const std::size_t index = static_cast<std::size_t>(row) * width + column;
        if (!grid[index].valid()) {continue;}
        const auto nearby = neighbors(grid, row, column, config_.isolated_radius, width, height);
        const int support = static_cast<int>(std::count_if(nearby.begin(), nearby.end(), [this, &grid, index](float z) {
          return std::abs(z - grid[index].z) <= config_.isolated_support_height_diff;
        }));
        const float neighborhood_median = median(nearby);
        if (support < config_.isolated_min_neighbors && std::isfinite(neighborhood_median) &&
          std::abs(grid[index].z - neighborhood_median) >= config_.isolated_outlier_height_diff)
        {
          filtered[index].z = neighborhood_median;
        }
      }
    }
    grid.swap(filtered);
  }

  if (config_.hole_radius > 0 && config_.hole_min_neighbors > 0) {
    auto filled = grid;
    for (int row = 0; row < static_cast<int>(height); ++row) {
      for (int column = 0; column < static_cast<int>(width); ++column) {
        const std::size_t index = static_cast<std::size_t>(row) * width + column;
        if (grid[index].valid()) {continue;}
        const auto nearby = neighbors(grid, row, column, config_.hole_radius, width, height);
        if (static_cast<int>(nearby.size()) < config_.hole_min_neighbors) {continue;}
        const auto range = std::minmax_element(nearby.begin(), nearby.end());
        if (*range.second - *range.first > config_.hole_max_height_diff) {continue;}
        float sum = 0.0F;
        for (const float value : nearby) {sum += value;}
        filled[index] = {sum / static_cast<float>(nearby.size()), 0.0F, nearby.size()};
      }
    }
    grid.swap(filled);
  }

  const double spatial_denom = 2.0 * config_.bilateral_sigma_spatial * config_.bilateral_sigma_spatial;
  const double height_denom = 2.0 * config_.bilateral_sigma_height * config_.bilateral_sigma_height;
  for (int pass = 0; pass < config_.bilateral_passes && config_.bilateral_radius > 0; ++pass) {
    auto filtered = grid;
    for (int row = 0; row < static_cast<int>(height); ++row) {
      for (int column = 0; column < static_cast<int>(width); ++column) {
        const std::size_t index = static_cast<std::size_t>(row) * width + column;
        if (!grid[index].valid()) {continue;}
        double weighted_sum = grid[index].z;
        double weight_sum = 1.0;
        for (int dy = -config_.bilateral_radius; dy <= config_.bilateral_radius; ++dy) {
          for (int dx = -config_.bilateral_radius; dx <= config_.bilateral_radius; ++dx) {
            const int neighbor_row = row + dy;
            const int neighbor_column = column + dx;
            if ((dx == 0 && dy == 0) || neighbor_row < 0 || neighbor_column < 0 ||
              neighbor_row >= static_cast<int>(height) || neighbor_column >= static_cast<int>(width))
            {
              continue;
            }
            const auto & neighbor = grid[static_cast<std::size_t>(neighbor_row) * width + neighbor_column];
            if (!neighbor.valid() || std::abs(neighbor.z - grid[index].z) > config_.bilateral_max_height_diff) {
              continue;
            }
            const double dz = neighbor.z - grid[index].z;
            const double weight = std::exp(-(static_cast<double>(dx * dx + dy * dy) / spatial_denom) -
              ((dz * dz) / height_denom));
            weighted_sum += weight * neighbor.z;
            weight_sum += weight;
          }
        }
        filtered[index].z = static_cast<float>(weighted_sum / weight_sum);
      }
    }
    grid.swap(filtered);
  }
}

}  // namespace autonomy_light::elevation
