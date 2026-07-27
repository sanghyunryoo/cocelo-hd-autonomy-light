#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace autonomy_light::elevation
{

struct GridGeometry
{
  double resolution{0.01};
  double x_length{1.6};
  double y_length{0.8};

  [[nodiscard]] std::size_t width() const
  {
    return static_cast<std::size_t>(std::ceil(x_length / resolution));
  }

  [[nodiscard]] std::size_t height() const
  {
    return static_cast<std::size_t>(std::ceil(y_length / resolution));
  }

  [[nodiscard]] std::size_t size() const {return width() * height();}
  [[nodiscard]] double xMin() const {return -0.5 * x_length;}
  [[nodiscard]] double yMin() const {return -0.5 * y_length;}

  [[nodiscard]] bool contains(const Eigen::Vector3f & point) const
  {
    return point.x() >= xMin() && point.x() < -xMin() &&
           point.y() >= yMin() && point.y() < -yMin();
  }

  [[nodiscard]] std::size_t index(const Eigen::Vector3f & point) const
  {
    const auto column = static_cast<std::size_t>((point.x() - xMin()) / resolution);
    const auto row = static_cast<std::size_t>((point.y() - yMin()) / resolution);
    return row * width() + column;
  }

  [[nodiscard]] Eigen::Vector2f center(const std::size_t row, const std::size_t column) const
  {
    return {static_cast<float>(xMin() + (static_cast<double>(column) + 0.5) * resolution),
      static_cast<float>(yMin() + (static_cast<double>(row) + 0.5) * resolution)};
  }
};

struct GridCell
{
  float z{std::numeric_limits<float>::quiet_NaN()};
  float variance{std::numeric_limits<float>::infinity()};
  std::size_t support{0U};

  [[nodiscard]] bool valid() const {return std::isfinite(z);}
};

using ElevationGrid = std::vector<GridCell>;

}  // namespace autonomy_light::elevation
