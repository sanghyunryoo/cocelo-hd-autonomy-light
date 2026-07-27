#pragma once

#include <chrono>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "autonomy_light/slam/isam_pose_graph.hpp"
#include "autonomy_light/slam/keyframe.hpp"
#include "autonomy_light/slam/map_product_builder.hpp"
#include "autonomy_light/slam/nano_gicp_registration.hpp"
#include "autonomy_light/slam/ring_sector_index.hpp"

namespace autonomy_light
{

class FullSlamNode final : public rclcpp::Node
{
public:
  FullSlamNode();
  ~FullSlamNode() override;

private:
  struct TimedOdom
  {
    double stamp_seconds{};
    nav_msgs::msg::Odometry message;
  };

  void onOdometry(const nav_msgs::msg::Odometry & message);
  void onCloud(const sensor_msgs::msg::PointCloud2 & message);
  void processOneLoop();
  void broadcastMapToOdom();
  void publishState();
  [[nodiscard]] bool shouldCreateKeyframe(const Eigen::Isometry3d & pose) const;
  [[nodiscard]] bool matchingOdometry(double stamp_seconds, nav_msgs::msg::Odometry & output) const;
  [[nodiscard]] slam::Cloud buildTargetSubmap(std::size_t center_id) const;
  [[nodiscard]] Eigen::Isometry3d mapFromOdom() const;
  [[nodiscard]] slam::MapProducts buildMapProducts() const;
  void saveMapProducts(const slam::MapProducts & products) const;
  [[nodiscard]] static Eigen::Isometry3d poseFromOdometry(const nav_msgs::msg::Odometry & odom);
  [[nodiscard]] static double seconds(const builtin_interfaces::msg::Time & stamp);

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  double keyframe_distance_;
  double keyframe_angle_radians_;
  std::size_t loop_exclude_recent_;
  std::size_t loop_candidate_count_;
  double loop_descriptor_distance_;
  std::size_t submap_neighbor_count_;
  double map_voxel_size_;
  std::string raw_map_file_;
  std::string refined_map_file_;
  double map_save_period_seconds_;
  slam::MapProductBuilder map_builder_;

  mutable std::mutex mutex_;
  std::deque<TimedOdom> odom_cache_;
  std::vector<slam::Keyframe> keyframes_;
  std::deque<std::size_t> pending_loop_ids_;
  std::set<std::pair<std::size_t, std::size_t>> accepted_loops_;
  slam::RingSectorIndex place_index_;
  slam::IsamPoseGraph pose_graph_;
  slam::NanoGicpRegistration registration_;
  bool map_dirty_{false};
  std::chrono::steady_clock::time_point last_map_save_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr refined_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace autonomy_light
