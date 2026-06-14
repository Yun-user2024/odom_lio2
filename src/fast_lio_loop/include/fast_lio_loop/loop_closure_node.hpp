#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

#include "fast_lio_loop/keyframe.hpp"
#include "fast_lio_loop/scan_context.hpp"
#include "fast_lio_loop/pose_graph.hpp"

#include <deque>
#include <memory>

class LoopClosureNode : public rclcpp::Node {
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using CloudPtr = Cloud::Ptr;

  explicit LoopClosureNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LoopClosureNode();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timerCallback();
  void saveMapCallback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                        const std_srvs::srv::Trigger::Response::SharedPtr res);
  bool needNewKeyframe(const Eigen::Vector3d& pos, const Eigen::Quaterniond& q) const;
  void processKeyframe(KeyFramePtr kf);
  bool detectLoopByScanContext(int query_id, int& match_id, int& best_shift, Eigen::Matrix4d& relative_T);
  bool verifyLoopByICP(const KeyFrame& query_kf, const KeyFrame& match_kf, int best_shift,
                        Eigen::Matrix4d& relative_T, float& fitness);
  void publishCorrectedPath();
  void publishLoopMarkers(int kf1_id, int kf2_id, const Eigen::Matrix4d& relative_T);
  void odomToEigen(const nav_msgs::msg::Odometry& msg, Eigen::Vector3d& pos,
                    Eigen::Quaterniond& q) const;
  CloudPtr downsampleCloud(const CloudPtr& cloud, float leaf_size);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_corrected_path_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_corrected_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corrected_map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_loop_markers_;
  rclcpp::TimerBase::SharedPtr timer_;
 rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_map_;

  std::mutex data_mutex_;
  std::deque<KeyFramePtr> keyframes_;
  std::shared_ptr<ScanContext> scan_context_manager_;
  std::shared_ptr<PoseGraph> pose_graph_;

  Eigen::Vector3d latest_odom_pos_;
  Eigen::Quaterniond latest_odom_quat_;
  double latest_odom_time_;
  bool has_odom_ = false;

  CloudPtr latest_cloud_;
  bool has_cloud_ = false;

  bool need_keyframe_ = false;
  int keyframe_counter_ = 0;
  int loop_counter_ = 0;

  CloudPtr corrected_map_;

  double kf_dist_threshold_;
  double kf_angle_threshold_;
  double kf_cloud_downsample_;
  double sc_match_threshold_;
  double sc_max_range_;
  int sc_candidate_top_k_;
  int sc_history_skip_;
  double loop_search_radius_;
  double icp_max_corr_dist_;
  int icp_max_iter_;
  double icp_fitness_threshold_;
  double corrected_map_leaf_;
 std::string map_file_path_;
  bool publish_corrected_map_;
  void saveCorrectedMap(const std::string& path);
};
