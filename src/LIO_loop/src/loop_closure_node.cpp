#include "LIO_loop/loop_closure_node.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <chrono>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

LoopClosureNode::LoopClosureNode(const rclcpp::NodeOptions& options)
  : Node("loop_closure_node", options)
{
  // Parameters
  auto declare_param = [this](const std::string& name, auto default_val) {
    return this->declare_parameter(name, default_val);
  };

  kf_dist_threshold_ = declare_param("keyframe.distance_threshold", 1.0);
  kf_angle_threshold_ = declare_param("keyframe.angle_threshold", 0.35);  // ~20 deg
  kf_cloud_downsample_ = declare_param("keyframe.cloud_downsample", 0.5);
  sc_max_range_ = declare_param("scan_context.max_range", 80.0);
  sc_candidate_top_k_ = declare_param("scan_context.candidate_top_k", 10);
  sc_match_threshold_ = declare_param("scan_context.match_threshold", 0.6);
  sc_history_skip_ = declare_param("scan_context.history_skip", 30);
  loop_search_radius_ = declare_param("loop.search_radius", 20.0);
  icp_max_corr_dist_ = declare_param("icp.max_corr_dist", 2.0);
  icp_max_iter_ = declare_param("icp.max_iterations", 50);
  icp_fitness_threshold_ = declare_param("icp.fitness_threshold", 0.5);
  corrected_map_leaf_ = declare_param("map.corrected_leaf_size", 0.2);
  map_file_path_ = declare_param("map.save_path", "/home/hy/ros_ws/fast_ws/map");
  publish_corrected_map_ = declare_param("publish.corrected_map", true);

  // Initialize members
  scan_context_manager_ = std::make_shared<ScanContext>();
  scan_context_manager_->max_range = sc_max_range_;
  pose_graph_ = std::make_shared<PoseGraph>();
  latest_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  corrected_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  // Subscriptions
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", 100, std::bind(&LoopClosureNode::odomCallback, this, std::placeholders::_1));

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", 100, std::bind(&LoopClosureNode::cloudCallback, this, std::placeholders::_1));

  // Publishers
  pub_corrected_path_ = this->create_publisher<nav_msgs::msg::Path>("/loop_closure/path", 10);
  pub_corrected_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/loop_closure/odometry", 10);
  pub_corrected_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_closure/map", 10);
  pub_loop_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure/markers", 10);

  // Services
  srv_save_map_ = this->create_service<std_srvs::srv::Trigger>(
    "loop_closure/save_map", std::bind(&LoopClosureNode::saveMapCallback, this,
    std::placeholders::_1, std::placeholders::_2));

  // Timer for periodic publishing (2 Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&LoopClosureNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Loop closure node initialized");
}

void LoopClosureNode::odomToEigen(
  const nav_msgs::msg::Odometry& msg,
  Eigen::Vector3d& pos, Eigen::Quaterniond& q) const
{
  pos.x() = msg.pose.pose.position.x;
  pos.y() = msg.pose.pose.position.y;
  pos.z() = msg.pose.pose.position.z;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
  q.w() = msg.pose.pose.orientation.w;
}

bool LoopClosureNode::needNewKeyframe(
  const Eigen::Vector3d& pos, const Eigen::Quaterniond& q) const
{
  if (keyframes_.empty()) return true;

  auto& last = keyframes_.back();
  double dist = (pos - last->position).norm();
  double angle = q.angularDistance(last->orientation);
  return dist > kf_dist_threshold_ || angle > kf_angle_threshold_;
}

LoopClosureNode::CloudPtr LoopClosureNode::downsampleCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float leaf_size)
{
  LoopClosureNode::CloudPtr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*filtered);
  return filtered;
}

void LoopClosureNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odomToEigen(*msg, latest_odom_pos_, latest_odom_quat_);
  latest_odom_time_ = rclcpp::Time(msg->header.stamp).seconds();
  has_odom_ = true;

  if (!needNewKeyframe(latest_odom_pos_, latest_odom_quat_)) return;
  need_keyframe_ = true;
}

void LoopClosureNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Always keep latest cloud
  pcl::fromROSMsg(*msg, *latest_cloud_);
  has_cloud_ = true;

  if (!need_keyframe_ || !has_odom_) return;
  need_keyframe_ = false;

  // Create keyframe
  auto kf = std::make_shared<KeyFrame>();
  kf->id = keyframe_counter_++;
  kf->timestamp = latest_odom_time_;
  kf->position = latest_odom_pos_;
  kf->orientation = latest_odom_quat_;
  kf->opt_position = kf->position;
  kf->opt_orientation = kf->orientation;

  // Downsample and store cloud
  kf->cloud = downsampleCloud(latest_cloud_, kf_cloud_downsample_);

  // Process the keyframe (loop detection + graph update)
  processKeyframe(kf);
}

void LoopClosureNode::processKeyframe(KeyFramePtr kf)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Generate ScanContext descriptor
  auto sc_desc = ScanContext::makeDescriptor(*kf->cloud, sc_max_range_);
  auto ring_key = ScanContext::makeRingKey(sc_desc);

  // Store in manager
  scan_context_manager_->descriptors_.push_back(sc_desc);
  scan_context_manager_->ring_keys_.push_back(ring_key);

  // Add to keyframe deque
  keyframes_.push_back(kf);
  int kf_id = kf->id;

  // Add odometry edge to pose graph
  if (keyframes_.size() >= 2) {
    auto& prev = *(keyframes_.end() - 2);
    auto relative = PoseGraph::computeRelative(
      prev->position, prev->orientation, kf->position, kf->orientation);
    double dist = (kf->position - prev->position).norm();
    pose_graph_->addOdometryFactor(prev->id, kf_id, relative, dist, false);
  }

  // Loop detection
  int loop_match_id = -1;
  Eigen::Matrix4d loop_T = Eigen::Matrix4d::Identity();

  int best_shift = 0;
  if (detectLoopByScanContext(kf_id, loop_match_id, best_shift, loop_T)) {
    RCLCPP_INFO(this->get_logger(), "Loop detected: KF %d -> KF %d", kf_id, loop_match_id);

    // Update pose graph with loop factor
    float fitness = 0.0f;
    if (keyframes_[loop_match_id]->cloud) {
      Eigen::Matrix4d icp_T;
      float icp_fitness;
      if (verifyLoopByICP(*kf, *keyframes_[loop_match_id], best_shift, icp_T, icp_fitness)) {
        loop_T = icp_T;
        fitness = icp_fitness;
        kf->has_loop = true;
        keyframes_[loop_match_id]->has_loop = true;
        loop_counter_++;

        publishLoopMarkers(kf_id, loop_match_id, loop_T);
      } else {
        RCLCPP_WARN(this->get_logger(), "Loop candidate %d rejected by ICP (fitness %.3f)", loop_match_id, icp_fitness);
        loop_match_id = -1;
      }
    }

    if (loop_match_id >= 0) {
      // Compute relative pose from match_kf to query_kf in world frame
      // loop_T = T_match^{-1} * T_query  (from ICP)
      gtsam::Pose3 match_T = PoseGraph::eigenToPose3(
        keyframes_[loop_match_id]->position, keyframes_[loop_match_id]->orientation);
      gtsam::Pose3 query_T = PoseGraph::eigenToPose3(kf->position, kf->orientation);
      gtsam::Pose3 loop_relative = match_T.between(query_T);

      pose_graph_->addLoopFactor(loop_match_id, kf_id, loop_relative, fitness, true);

      // Update all keyframe optimized poses
      auto opt_poses = pose_graph_->getAllOptimizedPoses();
      for (size_t i = 0; i < opt_poses.size() && i < keyframes_.size(); ++i) {
        Eigen::Matrix3d rot = opt_poses[i].block<3,3>(0,0);
        Eigen::Quaterniond opt_q(rot);
        keyframes_[i]->opt_position = opt_poses[i].block<3,1>(0,3);
        keyframes_[i]->opt_orientation = opt_q;
      }

      publishCorrectedPath();
    }
  } else {
    // Still update the pose graph with current odometry edge
    if (keyframes_.size() >= 2) {
      pose_graph_->update();
    }
  }
}

bool LoopClosureNode::detectLoopByScanContext(
  int query_id, int& match_id, int& best_shift, Eigen::Matrix4d& relative_T)
{
  if (scan_context_manager_->ring_keys_.size() < 5) {
    return false;
  }

  const auto& query_desc = scan_context_manager_->descriptors_.back();
  const auto& query_key = scan_context_manager_->ring_keys_.back();
  const Eigen::Vector3d& query_pos = keyframes_[query_id]->position;

  int end_idx = static_cast<int>(scan_context_manager_->ring_keys_.size()) - sc_history_skip_ - 1;
  end_idx = std::max(end_idx, 0);

  std::vector<std::pair<float, int>> candidates;
  for (int i = 0; i <= end_idx; ++i) {
    float d = ScanContext::ringKeyDist(query_key, scan_context_manager_->ring_keys_[i]);
    candidates.emplace_back(d, i);
  }

  if (candidates.empty()) return false;
  std::sort(candidates.begin(), candidates.end());

  float best_dist = static_cast<float>(sc_match_threshold_);
  float best_candidate_dist = 1.0f;
  match_id = -1;

  for (int k = 0; k < std::min(sc_candidate_top_k_, static_cast<int>(candidates.size())); ++k) {
    int idx = candidates[k].second;
    int shift = 0;
    float desc_dist = ScanContext::descDist(query_desc, scan_context_manager_->descriptors_[idx], &shift);

    // Soft position distance (use corrected pose if previously optimized)
    double pos_dist = (query_pos - keyframes_[idx]->position).norm();
    if (keyframes_[idx]->has_loop) {
      pos_dist = (keyframes_[query_id]->opt_position - keyframes_[idx]->opt_position).norm();
    }

    // Z-difference: skip cross-floor candidates
    double z_diff = std::abs(query_pos.z() - keyframes_[idx]->position.z());
    if (z_diff > 3.0) {
      continue;
    }

    RCLCPP_DEBUG(this->get_logger(),
      "SC candidate KF %d: descDist=%.3f posDist=%.1f zDiff=%.1f shift=%d",
      idx, desc_dist, pos_dist, z_diff, shift);

    if (desc_dist < best_candidate_dist) {
      best_candidate_dist = desc_dist;
    }
    if (desc_dist < best_dist) {
      best_dist = desc_dist;
      match_id = idx;
      best_shift = shift;
    }
  }

  // Log the best candidate even when no match
  static float worst_best_dist = 1.0f;
  if (match_id >= 0) {
    worst_best_dist = best_dist;
    RCLCPP_INFO(this->get_logger(),
      "Loop candidate KF %d (descDist=%.3f, shift=%d)",
      match_id, best_dist, best_shift);
  } else {
    // No high-quality ScanContext match — try geometry proximity search
    for (int i = std::min(end_idx, query_id - 2); i >= 0; --i) {
      double pos_dist = (query_pos - keyframes_[i]->position).norm();
      if (keyframes_[i]->has_loop) {
        pos_dist = (keyframes_[query_id]->opt_position - keyframes_[i]->opt_position).norm();
      }
      double z_diff = std::abs(query_pos.z() - keyframes_[i]->position.z());
      if (pos_dist < 8.0 && z_diff < 2.0) {
        // Compute ScanContext yaw shift for better ICP initial guess
        if (i < static_cast<int>(scan_context_manager_->descriptors_.size())) {
          int shift = 0;
          ScanContext::descDist(query_desc, scan_context_manager_->descriptors_[i], &shift);
          best_shift = shift;
        }
        match_id = i;
        RCLCPP_INFO(this->get_logger(),
          "Geometry proximity candidate KF %d (posDist=%.2f, shift=%d)",
          match_id, pos_dist, best_shift);
        break;
      }
    }
    if (match_id < 0) {
      RCLCPP_DEBUG(this->get_logger(),
        "Best SC descDist=%.3f (no match < %.3f, %d candidates)",
        best_candidate_dist, static_cast<float>(sc_match_threshold_),
        static_cast<int>(candidates.size()));
    }
  }

  return match_id >= 0;
}

bool LoopClosureNode::verifyLoopByICP(
  const KeyFrame& query_kf, const KeyFrame& match_kf, int best_shift,
  Eigen::Matrix4d& relative_T, float& fitness)
{
  // Stage 1: coarse alignment with large correspondence distance
  // and ScanContext yaw-corrected initial guess
  float yaw = static_cast<float>(best_shift) * 2.0f * M_PI / ScanContext::SECTOR_NUM;
  Eigen::Matrix3f R_yaw = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  // Corrected initial guess: apply ScanContext yaw correction
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  init_guess.block<3,3>(0,0) = R_yaw *
    (match_kf.orientation.inverse() * query_kf.orientation).toRotationMatrix().cast<float>();
  init_guess.block<3,1>(0,3) =
    (match_kf.orientation.inverse() * (query_kf.position - match_kf.position)).cast<float>();

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp_coarse;
  gicp_coarse.setMaxCorrespondenceDistance(15.0);
  gicp_coarse.setMaximumIterations(50);
  gicp_coarse.setTransformationEpsilon(1e-3);
  gicp_coarse.setEuclideanFitnessEpsilon(1e-3);

  pcl::PointCloud<pcl::PointXYZI> aligned_coarse;
  gicp_coarse.setInputSource(query_kf.cloud);
  gicp_coarse.setInputTarget(match_kf.cloud);
  gicp_coarse.align(aligned_coarse, init_guess);

  if (!gicp_coarse.hasConverged()) {
    fitness = gicp_coarse.getFitnessScore();
    std::cout << "[ICP] Coarse alignment failed, fitness=" << fitness << std::endl;
    return false;
  }

  // Stage 2: fine alignment with configured correspondence distance
  Eigen::Matrix4f coarse_T = gicp_coarse.getFinalTransformation();

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp_fine;
  gicp_fine.setMaxCorrespondenceDistance(icp_max_corr_dist_);
  gicp_fine.setMaximumIterations(icp_max_iter_);
  gicp_fine.setTransformationEpsilon(1e-6);
  gicp_fine.setEuclideanFitnessEpsilon(1e-6);

  pcl::PointCloud<pcl::PointXYZI> aligned_fine;
  gicp_fine.setInputSource(query_kf.cloud);
  gicp_fine.setInputTarget(match_kf.cloud);
  gicp_fine.align(aligned_fine, coarse_T);

  fitness = gicp_fine.getFitnessScore();
  relative_T = gicp_fine.getFinalTransformation().cast<double>();

  std::cout << "[ICP] Fine alignment fitness=" << fitness
            << " converged=" << gicp_fine.hasConverged()
            << " threshold=" << icp_fitness_threshold_
            << " accepted=" << (gicp_fine.hasConverged() && fitness < icp_fitness_threshold_)
            << std::endl;

  return gicp_fine.hasConverged() && fitness < icp_fitness_threshold_;
}


void LoopClosureNode::publishCorrectedPath()
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "camera_init";
  path_msg.header.stamp = this->get_clock()->now();

  for (const auto& kf : keyframes_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "camera_init";
    ps.header.stamp = rclcpp::Time(static_cast<int64_t>(kf->timestamp * 1e9));
    ps.pose.position.x = kf->opt_position.x();
    ps.pose.position.y = kf->opt_position.y();
    ps.pose.position.z = kf->opt_position.z();
    ps.pose.orientation.x = kf->opt_orientation.x();
    ps.pose.orientation.y = kf->opt_orientation.y();
    ps.pose.orientation.z = kf->opt_orientation.z();
    ps.pose.orientation.w = kf->opt_orientation.w();
    path_msg.poses.push_back(ps);
  }

  pub_corrected_path_->publish(path_msg);

  // Also publish corrected odometry (latest)
  if (!keyframes_.empty()) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "camera_init";
    odom_msg.child_frame_id = "body";
    odom_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframes_.back()->timestamp * 1e9));
    odom_msg.pose.pose.position.x = keyframes_.back()->opt_position.x();
    odom_msg.pose.pose.position.y = keyframes_.back()->opt_position.y();
    odom_msg.pose.pose.position.z = keyframes_.back()->opt_position.z();
    odom_msg.pose.pose.orientation.x = keyframes_.back()->opt_orientation.x();
    odom_msg.pose.pose.orientation.y = keyframes_.back()->opt_orientation.y();
    odom_msg.pose.pose.orientation.z = keyframes_.back()->opt_orientation.z();
    odom_msg.pose.pose.orientation.w = keyframes_.back()->opt_orientation.w();
    pub_corrected_odom_->publish(odom_msg);
  }

  // Build corrected map
  if (publish_corrected_map_ && !keyframes_.empty()) {
    pcl::PointCloud<pcl::PointXYZI> corrected_cloud;
    for (const auto& kf : keyframes_) {
      if (!kf->cloud || kf->cloud->empty()) continue;

      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3,3>(0,0) = kf->opt_orientation.toRotationMatrix();
      T.block<3,1>(0,3) = kf->opt_position;

      // Compute correction relative to raw odometry
      Eigen::Matrix4d T_raw = Eigen::Matrix4d::Identity();
      T_raw.block<3,3>(0,0) = kf->orientation.toRotationMatrix();
      T_raw.block<3,1>(0,3) = kf->position;

      Eigen::Matrix4d T_correction = T * T_raw.inverse();

      for (const auto& pt : kf->cloud->points) {
        Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d p_corrected = T_correction * p;
        pcl::PointXYZI pcl_pt;
        pcl_pt.x = p_corrected.x();
        pcl_pt.y = p_corrected.y();
        pcl_pt.z = p_corrected.z();
        pcl_pt.intensity = pt.intensity;
        corrected_cloud.push_back(pcl_pt);
      }
    }

    if (!corrected_cloud.empty()) {
      pcl::PointCloud<pcl::PointXYZI> filtered;
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(corrected_cloud));
      vg.setLeafSize(corrected_map_leaf_, corrected_map_leaf_, corrected_map_leaf_);
      vg.filter(filtered);

      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(filtered, cloud_msg);
      cloud_msg.header.frame_id = "camera_init";
      cloud_msg.header.stamp = this->get_clock()->now();
      pub_corrected_map_->publish(cloud_msg);
    }
  }
}

void LoopClosureNode::publishLoopMarkers(
  int kf1_id, int kf2_id, const Eigen::Matrix4d& relative_T)
{
  visualization_msgs::msg::MarkerArray markers;

  if (kf1_id >= static_cast<int>(keyframes_.size()) ||
      kf2_id >= static_cast<int>(keyframes_.size())) return;

  auto& kf1 = keyframes_[kf1_id];
  auto& kf2 = keyframes_[kf2_id];

  // Line connecting the two keyframes
  visualization_msgs::msg::Marker line;
  line.header.frame_id = "camera_init";
  line.header.stamp = this->get_clock()->now();
  line.ns = "loop_closure";
  line.id = loop_counter_;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.1;
  line.color.a = 1.0;
  line.color.r = 0.0;
  line.color.g = 1.0;
  line.color.b = 0.0;

  geometry_msgs::msg::Point p1, p2;
  p1.x = kf1->opt_position.x();
  p1.y = kf1->opt_position.y();
  p1.z = kf1->opt_position.z();
  p2.x = kf2->opt_position.x();
  p2.y = kf2->opt_position.y();
  p2.z = kf2->opt_position.z();
  line.points.push_back(p1);
  line.points.push_back(p2);
  markers.markers.push_back(line);

  pub_loop_markers_->publish(markers);
}

void LoopClosureNode::timerCallback()
{
  // Periodically republish corrected path
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!keyframes_.empty()) {
    publishCorrectedPath();
  }
}

void LoopClosureNode::saveMapCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req;
  saveCorrectedMap(map_file_path_);
  res->success = true;
  res->message = "Map saved";
}

void LoopClosureNode::saveCorrectedMap(const std::string& path)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (keyframes_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Save map: no keyframes available");
    return;
  }

  // Build corrected point cloud from all keyframes
  pcl::PointCloud<pcl::PointXYZI> corrected_cloud;
  for (const auto& kf : keyframes_) {
    if (!kf->cloud || kf->cloud->empty()) continue;

    Eigen::Matrix4d T_opt = Eigen::Matrix4d::Identity();
    T_opt.block<3,3>(0,0) = kf->opt_orientation.toRotationMatrix();
    T_opt.block<3,1>(0,3) = kf->opt_position;

    Eigen::Matrix4d T_raw = Eigen::Matrix4d::Identity();
    T_raw.block<3,3>(0,0) = kf->orientation.toRotationMatrix();
    T_raw.block<3,1>(0,3) = kf->position;

    Eigen::Matrix4d T_correction = T_opt * T_raw.inverse();

    for (const auto& pt : kf->cloud->points) {
      Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
      Eigen::Vector4d p_corrected = T_correction * p;
      pcl::PointXYZI pcl_pt;
      pcl_pt.x = p_corrected.x();
      pcl_pt.y = p_corrected.y();
      pcl_pt.z = p_corrected.z();
      pcl_pt.intensity = pt.intensity;
      corrected_cloud.push_back(pcl_pt);
    }
  }

  if (corrected_cloud.empty()) {
    RCLCPP_WARN(this->get_logger(), "Save map: corrected cloud empty");
    return;
  }

  // Downsample
  pcl::PointCloud<pcl::PointXYZI> filtered;
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(corrected_cloud));
  vg.setLeafSize(corrected_map_leaf_, corrected_map_leaf_, corrected_map_leaf_);
  vg.filter(filtered);

  // Build filename with timestamp
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&t);
  char buf[64];
  std::strftime(buf, sizeof(buf), "corrected_map_%Y%m%d_%H%M%S.pcd", &tm);

  // Ensure directory exists
  std::string full_path = path + "/" + buf;

  pcl::PCDWriter writer;
  writer.writeBinary(full_path, filtered);

  RCLCPP_INFO(this->get_logger(), "Saved corrected map to %s (%zu points)",
    full_path.c_str(), filtered.size());
}

LoopClosureNode::~LoopClosureNode()
{
  saveCorrectedMap(map_file_path_);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoopClosureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
