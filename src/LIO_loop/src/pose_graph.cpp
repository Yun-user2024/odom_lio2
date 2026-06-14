#include "LIO_loop/pose_graph.hpp"
#include <iostream>

PoseGraph::PoseGraph()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  params.factorization = gtsam::ISAM2Params::CHOLESKY;
  isam_ = gtsam::ISAM2(params);
  std::cout << "[PoseGraph] iSAM2 initialized" << std::endl;
}

PoseGraph::Pose3 PoseGraph::eigenToPose3(
  const Eigen::Vector3d& t,
  const Eigen::Quaterniond& q)
{
  gtsam::Rot3 R(q.toRotationMatrix());
  gtsam::Point3 p(t.x(), t.y(), t.z());
  return Pose3(R, p);
}

PoseGraph::Pose3 PoseGraph::computeRelative(
  const Eigen::Vector3d& t1, const Eigen::Quaterniond& q1,
  const Eigen::Vector3d& t2, const Eigen::Quaterniond& q2)
{
  gtsam::Rot3 R1(q1.toRotationMatrix());
  gtsam::Point3 p1(t1.x(), t1.y(), t1.z());
  gtsam::Rot3 R2(q2.toRotationMatrix());
  gtsam::Point3 p2(t2.x(), t2.y(), t2.z());
  Pose3 pose1(R1, p1);
  Pose3 pose2(R2, p2);
  return pose1.between(pose2);
}

void PoseGraph::addOdometryFactor(
  int from_id, int to_id, const Pose3& relative_pose,
  float dist_moved, bool /*update_now*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  using gtsam::symbol_shorthand::X;

  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_initial;

  if (!is_initialized_) {
    // ======== FIRST CALL: add prior for first node + first odometry edge ========
    // This is called when we have at least 2 keyframes (e.g. from_id=0, to_id=1)
    std::cout << "[PoseGraph] First init: prior X(" << from_id
              << ") + odom X(" << from_id << ")->X(" << to_id << ")" << std::endl;

    // Prior for the first node (world origin anchor)
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    new_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(from_id), Pose3(), prior_noise));
    new_initial.insert(X(from_id), Pose3());

    // First odometry edge: from_id -> to_id
    double pos_noise_init = 0.1;
    double ang_noise_init = 0.05;
    auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << ang_noise_init, ang_noise_init, ang_noise_init,
       pos_noise_init, pos_noise_init, pos_noise_init).finished());
    new_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(from_id), X(to_id), relative_pose, odom_noise));
    new_initial.insert(X(to_id), relative_pose);

    is_initialized_ = true;
    num_nodes_ = std::max(to_id + 1, from_id + 1);
    isam_.update(new_graph, new_initial);
    std::cout << "[PoseGraph] First update done, " << num_nodes_ << " nodes" << std::endl;
    return;
  }

  // ======== SUBSEQUENT CALLS: odometry edge from_id -> to_id ========
  double pos_noise = 0.05 + 0.1 * dist_moved;
  double ang_noise = 0.02 + 0.05 * dist_moved;
  auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << ang_noise, ang_noise, ang_noise,
     pos_noise, pos_noise, pos_noise).finished());

  new_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    X(from_id), X(to_id), relative_pose, odom_noise));

  // Initial estimate for new node from current iSAM2 state
  gtsam::Values current = isam_.calculateBestEstimate();
  if (current.exists(X(from_id))) {
    gtsam::Pose3 from_pose = current.at<gtsam::Pose3>(X(from_id));
    new_initial.insert(X(to_id), from_pose * relative_pose);
    std::cout << "[PoseGraph] Odom " << from_id << "->" << to_id
              << " (dist=" << dist_moved << "): from X(" << from_id << ") found in isam"
              << std::endl;
  } else {
    // Fallback: shouldn't happen for correctly ordered keyframes
    std::cerr << "[PoseGraph] WARNING: X(" << from_id << ") NOT found in isam! "
              << "Using relative_pose as initial." << std::endl;
    new_initial.insert(X(to_id), relative_pose);
  }

  num_nodes_ = std::max(num_nodes_, to_id + 1);
  isam_.update(new_graph, new_initial);
}

void PoseGraph::addLoopFactor(
  int from_id, int to_id, const Pose3& relative_pose,
  float fitness_score, bool /*update_now*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  using gtsam::symbol_shorthand::X;

  std::cout << "[PoseGraph] Loop edge: X(" << from_id << ")->X(" << to_id
            << ") fitness=" << fitness_score << std::endl;

  double pos_noise = 0.1 + 0.5 * fitness_score;
  double ang_noise = 0.05 + 0.2 * fitness_score;
  auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << ang_noise, ang_noise, ang_noise,
     pos_noise, pos_noise, pos_noise).finished());

  gtsam::NonlinearFactorGraph new_graph;
  new_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
    X(from_id), X(to_id), relative_pose, loop_noise));

  isam_.update(new_graph);
}

void PoseGraph::update()
{
  // No-op: factors are processed incrementally in addOdometryFactor/addLoopFactor
}

Eigen::Matrix4d PoseGraph::getOptimizedPose(int id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  using gtsam::symbol_shorthand::X;

  try {
    gtsam::Values result = isam_.calculateBestEstimate();
    if (result.exists(X(id))) {
      return result.at<gtsam::Pose3>(X(id)).matrix();
    }
    std::cerr << "[PoseGraph] getOptimizedPose(" << id << ") not found" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[PoseGraph] getOptimizedPose(" << id << ") exception: " << e.what() << std::endl;
  }

  return Eigen::Matrix4d::Identity();
}

std::vector<Eigen::Matrix4d> PoseGraph::getAllOptimizedPoses() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Eigen::Matrix4d> poses;
  using gtsam::symbol_shorthand::X;

  try {
    gtsam::Values result = isam_.calculateBestEstimate();
    std::cout << "[PoseGraph] getAllOptimizedPoses: " << result.size()
              << " values in isam, expected " << num_nodes_ << " nodes" << std::endl;
    for (int i = 0; i < num_nodes_; ++i) {
      if (result.exists(X(i))) {
        poses.push_back(result.at<gtsam::Pose3>(X(i)).matrix());
      } else {
        std::cerr << "[PoseGraph] getAllOptimizedPoses: X(" << i << ") missing!" << std::endl;
        poses.push_back(Eigen::Matrix4d::Identity());
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "[PoseGraph] getAllOptimizedPoses exception: " << e.what() << std::endl;
  }

  return poses;
}
