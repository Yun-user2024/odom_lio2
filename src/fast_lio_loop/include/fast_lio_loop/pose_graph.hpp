#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <vector>
#include <mutex>

struct PoseGraph {
  using Pose3 = gtsam::Pose3;

  PoseGraph();

  static Pose3 eigenToPose3(const Eigen::Vector3d& t, const Eigen::Quaterniond& q);
  static Pose3 computeRelative(const Eigen::Vector3d& t1, const Eigen::Quaterniond& q1,
                                const Eigen::Vector3d& t2, const Eigen::Quaterniond& q2);

  void addOdometryFactor(int from_id, int to_id, const Pose3& relative_pose,
                          float dist_moved, bool update_now = false);

  void addLoopFactor(int from_id, int to_id, const Pose3& relative_pose,
                      float fitness_score, bool update_now = true);

  void update();
  Eigen::Matrix4d getOptimizedPose(int id) const;
  std::vector<Eigen::Matrix4d> getAllOptimizedPoses() const;
  int numNodes() const { return num_nodes_; }

private:
  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  int num_nodes_ = 0;
  mutable std::mutex mutex_;
  bool is_initialized_ = false;
};

using PoseGraphPtr = std::shared_ptr<PoseGraph>;
