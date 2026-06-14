#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

struct KeyFrame {
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using CloudPtr = Cloud::Ptr;

  int id = -1;
  double timestamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  // Downsampled scan registered in the LiDAR body frame
  CloudPtr cloud = nullptr;

  // GTSAM-optimized pose (relative to the same origin as position/orientation)
  // Equal to position/orientation before any loop closure correction
  Eigen::Vector3d opt_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond opt_orientation = Eigen::Quaterniond::Identity();

  bool has_loop = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using KeyFramePtr = std::shared_ptr<KeyFrame>;
