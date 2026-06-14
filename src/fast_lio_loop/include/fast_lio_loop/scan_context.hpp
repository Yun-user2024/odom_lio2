#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

struct ScanContext {
  static const int RING_NUM = 20;
  static const int SECTOR_NUM = 60;

  using Desc = Eigen::Matrix<float, RING_NUM, SECTOR_NUM>;

  /// @brief Build a ScanContext descriptor from a point cloud
  static Desc makeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    float max_range = 80.0f);

  /// @brief Compute ring key from descriptor (ratio of non-empty bins per ring)
  static Eigen::VectorXf makeRingKey(const Desc& desc);

  /// @brief L1 distance between two ring keys for fast candidate search
  static float ringKeyDist(const Eigen::VectorXf& k1, const Eigen::VectorXf& k2);

  /// @brief Column-wise cosine distance between two descriptors (yaw-invariant)
  static float descDist(const Desc& d1, const Desc& d2, int* best_shift = nullptr);

  float max_range = 80.0f;
  int ring_num = RING_NUM;
  int sector_num = SECTOR_NUM;

  std::vector<Desc> descriptors_;
  std::vector<Eigen::VectorXf> ring_keys_;
};
