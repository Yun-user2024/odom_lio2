#include "LIO_loop/scan_context.hpp"
#include <cmath>
#include <algorithm>

ScanContext::Desc ScanContext::makeDescriptor(
  const pcl::PointCloud<pcl::PointXYZI>& cloud,
  float max_range)
{
  Desc desc = Desc::Zero();

  for (const auto& pt : cloud.points) {
    float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (r < 0.1f || r > max_range) continue;

    int ring = static_cast<int>(r / max_range * RING_NUM);
    ring = std::min(ring, RING_NUM - 1);

    float theta = std::atan2(pt.y, pt.x);
    int sector = static_cast<int>((theta + M_PI) / (2.0 * M_PI) * SECTOR_NUM);
    sector = std::min(sector, SECTOR_NUM - 1);

    float z = pt.z;
    if (z > desc(ring, sector)) {
      desc(ring, sector) = z;
    }
  }

  return desc;
}

Eigen::VectorXf ScanContext::makeRingKey(const Desc& desc)
{
  Eigen::VectorXf key(RING_NUM);
  for (int r = 0; r < RING_NUM; ++r) {
    int non_empty = 0;
    for (int s = 0; s < SECTOR_NUM; ++s) {
      if (desc(r, s) > -1000.0f) ++non_empty;
    }
    key(r) = static_cast<float>(non_empty) / SECTOR_NUM;
  }
  return key;
}

float ScanContext::ringKeyDist(const Eigen::VectorXf& k1, const Eigen::VectorXf& k2)
{
  return (k1 - k2).lpNorm<1>() / k1.size();
}

float ScanContext::descDist(const Desc& d1, const Desc& d2, int* best_shift)
{
  float min_dist = std::numeric_limits<float>::max();
  int min_shift = 0;

  for (int shift = 0; shift < SECTOR_NUM; ++shift) {
    float dist = 0.0f;
    for (int c = 0; c < SECTOR_NUM; ++c) {
      int col_b = (c + shift) % SECTOR_NUM;
      float dot = 0.0f, norm_a = 0.0f, norm_b = 0.0f;
      for (int r = 0; r < RING_NUM; ++r) {
        float va = d1(r, c);
        float vb = d2(r, col_b);
        dot += va * vb;
        norm_a += va * va;
        norm_b += vb * vb;
      }
      float denom = std::sqrt(norm_a * norm_b);
      if (denom > 1e-6f) {
        dist += 1.0f - dot / denom;
      } else {
        dist += 1.0f;
      }
    }
    dist /= SECTOR_NUM;
    if (dist < min_dist) {
      min_dist = dist;
      min_shift = shift;
    }
  }

  if (best_shift) *best_shift = min_shift;
  return min_dist;
}
