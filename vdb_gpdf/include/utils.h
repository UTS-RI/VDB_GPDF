/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 *        Jiaojian HAO (https://gogojjh.github.io/)
 * Date: 2022-11-05
 * Reference: vdbfusion_ros: https://github.com/PRBonn/vdbfusion_ros and voxblox
 * https://github.com/ethz-asl/voxblox
 */
#ifndef UTILS_H
#define UTILS_H

#include <openvdb/openvdb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

namespace common {

// clang-format off
class WeightFunction {
    public:
    inline static float tsdf_ = 0.0f;
    inline static float epsilon_ = 0.0f;

    inline static auto constant_weight = [](float sdf) { return 1.0f; };
    inline static auto no_constant_weight = [](float sdf) {
      const float dist_z = std::abs(sdf);
      if (dist_z > epsilon_) {
        return 1.0f / (dist_z * dist_z);
      }
    };
    inline static auto linear_weight = [](float sdf) {
    static float a = 0.5f * tsdf_ / (tsdf_ - epsilon_);
    static float b = 0.5f         / (tsdf_ - epsilon_);
    if (sdf < epsilon_) return 1.0f;
    if ((epsilon_ <= sdf) && (sdf <= tsdf_))
        return (a - b * sdf  + 0.5f);
    return 0.5f;
    };

    inline static auto exp_weight = [](float sdf) {
    if (sdf < epsilon_) return 1.0f;
    if ((epsilon_ <= sdf) && (sdf <= tsdf_))
        return float(exp(-tsdf_ * (sdf - epsilon_) * (sdf - epsilon_)));
    return float(exp(-tsdf_ * (tsdf_ - epsilon_) * (tsdf_ - epsilon_)));
    };
};
// clang-format on

template <typename PCLPoint>
openvdb::Vec3i convertColor(const PCLPoint& point);

template <>
inline openvdb::Vec3i convertColor(const pcl::PointXYZRGB& point) {
  // std::cout << "r,g,b: "<< static_cast<int>(point.r) <<
  // static_cast<int>(point.g) << static_cast<int>(point.b) <<std::endl;
  return openvdb::Vec3i(static_cast<int>(point.r), static_cast<int>(point.g),
                        static_cast<int>(point.b));
}

template <>
inline openvdb::Vec3i convertColor(const pcl::PointXYZI& point) {
  return openvdb::Vec3i(0, 0, 0);
}

template <>
inline openvdb::Vec3i convertColor(const pcl::PointXYZ& /*point*/) {
  return openvdb::Vec3i(0, 0, 0);
}

// Check if all coordinates in the PCL point are finite.
template <typename PointType>
inline bool isPointFinite(const PointType& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

template <typename Scalar, typename PointType>
inline void PCL2Eigen(const pcl::PointCloud<PointType>& ptcloud_pcl,
                      std::vector<Eigen::Matrix<Scalar, 3, 1>>& ptcloud_eig) {
  ptcloud_eig.clear();
  ptcloud_eig.reserve(ptcloud_pcl.size());
  for (const auto point : ptcloud_pcl) {
    if (!isPointFinite(point)) continue;
    Eigen::Matrix<Scalar, 3, 1> pt;
    pt(0) = static_cast<Scalar>(point.x);
    pt(1) = static_cast<Scalar>(point.y);
    pt(2) = static_cast<Scalar>(point.z);
    ptcloud_eig.push_back(pt);
  }
}

class ThreadSafeIndex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // NOTE: The ThreadSafeIndex base destructor must be marked virtual.
  //       Otherwise the destructors of derived classes don't get called when
  //       derived class instances are destructed through base class pointers.
  //       This would result leaking memory due to derived class member
  //       variables not being freed.
  virtual ~ThreadSafeIndex() = default;

  /// returns true if index is valid, false otherwise
  bool getNextIndex(size_t* idx);

  void reset();

 protected:
  explicit ThreadSafeIndex(size_t number_of_points);

  virtual size_t getNextIndexImpl(size_t base_idx) = 0;

  std::atomic<size_t> atomic_idx_;
  const size_t number_of_points_;
};

}  // namespace common

#endif  // UTILS_H