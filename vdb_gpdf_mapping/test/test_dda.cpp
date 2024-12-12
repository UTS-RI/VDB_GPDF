/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Jianhao JIAO (https://gogojjh.github.io/)
 * Date: 2022-09-14
 * Description: test on dda and hdda speed
 * Reference: OPENVDB and vdb_gpdf_ros
 */
#include <cmath>
#include <iostream>
#include <string>

#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "timer.h"

float ComputeSDF(const Eigen::Vector3d &origin, const Eigen::Vector3d &point,
                 const Eigen::Vector3d &voxel_center) {
  const Eigen::Vector3d v_voxel_origin = voxel_center - origin;
  const Eigen::Vector3d v_point_voxel = point - voxel_center;
  const double dist = v_point_voxel.norm();
  const double proj = v_voxel_origin.dot(v_point_voxel);
  const double sign = proj / std::abs(proj);
  return static_cast<float>(sign * dist);
}

Eigen::Vector3d GetVoxelCenter(const openvdb::Coord &voxel,
                               const openvdb::math::Transform &xform) {
  const float voxel_size = xform.voxelSize()[0];
  openvdb::math::Vec3d v_wf = xform.indexToWorld(voxel) + voxel_size / 2.0;
  return Eigen::Vector3d(v_wf.x(), v_wf.y(), v_wf.z());
}

void generateSpherePoint(std::vector<Eigen::Vector3d> &points) {
  double radius_ball = 10.0;
  for (double phi = 0.0; phi <= M_PI; phi += M_PI / 360.0) {
    double radius_circle = radius_ball * sin(phi);
    double z = radius_ball * cos(phi);
    for (double theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 360.0) {
      double x = radius_circle * cos(theta);
      double y = radius_circle * sin(theta);
      points.push_back(Eigen::Vector3d(x, y, z));
    }
  }

  radius_ball /= 2.0;
  for (double phi = 0.0; phi <= M_PI; phi += M_PI / 360.0) {
    double radius_circle = radius_ball * sin(phi);
    double z = radius_ball * cos(phi);
    for (double theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 360.0) {
      double x = radius_circle * cos(theta);
      double y = radius_circle * sin(theta);
      points.push_back(Eigen::Vector3d(x, y, z));
    }
  }
}

int main(int argc, char *argv[]) {
  TIC;
  openvdb::initialize();

  // Create an empty floating-point grid with background value 0.
  float sdf_trunc = 0.2;
  float voxel_size = 0.1;
  bool space_carving = false;

  openvdb::FloatGrid::Ptr tsdf = openvdb::FloatGrid::create(sdf_trunc);
  // Name the grid .
  tsdf->setName("sdf_distance");
  // Associate a scaling transform with the grid that sets the voxel size
  tsdf->setTransform(openvdb::math::Transform::createLinearTransform(
      /*voxel size=*/voxel_size));
  tsdf->setGridClass(openvdb::GRID_LEVEL_SET);

  openvdb::FloatGrid::Ptr weights = openvdb::FloatGrid::create(0.0f);
  weights->setName("sdf_weight");
  weights->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size));
  weights->setGridClass(openvdb::GRID_UNKNOWN);

  // insert a point
  std::vector<Eigen::Vector3d> points;
  generateSpherePoint(points);
  {
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const openvdb::math::Transform &xform = tsdf->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());
    auto tsdf_acc = tsdf->getAccessor();
    auto weights_acc = weights->getAccessor();

    size_t cnt = 0;
    TRE;
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
      const Eigen::Vector3d direction = point - origin;
      openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
      dir.normalize();
      const auto depth = static_cast<float>(direction.norm());
      const float t0 = depth - sdf_trunc;
      const float t1 = depth + sdf_trunc;
      const auto ray =
          openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
      openvdb::math::DDA<decltype(ray)> dda(ray);
      do {
        const auto voxel = dda.voxel();
        const auto voxel_center = GetVoxelCenter(voxel, xform);
        const auto sdf = ComputeSDF(origin, point, voxel_center);
        if (sdf > -sdf_trunc) {
          const float tsdf = std::min(sdf_trunc, sdf);
          const float weight = 1.0;
          const float last_weight = weights_acc.getValue(voxel);
          const float last_tsdf = tsdf_acc.getValue(voxel);
          const float new_weight = weight + last_weight;
          const float new_tsdf =
              (last_tsdf * last_weight + tsdf * weight) / (new_weight);
          tsdf_acc.setValue(voxel, new_tsdf);       // tsdf update
          weights_acc.setValue(voxel, new_weight);  // weight update
          if (last_weight < 0.5) cnt++;
        }
      } while (dda.step());
    });
    TOC("inserting points costs", true);
    std::cout << "number of active voxels: " << cnt << std::endl;
  }

  // search active voxel
  {
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const openvdb::math::Transform &xform = tsdf->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());
    auto tsdf_acc = tsdf->getAccessor();
    auto weights_acc = weights->getAccessor();

    TRE;
    size_t cnt = 0;
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
      const Eigen::Vector3d direction = point - origin;
      openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
      dir.normalize();
      const auto depth = static_cast<float>(direction.norm());
      const float t0 = 0.0f;
      const float t1 = depth - sdf_trunc;

      openvdb::math::Ray<float> ray =
          openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
      openvdb::math::DDA<decltype(ray)> dda(ray);
      do {
        const openvdb::Coord voxel = dda.voxel();
        if (tsdf_acc.isValueOn(voxel)) {
          // std::cout << "active voxel: " << voxel << std::endl;
          // dda.print();
          cnt++;
        }
      } while (dda.step());
    });
    TOC("DDA searching active voxel", true);
    std::cout << "number of active voxels (repeated counted): " << cnt
              << std::endl;
  }

  {
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const openvdb::math::Transform &xform = tsdf->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());
    auto tsdf_acc = tsdf->getAccessor();
    auto weights_acc = weights->getAccessor();

    size_t cnt = 0;
    TRE;
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
      const Eigen::Vector3d direction = point - origin;
      openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
      dir.normalize();
      const auto depth = static_cast<float>(direction.norm());
      const float t0 = 0.0f;
      const float t1 = depth - sdf_trunc;

      openvdb::math::Ray<float> ray =
          openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
      openvdb::math::Ray<float> ray_sub =
          openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);

      openvdb::math::VolumeHDDA<openvdb::FloatGrid::TreeType, decltype(ray), 0>
          volume_hdda;
      std::vector<openvdb::math::Ray<float>::TimeSpan> times;
      volume_hdda.hits(ray, tsdf_acc, times);
      for (const auto &t : times) {
        ray_sub.setTimes(t.t0, t.t1);
        openvdb::math::DDA<decltype(ray_sub)> dda(ray_sub);
        do {
          const openvdb::Coord voxel = dda.voxel();
          if (tsdf_acc.isValueOn(voxel)) {
            cnt++;
          }
        } while (dda.step());
      }
    });
    TOC("HDDA searching active voxel: ", true);
    std::cout << "number of active voxels (repeated counted): " << cnt
              << std::endl;
  }

  // {
  //   openvdb::io::File file("/tmp/test_dda.vdb");
  //   openvdb::GridPtrVec grids;
  //   grids.push_back(tsdf);
  //   file.write(grids);
  //   file.close();
  // }

  return 0;
}
