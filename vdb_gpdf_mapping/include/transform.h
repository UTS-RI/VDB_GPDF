/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 * Description: callback on the transformer, save the pose according to the pt
 * time Reference: mainly from the voxblox with slightly modification.
 */

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <cmath>
#include <deque>

#include "kindr/minimal/quat-transformation.h"

namespace common {

typedef kindr::minimal::QuatTransformationTemplate<double> kindrQuatT;
typedef kindr::minimal::RotationQuaternionTemplate<double> kindrRotaT;
typedef std::pair<ros::Time, std::vector<double>> pairTP;
inline void transform2Eigen(Eigen::Matrix4d& res, double tx, double ty,
                            double tz, double w, double x, double y, double z) {
  Eigen::Vector3d t(tx, ty, tz);
  Eigen::Quaterniond q(w, x, y, z);

  res.block<3, 3>(0, 0) = q.toRotationMatrix();
  res.block<3, 1>(0, 3) = t;
};
inline void SixVector2Eigen(Eigen::Matrix4d& res, double tx, double ty,
                            double tz, double roll, double pitch, double yaw) {
  Eigen::Vector3d t(tx, ty, tz);
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  res.block<3, 3>(0, 0) = q.toRotationMatrix();
  res.block<3, 1>(0, 3) = t;
};

inline void kindrT2Eigen(Eigen::Matrix4d& res, kindrQuatT& input) {
  res.block<3, 3>(0, 0) = input.getRotationMatrix();
  res.block<3, 1>(0, 3) = input.getPosition();
};

// part of code from voxblox: https://github.com/ethz-asl/voxblox
// part of reference vdbfusion_ros: https://github.com/PRBonn/vdbfusion_ros
class Transformer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Transformer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  virtual ~Transformer() = default;

  bool lookUpTransformfromSource(
      const sensor_msgs::PointCloud2::ConstPtr& input,
      Eigen::Matrix4d& tf_matrix);
  inline std::string getWorldframe() {return world_frame_;};
 private:
  bool tfTreeRead(const ros::Time& timestamp, const ros::Duration& tolerance,
                  geometry_msgs::TransformStamped& transform);

  void tfCallback(const geometry_msgs::TransformStamped& input);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& input);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener tf_listener_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_;
  
  ros::Subscriber tf_sub_, odom_sub_;

  // save all pose queue
  std::vector<std::pair<ros::Time, std::vector<double>>> TimeWithPose_queue;
  kindr::minimal::QuatTransformationTemplate<double> static_tf;

  int64_t tolerance_ns_;
  int pose_source_ = -1;  // [0:tf_tree, 1:tf_topic, 2:odom_topic]

  std::string world_frame_ = "map";
  std::string child_frame_ = "lidar";
};

}  // namespace common

#endif  // TRANSFORM_H