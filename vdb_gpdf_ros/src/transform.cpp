/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 * Description: callback on the transformer, save the pose according to the pt
 * time Reference: mainly from the voxblox with slightly modification.
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tf_conversions/tf_eigen.h>

#include "transform.h"

namespace common {

Transformer::Transformer(const ros::NodeHandle &nh,
                         const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), buffer_(ros::Duration(50, 0)), tf_(buffer_) {
  nh_private_.getParam("world_frame", world_frame_);
  nh_private_.getParam("child_frame", child_frame_);
  nh_private_.getParam("pose_source", pose_source_);

  double tolerance_ms;
  nh_private_.getParam("timestamp_tolerance_ms", tolerance_ms);
  tolerance_ns_ =
      static_cast<int64_t>(tolerance_ms * 100000);  // 1ms = 1000000 ns chagne to 0.1ms

  double tx = 0, ty = 0, tz = 0, x = 0, y = 0, z = 0, w = 1;  // static
  nh_private_.getParam("tx", tx);
  nh_private_.getParam("ty", ty);
  nh_private_.getParam("tz", tz);
  nh_private_.getParam("x", x);
  nh_private_.getParam("y", y);
  nh_private_.getParam("z", z);
  nh_private_.getParam("w", w);

  kindrQuatT tmp(kindrRotaT(w, x, y, z),
                 Eigen::Matrix<double, 3, 1>(tx, ty, tz));
  bool invert_static_tf = false;
  nh_private_.getParam("invert_static_tf", invert_static_tf);
  if (invert_static_tf)
    static_tf = tmp.inverse();
  else
    static_tf = tmp;
  //LOG(INFO) << "static tf:\n\r" << static_tf << "\n\r -----";
  // [0:tf_tree, 1:tf_topic, 2:odom_topic]
  if (pose_source_ == -1) {
    // ERROR!!!
  } else if (pose_source_ == 1) {
    std::string tf_topic_;
    nh_private_.getParam("tf_topic", tf_topic_);
    tf_sub_ = nh_.subscribe(tf_topic_, 500, &Transformer::tfCallback, this);
    LOG(INFO) << "TF TF TF TOPIC FOR POSE, Please check the topic name:"
              << tf_topic_;
  } else if (pose_source_ == 2) {
    std::string odom_topic_;
    nh_private_.getParam("odom_topic", odom_topic_);
    odom_sub_ =
        nh_.subscribe(odom_topic_, 40, &Transformer::odomCallback, this);
    LOG(INFO) << "ODOM ODOM ODOM TOPIC FOR POSE, Please check the topic name:"
              << odom_topic_;
  }
  LOG(INFO) << "Transformer init success";
}
bool Transformer::lookUpTransformfromSource(
    const sensor_msgs::PointCloud2::ConstPtr &input,
    Eigen::Matrix4d &tf_matrix) {
  ros::Time timestamp = input->header.stamp;
  // [0:tf_tree, 1:tf_topic, 2:odom_topic]
  if (pose_source_ == 0) {
    tf::StampedTransform T_G_C;
    // Previous behavior was just to use the latest transform if the time is in
    // the future. Now we will just wait.
    if (!tf_listener_.canTransform(world_frame_, child_frame_,
                                   timestamp)) {
      std::string* msg;
      int test = tf_listener_.getLatestCommonTime(world_frame_, child_frame_, timestamp, msg);
      ros::Duration tolerance_ = ros::Duration(0, 1000);
      // in case the bag use the tf2 !!!
      if(buffer_.canTransform(world_frame_, child_frame_, timestamp, tolerance_)) {
        geometry_msgs::TransformStamped tf_msg_;
        tf_msg_ = buffer_.lookupTransform(world_frame_, child_frame_,  timestamp, tolerance_);
        std::vector<double> pose = {tf_msg_.transform.translation.x, tf_msg_.transform.translation.y,
                                    tf_msg_.transform.translation.z, tf_msg_.transform.rotation.w,
                                    tf_msg_.transform.rotation.x,    tf_msg_.transform.rotation.y,
                                    tf_msg_.transform.rotation.z};
        kindrQuatT res(kindrRotaT(pose[3], pose[4], pose[5], pose[6]),
                      Eigen::Matrix<double, 3, 1>(pose[0], pose[1], pose[2]));

        kindrT2Eigen(tf_matrix, res);
        //LOG(INFO) << "I'm using tf2 tree."; // only the tum
        return true;
      }
      LOG(WARNING) << "Please check the tf tree between: "
                 << child_frame_ << " and " << world_frame_;
      return false;
    }
    try {
      tf_listener_.lookupTransform(world_frame_, child_frame_,
                                   timestamp, T_G_C);
      //LOG(INFO) << "I'm using tf tree."; // mai city, kitti, newer
    } catch (tf::TransformException &ex) {  // NOLINT
      LOG(ERROR) << "Please check the tf tree between: "
                 << child_frame_ << " and " << world_frame_;
      ROS_ERROR_STREAM(
          "Error getting TF transform from sensor data: " << ex.what());
    }
    // TODO fix it! ===> cannot find the problem now
    tf::Vector3 trans = T_G_C.getOrigin();
    tf::Quaternion rota = T_G_C.getRotation();
    transform2Eigen(tf_matrix, trans.getX(), trans.getY(), trans.getZ(),
                    rota.getW(), rota.getX(), rota.getY(), rota.getZ());
    //LOG(INFO) << tf_matrix;
    return true;
  } else {
    if (TimeWithPose_queue.empty()) {
      LOG(WARNING) << "No match found for transform timestamp: " << timestamp
                   << " as transform queue is empty.";
      return false;
    }
    bool match_found = false;
    int id = 0;
    std::vector<pairTP>::iterator it = TimeWithPose_queue.begin();
    for (; it != TimeWithPose_queue.end(); it++, id++) {
      if (it->first >= timestamp) {
        // If the current transform is newer than the requested timestamp, we
        // need to break.
        if ((it->first - timestamp).toNSec() <= tolerance_ns_) {
          match_found = true;
        }
        break;
      }

      if ((timestamp - it->first).toNSec() < tolerance_ns_) {
        match_found = true;
        break;
      }
    }

    if (match_found) {
      std::vector<double> pose = it->second;

      kindrQuatT res(kindrRotaT(pose[3], pose[4], pose[5], pose[6]),
                     Eigen::Matrix<double, 3, 1>(pose[0], pose[1], pose[2]));

      res = res * static_tf.inverse();
      kindrT2Eigen(tf_matrix, res);
      TimeWithPose_queue.erase(TimeWithPose_queue.begin(), it);
      return true;
    } else {
      // If we think we have an inexact match, have to check that we're still
      // within bounds and interpolate.
      if (it == TimeWithPose_queue.begin() || it == TimeWithPose_queue.end()) {
        ROS_WARN_STREAM_THROTTLE(
            30, "No match found for transform timestamp: "
                    << timestamp
                    << " Queue front: " << TimeWithPose_queue.front().first
                    << " back: " << TimeWithPose_queue.back().first);
        return false;
      }
      std::vector<double> pose = it->second;
      kindrQuatT tf_newest(
          kindrRotaT(pose[3], pose[4], pose[5], pose[6]),
          Eigen::Matrix<double, 3, 1>(pose[0], pose[1], pose[2]));

      int64_t offset_newest_ns = (it->first - timestamp).toNSec();
      // We already checked that this is not the beginning.
      it--;
      pose = it->second;
      kindrQuatT tf_oldest(
          kindrRotaT(pose[3], pose[4], pose[5], pose[6]),
          Eigen::Matrix<double, 3, 1>(pose[0], pose[1], pose[2]));
      int64_t offset_oldest_ns = (timestamp - it->first).toNSec();

      // Interpolate between the two transformations using the exponential map.
      float t_diff_ratio =
          static_cast<float>(offset_oldest_ns) /
          static_cast<float>(offset_newest_ns + offset_oldest_ns);

      Eigen::Matrix<double, 6, 1> diff_vector =
          (tf_oldest.inverse() * tf_newest).log();

      kindrQuatT interpolate_tf = tf_oldest *
                                  kindrQuatT::exp(t_diff_ratio * diff_vector) *
                                  static_tf.inverse();
      kindrT2Eigen(tf_matrix, interpolate_tf);
      return true;
    }
  }
}
void Transformer::tfCallback(
    const geometry_msgs::TransformStamped &transform_msg) {
  geometry_msgs::Transform tf_msg_ = transform_msg.transform;
  std::vector<double> pose = {tf_msg_.translation.x, tf_msg_.translation.y,
                              tf_msg_.translation.z, tf_msg_.rotation.w,
                              tf_msg_.rotation.x,    tf_msg_.rotation.y,
                              tf_msg_.rotation.z};
  TimeWithPose_queue.push_back(
      std::make_pair(transform_msg.header.stamp, pose));
}
void Transformer::odomCallback(const nav_msgs::Odometry::ConstPtr &input) {
  std::vector<double> pose = {
      input->pose.pose.position.x,    input->pose.pose.position.y,
      input->pose.pose.position.z,    input->pose.pose.orientation.w,
      input->pose.pose.orientation.x, input->pose.pose.orientation.y,
      input->pose.pose.orientation.z};
  TimeWithPose_queue.push_back(std::make_pair(input->header.stamp, pose));
}
}  // namespace common