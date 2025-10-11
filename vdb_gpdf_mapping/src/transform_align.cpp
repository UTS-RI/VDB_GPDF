/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 * Description: callback on the transformer, save the pose according to the pt
 * time Reference: mainly from the voxblox with slightly modification.
 */
#include "transform_align.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace common {
  
// Constructor for Transformer
Transformer::Transformer(std::shared_ptr<rclcpp::Node> node)
: node_(node), buffer_(node->get_clock()), tf_listener_(buffer_){
    RCLCPP_INFO(node_->get_logger(), "Transformer initializing.");
    node_->declare_parameter<std::string>("world_frame", "map");
    node_->declare_parameter<std::string>("child_frame", "base_link");
    node_->declare_parameter<int>("pose_source", 0);
    node_->declare_parameter<double>("timestamp_tolerance_ms", 1.0);

    node_->declare_parameter<double>("tx", 0.0);
    node_->declare_parameter<double>("ty", 0.0);
    node_->declare_parameter<double>("tz", 0.0);
    node_->declare_parameter<double>("x", 0.0);
    node_->declare_parameter<double>("y", 0.0);
    node_->declare_parameter<double>("z", 0.0);
    node_->declare_parameter<double>("w", 1.0);
    node_->declare_parameter<bool>("invert_static_tf", false);

    node_->get_parameter("world_frame", world_frame_);
    node_->get_parameter("child_frame", child_frame_);
    node_->get_parameter("pose_source", pose_source_);

    double tolerance_ms;
    node_->get_parameter("timestamp_tolerance_ms", tolerance_ms);
    tolerance_ns_ = static_cast<int64_t>(tolerance_ms * 100000);  // 1ms = 1000000 ns change to 0.1ms

    double tx = 0, ty = 0, tz = 0, x = 0, y = 0, z = 0, w = 1;  // static
    node_->get_parameter("tx", tx);
    node_->get_parameter("ty", ty);
    node_->get_parameter("tz", tz);
    node_->get_parameter("x", x);
    node_->get_parameter("y", y);
    node_->get_parameter("z", z);
    node_->get_parameter("w", w);

    kindrQuatT tmp(kindrRotaT(w, x, y, z), Eigen::Matrix<double, 3, 1>(tx, ty, tz));
    bool invert_static_tf = false;
    node_->get_parameter("invert_static_tf", invert_static_tf);
    if (invert_static_tf)
        static_tf = tmp.inverse();
    else
        static_tf = tmp;
    
    RCLCPP_INFO(node_->get_logger(), "Pose source mode: %d", pose_source_);
    node_->declare_parameter<std::string>("tf_topic", "/tf_topic");
    node_->declare_parameter<std::string>("odom_topic", "/odom_topic");

    // [0:tf_tree, 1:tf_topic, 2:odom_topic]
    if (pose_source_ == 1) {
        std::string tf_topic_;
        
        node_->get_parameter("tf_topic", tf_topic_);
        tf_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
            tf_topic_, 500, std::bind(&Transformer::tfCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Using TF topic: %s", tf_topic_.c_str());
    } else if (pose_source_ == 2) {
        std::string odom_topic_;
        node_->get_parameter("odom_topic", odom_topic_);
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 40, std::bind(&Transformer::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Using Odometry topic: %s", odom_topic_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "Transformer initialized successfully.");
}

// Look up the transform from source
bool Transformer::lookUpTransformfromSource(const sensor_msgs::msg::PointCloud2::SharedPtr &input,
                                            Eigen::Matrix4d &tf_matrix) {
    rclcpp::Time timestamp = input->header.stamp;
    // [0:tf_tree, 1:tf_topic, 2:odom_topic]
    if (pose_source_ == 0) { // not tested yet since we
        rclcpp::Duration tolerance_ = rclcpp::Duration(0, 1000);
        if (buffer_.canTransform(world_frame_, child_frame_, timestamp, tolerance_)) {
            geometry_msgs::msg::TransformStamped tf_msg_;
            tf_msg_ = buffer_.lookupTransform(world_frame_, child_frame_, timestamp, tolerance_);
            std::vector<double> pose = {tf_msg_.transform.translation.x, tf_msg_.transform.translation.y,
                                        tf_msg_.transform.translation.z, tf_msg_.transform.rotation.w,
                                        tf_msg_.transform.rotation.x, tf_msg_.transform.rotation.y,
                                        tf_msg_.transform.rotation.z};
            kindrQuatT res(kindrRotaT(pose[3], pose[4], pose[5], pose[6]),
                            Eigen::Matrix<double, 3, 1>(pose[0], pose[1], pose[2]));
            kindrT2Eigen(tf_matrix, res);
            return true;
        }
        RCLCPP_WARN(node_->get_logger(), "Please check the tf tree between: %s and %s",
                    child_frame_.c_str(), world_frame_.c_str());
        return false;
    } else {
        if (TimeWithPose_queue.empty()) {
            RCLCPP_WARN(node_->get_logger(),
            "No match found for transform timestamp: %ld as transform queue is empty.",
            timestamp.nanoseconds());
            return false;
        }
        bool match_found = false;
        int id = 0;
        std::vector<pairTP>::iterator it = TimeWithPose_queue.begin();
        for (; it != TimeWithPose_queue.end(); it++, id++) {
          if (it->first >= timestamp) {
            // If the current transform is newer than the requested timestamp, we
            // need to break.
            if ((it->first - timestamp).nanoseconds() <= tolerance_ns_) {
              match_found = true;
            }
            break;
          }
    
          if ((timestamp - it->first).nanoseconds() < tolerance_ns_) {
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
            RCLCPP_WARN(node_->get_logger(),
            "No match found for transform timestamp: %ld as transform queue is not aligned.",
            timestamp.nanoseconds());
            return false;
          }
          std::vector<double> pose = it->second;
          kindrQuatT tf_newest(
              kindrRotaT(pose[3], pose[4], pose[5], pose[6]),
              Eigen::Matrix<double, 3, 1>(pose[0], pose[1], pose[2]));
    
          int64_t offset_newest_ns = (it->first - timestamp).nanoseconds();
          // We already checked that this is not the beginning.
          it--;
          pose = it->second;
          kindrQuatT tf_oldest(
              kindrRotaT(pose[3], pose[4], pose[5], pose[6]),
              Eigen::Matrix<double, 3, 1>(pose[0], pose[1], pose[2]));
          int64_t offset_oldest_ns = (timestamp - it->first).nanoseconds();
    
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

// TF callback for receiving transformations
void Transformer::tfCallback(const geometry_msgs::msg::TransformStamped &transform_msg) {
    geometry_msgs::msg::Transform tf_msg_ = transform_msg.transform;
    std::vector<double> pose = {tf_msg_.translation.x, tf_msg_.translation.y,
                                tf_msg_.translation.z, tf_msg_.rotation.w,
                                tf_msg_.rotation.x, tf_msg_.rotation.y,
                                tf_msg_.rotation.z};
    TimeWithPose_queue.push_back(std::make_pair(transform_msg.header.stamp, pose));
}

// Odometry callback
void Transformer::odomCallback(const nav_msgs::msg::Odometry &input) {
    std::vector<double> pose = {
        input.pose.pose.position.x, input.pose.pose.position.y,
        input.pose.pose.position.z, input.pose.pose.orientation.w,
        input.pose.pose.orientation.x, input.pose.pose.orientation.y,
        input.pose.pose.orientation.z};
    TimeWithPose_queue.push_back(std::make_pair(input.header.stamp, pose));
}

}  // namespace common