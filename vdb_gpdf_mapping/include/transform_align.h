/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 * Description: callback on the transformer, save the pose according to the pt
 * time Reference: mainly from the voxblox with slightly modification.
 */
 #include <rclcpp/rclcpp.hpp>
 #include <geometry_msgs/msg/transform.hpp>
 #include <geometry_msgs/msg/transform_stamped.hpp>
 #include <nav_msgs/msg/odometry.hpp>
 #include <sensor_msgs/msg/point_cloud2.hpp>
 #include <pcl/point_cloud.h>

 #include <tf2_ros/transform_broadcaster.h>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_eigen/tf2_eigen.hpp>
 //#include <tf2_eigen/tf2_eigen.h>
 //#include <tf2_ros.h>
 #include <Eigen/Core>
 #include <cmath>
 #include <deque>
 
 #include "kindr/minimal/quat-transformation.h"
 
 namespace common {
 
 typedef kindr::minimal::QuatTransformationTemplate<double> kindrQuatT;
 typedef kindr::minimal::RotationQuaternionTemplate<double> kindrRotaT;
 typedef std::pair<rclcpp::Time, std::vector<double>> pairTP;
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
 
 class Transformer {
  public:
   Transformer(std::shared_ptr<rclcpp::Node> node);
 
   ~Transformer() = default;
 
   bool lookUpTransformfromSource(
       const sensor_msgs::msg::PointCloud2::SharedPtr& input,
       Eigen::Matrix4d& tf_matrix);
   inline std::string getWorldframe() { return world_frame_; };
 
  private:
   bool tfTreeRead(const rclcpp::Time& timestamp, const rclcpp::Duration& tolerance,
                   geometry_msgs::msg::TransformStamped& transform);
 
   void tfCallback(const geometry_msgs::msg::TransformStamped& input);
 
   void odomCallback(const nav_msgs::msg::Odometry& input);
 
   std::shared_ptr<rclcpp::Node> node_;
   rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tf_sub_;
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
 
   tf2_ros::Buffer buffer_;
   tf2_ros::TransformListener tf_listener_;
 
   // save all pose queue
   std::vector<std::pair<rclcpp::Time, std::vector<double>>> TimeWithPose_queue;
   kindr::minimal::QuatTransformationTemplate<double> static_tf;
 
   int64_t tolerance_ns_;
   int pose_source_ = -1;  // [0:tf_tree, 1:tf_topic, 2:odom_topic]
 
   std::string world_frame_ = "map";
   std::string child_frame_ = "lidar";
 };
 
 }  // namespace common
 