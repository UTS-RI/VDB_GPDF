/*
 *    VDB-GPDF: Online Gaussian Process Distance Field with VDB Structure
 *    Copyright (C) 2024 Lan Wu <Lan.Wu-2@uts.edu.au>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License v3 as published by
 *    the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License v3 for more details.
 *
 *    You should have received a copy of the GNU General Public License v3
 *    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.
 *
 *    Author: Lan Wu <Lan.Wu-2@uts.edu.au>
 */
#ifndef VDB_GPDF_MAPPER_H_
#define VDB_GPDF_MAPPER_H_

#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <tuple>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "VDBVolume.h"
#include "MarchingCubesConst.h"
#include "transform_align.h"

#include "openvdb/openvdb.h"

#include "vdb_gpdf_mapping_msgs/srv/save_map.hpp"
#include "vdb_gpdf_mapping_msgs/srv/slice_query_map.hpp"
#include "vdb_gpdf_mapping_msgs/srv/points_query_map.hpp"

namespace vdb_gpdf_mapping {

using Transformation = geometry_msgs::msg::TransformStamped;

class VDBGPDFMapper {
 public:
  struct Config {
    int ros_spinner_threads = std::thread::hardware_concurrency();

    // default param
    double min_scan_range = 0.0;
    double max_scan_range = 0.0;
    double max_height = 0.0;
    bool fill_holes_ = false;
    bool use_color_ = false;
    double recon_min_weight_ = 0.5;
  };

  explicit VDBGPDFMapper(std::shared_ptr<rclcpp::Node> node);
  virtual ~VDBGPDFMapper() = default;

  // ROS callbacks.
  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);

  void saveMap_callback(
    const std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SaveMap::Request> req,
    std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SaveMap::Response> res);

  void pointsQueryMap_callback(
    const std::shared_ptr<vdb_gpdf_mapping_msgs::srv::PointsQueryMap::Request> reqQ,
    std::shared_ptr<vdb_gpdf_mapping_msgs::srv::PointsQueryMap::Response> resS);

  void sliceQueryMap_callback(
    const std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SliceQueryMap::Request> reqQ,
    std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SliceQueryMap::Response> resS);
  
  void visualQueriedDistances(const std::vector<float> queryPoints, const std::vector<double> pRes);
  void visualQueriedGradients(const std::vector<float> queryPoints, const std::vector<double> pRes);
  void visualCameraPose(Eigen::Matrix4d tf_matrix);
  visualization_msgs::msg::Marker create_arrow(float scale, geometry_msgs::msg::Point start,
                                                geometry_msgs::msg::Point end, int idnum, float color[]);
  
  void setConfig();
  void mapIntegrateProcess();

  void processPointCloudMessage(
      const sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud_msg,
      const Eigen::Matrix4d &tf_matrix);

  template <typename PCLPoint>
  void filterptRange(const typename pcl::PointCloud<PCLPoint> &pointcloud_pcl,
                     pcl::PointCloud<pcl::PointXYZ> &cloud_filter,
                     std::vector<openvdb::Vec3i> &color);

  // IO.
  const Config &getConfig() const { return config_; }

 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // variables
  Config config_;
  std::unique_ptr<vdb_gpdf::VDBVolume> gsdf_volume;
  common::Transformer retrive_mpose;

  // ros node
  std::shared_ptr<rclcpp::Node> node_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vdbmap_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr camera_marker_pub;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localGPsPoints_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localQueryPointsDis_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalGPsPoints_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalQueryPointsDis_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr globalQueryPointsGrd_pub;
  
  rclcpp::Service<vdb_gpdf_mapping_msgs::srv::SaveMap>::SharedPtr save_map_srv;
  rclcpp::Service<vdb_gpdf_mapping_msgs::srv::PointsQueryMap>::SharedPtr points_query_map_srv;
  rclcpp::Service<vdb_gpdf_mapping_msgs::srv::SliceQueryMap>::SharedPtr slice_query_map_srv;

  std::mutex m_fullmap, m_data;

  std::queue<std::tuple<Eigen::Matrix4d, std::vector<Eigen::Vector3d>,
                        std::vector<openvdb::Vec3i>>>
      data_buf;

  std::thread integrate_thread;

  std::string lidar_topic_ = "/odom_lidar";
  bool debug_print_ = false;
  bool enable_databuf_ = false;
  bool has_rgb_ = false;
  bool has_intensity_ = false;
  int enqueue = 0, dequeue = 0;
  
  double total_Integration_time_ = 0;
  int total_frame_ = 0;
};

}  // namespace vdb_gpdf_mapping

#endif  // VDB_GPDF_MAPPER_H_