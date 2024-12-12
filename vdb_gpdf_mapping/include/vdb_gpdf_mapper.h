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
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "MarchingCubesConst.h"
#include "VDBVolume.h"
#include "transform.h"

#include "openvdb/openvdb.h"
#include "vdb_gpdf_mapping_msgs/SaveMap.h"
#include "vdb_gpdf_mapping_msgs/SliceQueryMap.h"
#include "vdb_gpdf_mapping_msgs/PointsQueryMap.h"

namespace vdb_gpdf_mapping {
class VDBGPDFMapper {
 public:
  typedef tf::StampedTransform Transformation;
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

  VDBGPDFMapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~VDBGPDFMapper() = default;

  // ROS callbacks.
  void points_callback(const sensor_msgs::PointCloud2::Ptr &input);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &input);
  bool saveMap_callback(vdb_gpdf_mapping_msgs::SaveMap::Request &req,
                        vdb_gpdf_mapping_msgs::SaveMap::Response &res);

  bool pointsQueryMap_callback(vdb_gpdf_mapping_msgs::PointsQueryMap::Request &reqQ,
                               vdb_gpdf_mapping_msgs::PointsQueryMap::Response &resS);
  bool sliceQueryMap_callback(vdb_gpdf_mapping_msgs::SliceQueryMap::Request &reqQ,
                              vdb_gpdf_mapping_msgs::SliceQueryMap::Response &resS);
  
  void visualQueriedDistances(const std::vector<float> queryPoints, const std::vector<double> pRes);
  void visualQueriedGradients(const std::vector<float> queryPoints, const std::vector<double> pRes);
  void visualCameraPose(Eigen::Matrix4d tf_matrix);
  visualization_msgs::Marker create_arrow(float scale, geometry_msgs::Point start, geometry_msgs::Point end,  int idnum, float color[]);
  
  void setConfig();
  void mapIntegrateProcess();

  void processPointCloudMessage(
      const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
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
  vdb_gpdf::VDBVolume gsdf_volume;
  common::Transformer retrive_mpose;

  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber points_sub, odom_sub;
  ros::Publisher vdbmap_pub;
  ros::Publisher camera_marker_pub;
  
  ros::Publisher localGPsPoints_pub;
  ros::Publisher localQueryPointsDis_pub;
  ros::Publisher globalGPsPoints_pub;
  ros::Publisher globalQueryPointsDis_pub;
  ros::Publisher globalQueryPointsGrd_pub;

  ros::ServiceServer save_map_srv;
  ros::ServiceServer points_query_map_srv;
  ros::ServiceServer slice_query_map_srv;

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