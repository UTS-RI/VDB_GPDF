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

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include "timer.h"
#include "utils.h"
#include "vdb_gpdf_mapper.h"
#include "vdb_gpdf_mapping_msgs/srv/save_map.hpp"
#include "vdb_gpdf_mapping_msgs/srv/slice_query_map.hpp"
#include "vdb_gpdf_mapping_msgs/srv/points_query_map.hpp"


namespace vdb_gpdf_mapping {
  VDBGPDFMapper::VDBGPDFMapper(std::shared_ptr<rclcpp::Node> node)
  : node_(node), retrive_mpose(node)
{
  setConfig();
  
  gsdf_volume = std::make_unique<vdb_gpdf::VDBVolume>(node_);

  vdbmap_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/vdbmap", 10);
  camera_marker_pub = node_->create_publisher<visualization_msgs::msg::Marker>("/camera_pose", 10);

  localGPsPoints_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/localGPsPoints", 10);
  localQueryPointsDis_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/localQueryPointsDis", 10);
  globalGPsPoints_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/globalGPsPoints", 10);
  globalQueryPointsDis_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/globalQueryPointsDis", 10);
  globalQueryPointsGrd_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/globalQueryPointsGrd", 10);

  points_sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, rclcpp::QoS(20),
    std::bind(&VDBGPDFMapper::points_callback, this, std::placeholders::_1));

  save_map_srv = node_->create_service<vdb_gpdf_mapping_msgs::srv::SaveMap>(
      "/save_map",
      std::bind(&VDBGPDFMapper::saveMap_callback, this, std::placeholders::_1, std::placeholders::_2));

  points_query_map_srv = node_->create_service<vdb_gpdf_mapping_msgs::srv::PointsQueryMap>(
      "/points_query_map",
      std::bind(&VDBGPDFMapper::pointsQueryMap_callback, this, std::placeholders::_1, std::placeholders::_2));

  slice_query_map_srv = node_->create_service<vdb_gpdf_mapping_msgs::srv::SliceQueryMap>(
      "/slice_query_map",
      std::bind(&VDBGPDFMapper::sliceQueryMap_callback, this, std::placeholders::_1, std::placeholders::_2));


  // initial openvdb volume 
  openvdb::initialize();
}

void VDBGPDFMapper::mapIntegrateProcess() {
  rclcpp::Rate rate(1.0);
  while (rclcpp::ok()) {
    m_data.lock();
    if (data_buf.empty()) {
      m_data.unlock();
      RCLCPP_INFO(node_->get_logger(), "No data. Use ROS2 service to save map.");
      rate.sleep();
      continue;
    }

    int total_num = data_buf.size();
    if (debug_print_) {
      RCLCPP_INFO(node_->get_logger(), "Total frames in buffer: %d", total_num);
    }

    Eigen::Matrix4d tf_matrix = std::get<0>(data_buf.front());
    std::vector<Eigen::Vector3d> points = std::get<1>(data_buf.front());
    std::vector<openvdb::Vec3i> color = std::get<2>(data_buf.front());
    
    std::vector<Eigen::Vector3d> globalGPsPoints, localGPsPoints, localQueryPoints;
    std::vector<openvdb::Vec3i> globalGPsPointsColor;
    std::vector<double> localQueryDis;

    data_buf.pop();
    m_data.unlock();

    Eigen::Vector3d origin = tf_matrix.block<3, 1>(0, 3);
    if (debug_print_) {
      RCLCPP_INFO(node_->get_logger(), "Input pointcloud size: %zu", points.size());
    }
    TIC;

    // Note: simply pick one type of gsdf_volume.Integrate function to run
    // first one is for vdbgpdf, second one is for vdbfusion as comparision
    m_fullmap.lock();
    gsdf_volume->Integrate(points, color, origin,
                          common::WeightFunction::constant_weight, globalGPsPoints, globalGPsPointsColor, 
                          localGPsPoints, localQueryPoints, localQueryDis);
    // gsdf_volume->Integrate(points, color, origin,
    //                       common::WeightFunction::constant_weight);
    m_fullmap.unlock();
    
    double current_time = timer.End();
    total_Integration_time_ += current_time;
    TOC("full Integrate", debug_print_);
    total_frame_ += 1;

    //if (debug_print_) {
      RCLCPP_INFO(node_->get_logger(), "current_time(ms): %.2f; total_time(ms): %.2f; total frame: %d; avg time(ms): %.2f",
                  current_time,
                  total_Integration_time_,
                  total_frame_,
                  total_Integration_time_ / total_frame_);
    //}
    
    visualCameraPose(tf_matrix);
    if (localGPsPoints_pub->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB> localGPsPointsPCL;
      for (const auto& pt : localGPsPoints) {
        pcl::PointXYZRGB pcl_pt;
        pcl_pt.x = static_cast<float>(pt.x());
        pcl_pt.y = static_cast<float>(pt.y());
        pcl_pt.z = static_cast<float>(pt.z());
        pcl_pt.r = 255;
        pcl_pt.g = 0;
        pcl_pt.b = 0;
        localGPsPointsPCL.push_back(pcl_pt);
      }
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(localGPsPointsPCL, msg);
      msg.header.frame_id = retrive_mpose.getWorldframe();
      localGPsPoints_pub->publish(msg);
    }

    if (localQueryPointsDis_pub->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZI> localQueryPointsPCL;
      for (size_t i = 0; i < localQueryPoints.size(); ++i) {
        pcl::PointXYZI pt;
        pt.x = static_cast<float>(localQueryPoints[i].x());
        pt.y = static_cast<float>(localQueryPoints[i].y());
        pt.z = static_cast<float>(localQueryPoints[i].z());
        pt.intensity = static_cast<float>(std::abs(localQueryDis[i]));
        if (pt.intensity >= 1000) continue;
        localQueryPointsPCL.push_back(pt);
      }
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(localQueryPointsPCL, msg);
      msg.header.frame_id = retrive_mpose.getWorldframe();
      localQueryPointsDis_pub->publish(msg);
    }

    if (globalGPsPoints_pub->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB> globalGPsPointsPCL;
      for (size_t i = 0; i < globalGPsPoints.size(); ++i) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(globalGPsPoints[i].x());
        pt.y = static_cast<float>(globalGPsPoints[i].y());
        pt.z = static_cast<float>(globalGPsPoints[i].z());
        pt.r = static_cast<uint8_t>(globalGPsPointsColor[i][0]);
        pt.g = static_cast<uint8_t>(globalGPsPointsColor[i][1]);
        pt.b = static_cast<uint8_t>(globalGPsPointsColor[i][2]);
        globalGPsPointsPCL.push_back(pt);
      }
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(globalGPsPointsPCL, msg);
      msg.header.frame_id = retrive_mpose.getWorldframe();
      globalGPsPoints_pub->publish(msg);
    }

    if (debug_print_)
      std::cout << "------------------------------------------------------"
                << std::endl;
  }
}

void VDBGPDFMapper::points_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr input) {
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
  if (!retrive_mpose.lookUpTransformfromSource(input, tf_matrix)) {
    RCLCPP_INFO(node_->get_logger(), "Didn't find the pair pose, skip this message.");
    return;
  }
  // Horrible hack fix to fix color parsing colors in PCL.
  for (size_t d = 0; d < input->fields.size(); ++d) {
    if (input->fields[d].name == std::string("rgb")) {
      input->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
      has_rgb_ = true;
    } else if (input->fields[d].name == std::string("intensity")) {
      has_intensity_ = true;
    }
  }
  // filter by range
  pcl::PointCloud<pcl::PointXYZ> cloud_filter, trans_cloud;
  std::vector<openvdb::Vec3i> colors;
  
  // Convert differently depending on RGB or I type.
  if (has_rgb_ && config_.use_color_) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter, colors);
  } else if (has_intensity_ && config_.use_color_) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter, colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    pcl::fromROSMsg(*input, pointcloud_pcl);
    filterptRange(pointcloud_pcl, cloud_filter, colors);
  }

  pcl::transformPointCloud(cloud_filter, trans_cloud, tf_matrix.cast<float>());
  std::vector<Eigen::Vector3d> points;
  common::PCL2Eigen(trans_cloud, points);

  m_data.lock();
  if(enable_databuf_ == true){
    data_buf.push(std::make_tuple(tf_matrix, points, colors));
  }else{
    if(data_buf.size()<=1){
      data_buf.push(std::make_tuple(tf_matrix, points, colors));
    }
  }
  m_data.unlock();
}

template <typename PCLPoint>
void VDBGPDFMapper::filterptRange(
    const typename pcl::PointCloud<PCLPoint> &pointcloud_pcl,
    pcl::PointCloud<pcl::PointXYZ> &cloud_filter,
    std::vector<openvdb::Vec3i> &colors) {
    
    cloud_filter.reserve(pointcloud_pcl.size());
    colors.reserve(pointcloud_pcl.size());

    pcl::PointXYZ p;
    double p_radius;
    bool enable_filter = true;
    if (config_.min_scan_range < 0 || config_.max_scan_range < 0)
      enable_filter = false;

    int id = 0;
    for (auto item = pointcloud_pcl.begin(); item != pointcloud_pcl.end();
        item++, id++) {
      if (!(std::isfinite(item->x) && std::isfinite(item->y) &&
            std::isfinite(item->z)))
        continue;
      p.x = (double)item->x;
      p.y = (double)item->y;
      p.z = (double)item->z;
      if (enable_filter) {
        p_radius = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (config_.min_scan_range < p_radius &&
            p_radius < config_.max_scan_range && p.z < config_.max_height) {
          colors.push_back(common::convertColor(pointcloud_pcl.points[id]));
          cloud_filter.push_back(p);
        }
      } else {
        colors.push_back(common::convertColor(pointcloud_pcl.points[id]));
        cloud_filter.push_back(p);
      }
    }
}

void VDBGPDFMapper::saveMap_callback(
  const std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SaveMap::Request> req,
  std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SaveMap::Response> res) {
    std::cout << "=================== SAVE PROCESS START ======================"
              << std::endl;
    double save_map_filter_res = req->filter_res;
    TIC;
    std::string save_map_path = req->path;
    
    m_fullmap.lock();
    auto [vertices, triangles, color_] = gsdf_volume->ExtractTriangleMesh(
        config_.fill_holes_, config_.recon_min_weight_);
    m_fullmap.unlock();

    LOG(INFO) << "Mesh vertices num: " << vertices.size();
    if (vertices.size() == 0) {
      LOG(WARNING) << "No available mesh, please try again later";
    }
    if ((has_rgb_ || has_intensity_) && color_.size() != 0) {
      pcl::PointCloud<pcl::PointXYZRGB> map_cloud;
      for (size_t i = 0; i < vertices.size(); i++) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(vertices[i].x());
        pt.y = static_cast<float>(vertices[i].y());
        pt.z = static_cast<float>(vertices[i].z());
        pt.r = static_cast<uint8_t>(color_[i][0]);
        pt.g = static_cast<uint8_t>(color_[i][1]);
        pt.b = static_cast<uint8_t>(color_[i][2]);
        map_cloud.push_back(pt);
      }
      auto map_ptr = map_cloud.makeShared();
      sensor_msgs::msg::PointCloud2 map_msg1;
      pcl::toROSMsg(map_cloud, map_msg1);
      map_msg1.header.frame_id = retrive_mpose.getWorldframe();
      vdbmap_pub->publish(map_msg1);
      TOC("PUBLISH Color Msg", debug_print_);
      // Writing Point Cloud data to PCD file
      if (save_map_filter_res == 0.0)
        pcl::io::savePCDFileASCII(save_map_path + "vdb_gpdf_mesh.pcd", map_cloud);
      else {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_filtered(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(save_map_filter_res, save_map_filter_res,
                                      save_map_filter_res);
        voxel_grid_filter.setInputCloud(map_ptr);
        voxel_grid_filter.filter(*map_filtered);
        pcl::io::savePCDFileASCII(save_map_path + "vdb_gpdf_mesh.pcd", *map_filtered);
        std::cout << "Original: " << map_cloud.points.size() << " points."
                  << std::endl;
        std::cout << "Filtered: " << map_filtered->points.size() << " points."
                  << std::endl;
      }

    } else {
      pcl::PointCloud<pcl::PointXYZ> map_cloud;
      for (size_t i = 0; i < vertices.size(); i++) {
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(vertices[i].x());
        pt.y = static_cast<float>(vertices[i].y());
        pt.z = static_cast<float>(vertices[i].z());
        map_cloud.push_back(pt);
      }
      auto map_ptr1 = map_cloud.makeShared();
      sensor_msgs::msg::PointCloud2 map_msg2;
      pcl::toROSMsg(map_cloud, map_msg2);
      map_msg2.header.frame_id = retrive_mpose.getWorldframe();
      vdbmap_pub->publish(map_msg2);
      TOC("PUBLISH XYZ Msg", debug_print_);
      // Writing Point Cloud data to PCD file
      if (save_map_filter_res == 0.0)
        pcl::io::savePCDFileASCII(save_map_path + "vdb_gpdf_mesh.pcd", map_cloud);
      else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered(
            new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(save_map_filter_res, save_map_filter_res,
                                      save_map_filter_res);
        voxel_grid_filter.setInputCloud(map_ptr1);
        voxel_grid_filter.filter(*map_filtered);
        pcl::io::savePCDFileASCII(save_map_path + "vdb_gpdf_mesh.pcd", *map_filtered);
        std::cout << "Original: " << map_cloud.points.size() << " points."
                  << std::endl;
        std::cout << "Filtered: " << map_filtered->points.size() << " points."
                  << std::endl;
      }
    }
    LOG(INFO) << "save pcd in path: " << save_map_path + "_vertices.pcd";
    TOC("SAVE PCD", debug_print_);
    
    std::ofstream stream(save_map_path + "vdb_gpdf_mesh.ply");
    size_t num_points = vertices.size();
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << num_points << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    
    stream << "property uchar red" << std::endl;
    stream << "property uchar green" << std::endl;
    stream << "property uchar blue" << std::endl;
    stream << "property uchar alpha" << std::endl;

    stream << "element face " << triangles.size() << std::endl;
    stream << "property list uchar int vertex_indices"
            << std::endl;  // pcl-1.7(ros::kinetic) breaks ply convention by not using "vertex_index"

    stream << "end_header" << std::endl;

    for (size_t vert_idx = 0; vert_idx < vertices.size(); vert_idx++) {
      stream << vertices[vert_idx].x() << " " << vertices[vert_idx].y() << " " << vertices[vert_idx].z();
      int r = static_cast<int>(color_[vert_idx].x());
      int g = static_cast<int>(color_[vert_idx].y());
      int b = static_cast<int>(color_[vert_idx].z());
      int a = static_cast<int>(255);
      // Uint8 prints as character otherwise. :(
      stream << " " << r << " " << g << " " << b << " " << a;
      stream << std::endl;
    }
    
    for (size_t iF = 0; iF < triangles.size(); iF++) {
      stream << "3 ";
      stream << triangles[iF][0] << " "<< triangles[iF][2] << " "<< triangles[iF][1] << " ";
      stream << std::endl;
    }

    res->success = true;
    std::cout << "=================== SAVE PROCESS END ======================"
              << std::endl;
}

void VDBGPDFMapper::pointsQueryMap_callback(
  const std::shared_ptr<vdb_gpdf_mapping_msgs::srv::PointsQueryMap::Request> reqQ,
  std::shared_ptr<vdb_gpdf_mapping_msgs::srv::PointsQueryMap::Response> resS) { 
        
    std::vector<float> queryPoints(reqQ->points.begin(), reqQ->points.end());
    if (queryPoints.empty()) {
      std::cerr << "Input query points are empty. Please check again!\n";
      
    }
    auto validNumberTest = queryPoints.size()%3;
    if (validNumberTest != 0) {
      std::cerr << "Wrong size for the input query points. Please check again!\n";
      
    }

    int N_pts = queryPoints.size()/3;
    std::vector<double> pRes;
    std::vector<int> pObs;
    pRes.resize( N_pts * 8, 0 );
    pObs.resize( N_pts, 0 );

    auto start = std::chrono::high_resolution_clock::now();
    m_fullmap.lock();
    auto queryStatus = gsdf_volume->QueryMap(queryPoints.data(), N_pts, pRes.data(), pObs.data());
    m_fullmap.unlock();
    if (queryStatus == true){
      std::cout << "Finish query of " << N_pts << " query points in ms: " << (std::chrono::high_resolution_clock::now()-start).count()*1E-6 << std::endl << std::flush;
      resS->stamp = node_->get_clock()->now();
      resS->distances.resize(N_pts, 0);
      resS->gradients.resize(N_pts*3, 0);
      resS->if_observed.resize(N_pts, 1);

      for (int index = 0; index < N_pts; index++) {
          int k8 = index * 8;
          if(isinf(pRes[k8])) pRes[k8] = 300; // give a big default distance value for inf distance
          if(isnan(pRes[k8])) pRes[k8] = 300; // give a big default distance value for inf distance
          resS->distances[index] = static_cast<double>(pRes[k8]);
          resS->gradients[(index*3)] = static_cast<double>(pRes[k8 + 1]);
          resS->gradients[(index*3)+1] = static_cast<double>(pRes[k8 + 2]);
          resS->gradients[(index*3)+2] = static_cast<double>(pRes[k8 + 3]);
          resS->if_observed[index] = static_cast<bool>(pObs[index]);
      }
      visualQueriedDistances(queryPoints,pRes);
      visualQueriedGradients(queryPoints,pRes);
    }else{
      std::cerr << "Can not get the query values successfully!\n";
    }
}

void VDBGPDFMapper::sliceQueryMap_callback(
  const std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SliceQueryMap::Request> reqQ,
  std::shared_ptr<vdb_gpdf_mapping_msgs::srv::SliceQueryMap::Response> resS) { 
        
    // xMin, xMax, yMin, yMax, zSlice, resolution
    std::vector<float> queryRanges(reqQ->slice_ranges.begin(), reqQ->slice_ranges.end());
    if (queryRanges.empty()) {
      std::cerr << "Input query ranges are empty. Please check again!\n";
      
    }
    if (queryRanges.size() != 6) {
      std::cerr << "Wrong size for the input query ranges. Please check again!\n";
      
    }
    
    std::vector<float> queryPoints;
    for (double xIdx = queryRanges[0]; xIdx <= queryRanges[1]; xIdx = xIdx + queryRanges[5]) {
      for (double yIdx = queryRanges[2]; yIdx <= queryRanges[3]; yIdx = yIdx + queryRanges[5]) {
        queryPoints.push_back(xIdx);
        queryPoints.push_back(yIdx);
        queryPoints.push_back(queryRanges[4]); 
      }
    }
    
    int N_pts = queryPoints.size()/3;
    std::vector<double> pRes;
    std::vector<int> pObs;
    pRes.resize( N_pts * 8, 0 );
    pObs.resize( N_pts, 0 );

    auto start = std::chrono::high_resolution_clock::now();
    m_fullmap.lock();
    auto queryStatus = gsdf_volume->QueryMap(queryPoints.data(), N_pts, pRes.data(), pObs.data());
    m_fullmap.unlock();
    if (queryStatus == true){
      std::cout << "Finish query of " << N_pts << " query points in ms: " << (std::chrono::high_resolution_clock::now()-start).count()*1E-6 << std::endl << std::flush;
    
      resS->stamp = node_->get_clock()->now();
      resS->distances.resize(N_pts, 0);
      resS->gradients.resize(N_pts*3, 0);
      resS->if_observed.resize(N_pts, 1);
      
      for (int index = 0; index < N_pts; index++) {
          int k8 = index * 8;
          if(isinf(pRes[k8])) pRes[k8] = 300;
          if(isnan(pRes[k8])) pRes[k8] = 300;
          resS->distances[index] = static_cast<double>(pRes[k8]);
          resS->gradients[(index*3)] = static_cast<double>(pRes[k8 + 1]);
          resS->gradients[(index*3)+1] = static_cast<double>(pRes[k8 + 2]);
          resS->gradients[(index*3)+2] = static_cast<double>(pRes[k8 + 3]);
          resS->if_observed[index] = static_cast<bool>(pObs[index]);
      }
      visualQueriedDistances(queryPoints,pRes);
      visualQueriedGradients(queryPoints,pRes);
      
      //return queryStatus;
    }else{
      std::cerr << "Can not get the query values successfully!\n";
      //return queryStatus;
    }
}

void VDBGPDFMapper::visualCameraPose(Eigen::Matrix4d tf_matrix){
    // ready to show camera pose marker, the carema pose p and q
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << tf_matrix(0,0),tf_matrix(0,1),tf_matrix(0,2),tf_matrix(1,0),tf_matrix(1,1),tf_matrix(1,2),tf_matrix(2,0),tf_matrix(2,1),tf_matrix(2,2);
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(rotation_matrix);
    Eigen::Quaterniond qCamera(rotation_vector);
    Eigen::Vector3d pCamera = tf_matrix.block<3, 1>(0, 3);

    // camera direction
    const Eigen::Vector3d oc = Eigen::Vector3d(0.0, 0.0, 0.0);
    const Eigen::Vector3d oc1 = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d pt_oc(qCamera * (0.2 *oc) + pCamera);
    Eigen::Vector3d pt_oc1(qCamera * (0.2 *oc1) + pCamera);
    Eigen::Vector3d camera_direction(pt_oc.x()-pt_oc1.x(),pt_oc.y()-pt_oc1.y(),pt_oc.z()-pt_oc1.z());

    visualization_msgs::msg::Marker cameraMarker;
    cameraMarker.header.frame_id = retrive_mpose.getWorldframe();
    cameraMarker.header.stamp = node_->get_clock()->now();
    cameraMarker.ns = "camera_pose";
    cameraMarker.id = 0;
    cameraMarker.type = visualization_msgs::msg::Marker::ARROW;
    cameraMarker.action = visualization_msgs::msg::Marker::ADD;
    cameraMarker.scale.x = 0.05;
    cameraMarker.scale.y = 0.08;
    cameraMarker.scale.z = 0.1;
    cameraMarker.color.g = 1.0f;
    cameraMarker.color.a = 1.0;
    geometry_msgs::msg::Point p1, p2;
    p1.x = pt_oc.x();
    p1.y = pt_oc.y();
    p1.z = pt_oc.z();
    p2.x = pt_oc1.x();
    p2.y = pt_oc1.y();
    p2.z = pt_oc1.z();
    cameraMarker.points.push_back(p1) ;
    cameraMarker.points.push_back(p2) ;
    camera_marker_pub->publish(cameraMarker);
}

void VDBGPDFMapper::visualQueriedDistances(const std::vector<float> queryPoints, const std::vector<double> pRes){
  if (globalQueryPointsDis_pub->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZI> queryPointsPCL;
    int N_pts = queryPoints.size()/3;
    for (size_t ii = 0; ii < N_pts; ii++) {
      int k3 = ii * 3;
      int k8 = ii * 8;
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(queryPoints[k3]);
      pt.y = static_cast<float>(queryPoints[k3+1]);
      pt.z = static_cast<float>(queryPoints[k3+2]);
      //pt.z = static_cast<float>(pRes[k8]+0.9);
      pt.intensity = static_cast<float>(pRes[k8]);
      if(pRes[k8]==300){ // skip points with bad distance
        continue;
      }else{
        queryPointsPCL.push_back(pt);
      }
    }
    sensor_msgs::msg::PointCloud2 map_msg3;
    pcl::toROSMsg(queryPointsPCL, map_msg3);
    map_msg3.header.frame_id = retrive_mpose.getWorldframe();
    globalQueryPointsDis_pub->publish(map_msg3);
  }
}

void VDBGPDFMapper::visualQueriedGradients(const std::vector<float> queryPoints, const std::vector<double> pRes){
  visualization_msgs::msg::MarkerArray mArray;
  int N_pts = queryPoints.size()/3;
  for(int idx = 0; idx < N_pts; idx++) {
      int k3 = idx * 3;
      int k8 = idx * 8;
      // if you dont care about magnitude, normalize the gradient vector (looks better)
      geometry_msgs::msg::Point start;
      start.x = queryPoints[k3]; 
      start.y = queryPoints[k3+1]; 
      start.z = queryPoints[k3+2];
      float vecLen1 = 0.4; // scales the vector to 40cm ASSUMING it was normalized before
      geometry_msgs::msg::Point end;
      end.x = start.x + pRes[k8+1]*vecLen1; 
      end.y = start.y + pRes[k8+2]*vecLen1; 
      end.z = start.z + pRes[k8+3]*vecLen1;
      float colorGra[] = {0,1,1,1}; // RGBA. Calculate a colormap based on distance to color it according to distance field 
      mArray.markers.push_back(create_arrow(0.04, start, end, idx, colorGra));
  }
  globalQueryPointsGrd_pub->publish(mArray);
}

visualization_msgs::msg::Marker VDBGPDFMapper::create_arrow(float scale, geometry_msgs::msg::Point start, geometry_msgs::msg::Point end,  int idnum, float color[]) {
    visualization_msgs::msg::Marker m;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.header.frame_id = retrive_mpose.getWorldframe();
    m.header.stamp = node_->get_clock()->now();
    m.id = idnum;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = scale; // thickness of the arrow
    m.scale.y = scale*2; // thickness of the base
    m.scale.z = 0.1;
    m.color.r = color[0];
    m.color.g = color[1];
    m.color.b = color[2];
    m.color.a = color[3];

    m.points.push_back(start);
    m.points.push_back(end);
    
    return m;
}

void VDBGPDFMapper::setConfig() {
  // node_->declare_parameter<std::string>("lidar_topic", lidar_topic_);
  // node_->declare_parameter<bool>("debug_print", debug_print_);
  // node_->declare_parameter<bool>("enable_databuf", enable_databuf_);

  // node_->declare_parameter<double>("min_scan_range", config_.min_scan_range);
  // node_->declare_parameter<double>("max_scan_range", config_.max_scan_range);
  // node_->declare_parameter<double>("max_height", config_.max_height);

  // node_->declare_parameter<bool>("fill_holes", config_.fill_holes_);
  // node_->declare_parameter<bool>("use_color", config_.use_color_);
  // node_->declare_parameter<double>("recon_min_weight", config_.recon_min_weight_);
  RCLCPP_INFO(node_->get_logger(), "VDBGPDFMapper initializing.");
  
  node_->declare_parameter<std::string>("lidar_topic", "/odom_lidar");
  node_->declare_parameter<bool>("debug_print", false);
  node_->declare_parameter<bool>("enable_databuf", false);

  node_->declare_parameter<double>("min_scan_range", 0.0);
  node_->declare_parameter<double>("max_scan_range", 0.0);
  node_->declare_parameter<double>("max_height", 0.0);
  node_->declare_parameter<bool>("fill_holes", false);
  node_->declare_parameter<bool>("use_color", false);
  node_->declare_parameter<double>("recon_min_weight", 0.1);

  // Required parameter
  // if (!node_->get_parameter("lidar_topic", lidar_topic_)) {
  //   RCLCPP_FATAL(node_->get_logger(), "Missing required parameter: 'lidar_topic'");
  //   throw std::runtime_error("Missing required parameter: lidar_topic");
  // }

  node_->get_parameter("lidar_topic", lidar_topic_);
  node_->get_parameter("debug_print", debug_print_);
  node_->get_parameter("enable_databuf", enable_databuf_);

  node_->get_parameter("min_scan_range", config_.min_scan_range);
  node_->get_parameter("max_scan_range", config_.max_scan_range);
  node_->get_parameter("max_height", config_.max_height);

  node_->get_parameter("fill_holes", config_.fill_holes_);
  node_->get_parameter("use_color", config_.use_color_);
  node_->get_parameter("recon_min_weight", config_.recon_min_weight_);

  RCLCPP_INFO(node_->get_logger(), "Lidar topic set to: '%s'", lidar_topic_.c_str());

  RCLCPP_INFO(node_->get_logger(), "VDBGPDFMapper initialized successfully.");
}

}  // namespace vdb_gpdf_mapping