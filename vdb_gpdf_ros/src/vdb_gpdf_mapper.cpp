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

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <filesystem>

// our define
#include "timer.h"
#include "utils.h"
#include "vdb_gpdf_mapper.h"
#include <pcl/filters/voxel_grid.h>

namespace vdb_gpdf_mapper {
VDBGPDFMapper::VDBGPDFMapper(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), retrive_mpose(nh, nh_private), gsdf_volume(nh, nh_private){
  setConfig();

  vdbmap_pub = nh_.advertise<sensor_msgs::PointCloud2>("/vdbmap", 10); 
  camera_marker_pub = nh_.advertise<visualization_msgs::Marker>( "/camera_pose", 10 );

  localGPsPoints_pub = nh_.advertise<sensor_msgs::PointCloud2>( "/localGPsPoints", 10 );
  localQueryPointsDis_pub = nh_.advertise<sensor_msgs::PointCloud2>("/localQueryPointsDis", 10);
  globalGPsPoints_pub = nh_.advertise<sensor_msgs::PointCloud2>("/globalGPsPoints", 10);
  globalQueryPointsDis_pub = nh_.advertise<sensor_msgs::PointCloud2>("/globalQueryPointsDis", 10);
  globalQueryPointsGrd_pub = nh_.advertise<visualization_msgs::MarkerArray>("/globalQueryPointsGrd", 10);
  
  points_sub = nh_.subscribe(lidar_topic_, 20, &VDBGPDFMapper::points_callback, this);

  save_map_srv = nh_.advertiseService("/save_map",
                                      &VDBGPDFMapper::saveMap_callback, this);
  points_query_map_srv = nh_.advertiseService("/points_query_map",
                                      &VDBGPDFMapper::pointsQueryMap_callback, this);
  slice_query_map_srv = nh_.advertiseService("/slice_query_map",
                                      &VDBGPDFMapper::sliceQueryMap_callback, this);

  // initial openvdb volume 
  openvdb::initialize();
}

void VDBGPDFMapper::mapIntegrateProcess() {
  while (ros::ok()) {
    m_data.lock();
    if (data_buf.empty()) {
      std::chrono::seconds dura(2);
      m_data.unlock();
      LOG(INFO) << "There is no data now. Please save the result by calling ros srv.";
      std::this_thread::sleep_for(dura);
      continue;
    }

    int total_num = data_buf.size();
    LOG_IF(INFO, debug_print_) << "Total frames in buffer: " << total_num;

    Eigen::Matrix4d tf_matrix = std::get<0>(data_buf.front());
    std::vector<Eigen::Vector3d> points = std::get<1>(data_buf.front());
    std::vector<openvdb::Vec3i> color = std::get<2>(data_buf.front());
    
    std::vector<Eigen::Vector3d> globalGPsPoints;
    std::vector<openvdb::Vec3i> globalGPsPointsColor;
    std::vector<Eigen::Vector3d> localGPsPoints;
    std::vector<Eigen::Vector3d> localQueryPoints;
    std::vector<double> localQueryDis;

    data_buf.pop();
    m_data.unlock();

    Eigen::Vector3d origin = tf_matrix.block<3, 1>(0, 3);
    LOG_IF(INFO, debug_print_) << "Input pointcloud size: " << points.size();
    TIC;

    // Note: simply pick one type of gsdf_volume.Integrate function to run
    // first one is for vdbgpdf, second one is for vdbfusion as comparision
    m_fullmap.lock();
    gsdf_volume.Integrate(points, color, origin,
                          common::WeightFunction::constant_weight, globalGPsPoints, globalGPsPointsColor, 
                          localGPsPoints, localQueryPoints, localQueryDis);
    // gsdf_volume.Integrate(points, color, origin,
    //                       common::WeightFunction::constant_weight);
    m_fullmap.unlock();
    
    double current_time = timer.End();
    total_Integration_time_ = total_Integration_time_ + current_time;
    TOC("full Integrate", debug_print_);
    total_frame_ = total_frame_ + 1;
    LOG(INFO) << "current_time(ms): " << current_time 
              << "; total_time(ms): " << total_Integration_time_ 
              << "; total frame: " << total_frame_ 
              << "; avg time(ms): " << total_Integration_time_/total_frame_;
    
    visualCameraPose(tf_matrix);

    if (localGPsPoints_pub.getNumSubscribers() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB> localGPsPointsPCL;
      for (size_t ii = 0; ii < localGPsPoints.size(); ii++) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(localGPsPoints[ii].x());
        pt.y = static_cast<float>(localGPsPoints[ii].y());
        pt.z = static_cast<float>(localGPsPoints[ii].z());
        pt.r = static_cast<uint8_t>(255);
        pt.g = static_cast<uint8_t>(0);
        pt.b = static_cast<uint8_t>(0);
        localGPsPointsPCL.push_back(pt);
      }
      auto map_ptr3 = localGPsPointsPCL.makeShared();
      sensor_msgs::PointCloud2::Ptr map_msg_ptr3(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*map_ptr3, *map_msg_ptr3);
      map_msg_ptr3->header.frame_id = retrive_mpose.getWorldframe();
      localGPsPoints_pub.publish(*map_msg_ptr3);
    }

    if (localQueryPointsDis_pub.getNumSubscribers() > 0) {
      pcl::PointCloud<pcl::PointXYZI> localQueryPointsPCL;
      for (size_t ii = 0; ii < localQueryPoints.size(); ii++) {
        pcl::PointXYZI pttt;
        pttt.x = static_cast<float>(localQueryPoints[ii].x());
        pttt.y = static_cast<float>(localQueryPoints[ii].y());
        pttt.z = static_cast<float>(localQueryPoints[ii].z());
        pttt.intensity = static_cast<float>(abs(localQueryDis[ii]));
        if(pttt.intensity>=1000) continue;
        localQueryPointsPCL.push_back(pttt);
      }
      auto map_ptr2 = localQueryPointsPCL.makeShared();
      sensor_msgs::PointCloud2::Ptr map_msg_ptr2(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*map_ptr2, *map_msg_ptr2);
      map_msg_ptr2->header.frame_id = retrive_mpose.getWorldframe();
      localQueryPointsDis_pub.publish(*map_msg_ptr2);
    }

    // frame_count_++;  
    // if (frame_count_ % save_interval_ == 0){
    //   saved_poses_.emplace_back(frame_count_/10, tf_matrix);
    //   std::cout << "Saved frame! " << frame_count_ << std::endl;
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //   for (const auto& point : localGPsPoints) {
    //       Eigen::Vector3d dir = point - origin;
    //       double length = dir.norm();
    //       if (length < 1e-6) continue;

    //       dir.normalize();
    //       int num_steps = static_cast<int>(length / step_size_);
    //       for (int i = 0; i < num_steps; ++i) {
    //           double t = i * step_size_;
    //           Eigen::Vector3d sample = origin + t * dir;
    //           free_cloud->points.emplace_back(sample.x(), sample.y(), sample.z());
    //       }

    //       surface_cloud->points.emplace_back(point.x(), point.y(), point.z());
    //   }

    //   // Create voxel grid filter
    //   pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    //   voxel_filter.setInputCloud(free_cloud);
    //   voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);  // grid size in meters

    //   // Output cloud
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    //   voxel_filter.filter(*free_cloud_downsampled);

    //   // Replace free_cloud with the downsampled one
    //   free_cloud = free_cloud_downsampled;

    //   std::string output_dir = "/home/lan/ply_test/";
    //   std::filesystem::create_directories(output_dir);

    //   std::stringstream fs_name, sp_name;
    //   fs_name << output_dir << "free_space_" << std::setw(4) << std::setfill('0') << frame_count_/10 << ".ply";
    //   sp_name << output_dir << "surface_points_" << std::setw(4) << std::setfill('0') << frame_count_/10 << ".ply";

    //   if (pcl::io::savePLYFileASCII(fs_name.str(), *free_cloud) < 0)
    //     std::cerr << "Failed to save " << fs_name.str() << std::endl;

    //   if (pcl::io::savePLYFileASCII(sp_name.str(), *surface_cloud) < 0)
    //     std::cerr << "Failed to save " << sp_name.str() << std::endl;

    //   std::cout << "Saved frame " << frame_count_ << ": "
    //             << fs_name.str() << ", " << sp_name.str() << std::endl;

    //   savePosesToCSV("/home/lan/ply_test/poses.csv");
    // }

    if (globalGPsPoints_pub.getNumSubscribers() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB> globalGPsPointsPCL;
      for (size_t ii = 0; ii < globalGPsPoints.size(); ii++) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(globalGPsPoints[ii].x());
        pt.y = static_cast<float>(globalGPsPoints[ii].y());
        pt.z = static_cast<float>(globalGPsPoints[ii].z());
        pt.r = static_cast<uint8_t>(globalGPsPointsColor[ii][0]);
        pt.g = static_cast<uint8_t>(globalGPsPointsColor[ii][1]);
        pt.b = static_cast<uint8_t>(globalGPsPointsColor[ii][2]);
        globalGPsPointsPCL.push_back(pt);
      }
      auto map_ptr1 = globalGPsPointsPCL.makeShared();
      sensor_msgs::PointCloud2::Ptr map_msg_ptr1(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*map_ptr1, *map_msg_ptr1);
      map_msg_ptr1->header.frame_id = retrive_mpose.getWorldframe();
      globalGPsPoints_pub.publish(*map_msg_ptr1);
    }

    if (debug_print_)
      std::cout << "------------------------------------------------------"
                << std::endl;
  }
}

void VDBGPDFMapper::savePosesToCSV(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open pose file: " << filename << std::endl;
        return;
    }

    // Write header
    file << "frame_id,tx,ty,tz,qx,qy,qz,qw\n";

    for (const auto& frame_pose : saved_poses_) {
        int frame_id = frame_pose.first;
        const Eigen::Matrix4d& pose = frame_pose.second;

        // Extract translation
        Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

        // Extract rotation
        Eigen::Matrix3d rotation_matrix = pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation_matrix);
        quat.normalize();  // Always normalize to be safe

        file << frame_id << ","
             << translation.x() << "," << translation.y() << "," << translation.z() << ","
             << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << "\n";
    }

    file.close();
    std::cout << "Saved " << saved_poses_.size() << " poses to " << filename << std::endl;
}

void VDBGPDFMapper::points_callback(
    const sensor_msgs::PointCloud2::Ptr &input) {
  Eigen::Matrix4d tf_matrix = Eigen::Matrix4d::Identity();
  if (!retrive_mpose.lookUpTransformfromSource(input, tf_matrix)) {
    // LOG(WARNING) << "Didn't find the pair pose, skip this message";
    return;
  }
  // Horrible hack fix to fix color parsing colors in PCL.
  for (size_t d = 0; d < input->fields.size(); ++d) {
    if (input->fields[d].name == std::string("rgb")) {
      input->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
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

bool VDBGPDFMapper::saveMap_callback(
    vdb_gpdf_msgs::SaveMap::Request &req,
    vdb_gpdf_msgs::SaveMap::Response &res) {
    std::cout << "=================== SAVE PROCESS START ======================"
              << std::endl;
    double save_map_filter_res = req.filter_res;
    TIC;
    std::string save_map_path = req.path;
    
    m_fullmap.lock();
    auto [vertices, triangles, color_] = gsdf_volume.ExtractTriangleMesh(
        config_.fill_holes_, config_.recon_min_weight_);
    m_fullmap.unlock();

    LOG(INFO) << "Mesh vertices num: " << vertices.size();
    if (vertices.size() == 0) {
      LOG(WARNING) << "No available mesh, please try again later";
      return false;
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
      sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*map_ptr, *map_msg_ptr);
      map_msg_ptr->header.frame_id = retrive_mpose.getWorldframe();
      vdbmap_pub.publish(*map_msg_ptr);
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
      auto map_ptr = map_cloud.makeShared();
      sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*map_ptr, *map_msg_ptr);
      map_msg_ptr->header.frame_id = retrive_mpose.getWorldframe();
      vdbmap_pub.publish(*map_msg_ptr);
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
        voxel_grid_filter.setInputCloud(map_ptr);
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

    res.success = true;
    std::cout << "=================== SAVE PROCESS END ======================"
              << std::endl;

    return res.success;
}

bool VDBGPDFMapper::pointsQueryMap_callback(
    vdb_gpdf_msgs::PointsQueryMap::Request &reqQ,
    vdb_gpdf_msgs::PointsQueryMap::Response &resS) { 
        
    std::vector<float> queryPoints(reqQ.points.begin(), reqQ.points.end());
    if (queryPoints.empty()) {
      std::cerr << "Input query points are empty. Please check again!\n";
      return false;
    }
    auto validNumberTest = queryPoints.size()%3;
    if (validNumberTest != 0) {
      std::cerr << "Wrong size for the input query points. Please check again!\n";
      return false;
    }

    int N_pts = queryPoints.size()/3;
    std::vector<double> pRes;
    std::vector<int> pObs;
    pRes.resize( N_pts * 8, 0 );
    pObs.resize( N_pts, 0 );

    auto start = std::chrono::high_resolution_clock::now();
    m_fullmap.lock();
    auto queryStatus = gsdf_volume.QueryMap(queryPoints.data(), N_pts, pRes.data(), pObs.data());
    m_fullmap.unlock();
    if (queryStatus == true){
      std::cout << "Finish query of " << N_pts << " query points in ms: " << (std::chrono::high_resolution_clock::now()-start).count()*1E-6 << std::endl << std::flush;
      resS.stamp = ros::Time::now();
      resS.distances.resize(N_pts, 0);
      resS.gradients.resize(N_pts*3, 0);
      resS.if_observed.resize(N_pts, 1);

      for (int index = 0; index < N_pts; index++) {
          int k8 = index * 8;
          if(isinf(pRes[k8])) pRes[k8] = 300; // give a big default distance value for inf distance
          if(isnan(pRes[k8])) pRes[k8] = 300; // give a big default distance value for inf distance
          resS.distances[index] = static_cast<double>(pRes[k8]);
          resS.gradients[(index*3)] = static_cast<double>(pRes[k8 + 1]);
          resS.gradients[(index*3)+1] = static_cast<double>(pRes[k8 + 2]);
          resS.gradients[(index*3)+2] = static_cast<double>(pRes[k8 + 3]);
          resS.if_observed[index] = static_cast<bool>(pObs[index]);
      }
      visualQueriedDistances(queryPoints,pRes);
      visualQueriedGradients(queryPoints,pRes);

      return queryStatus;
    }else{
      std::cerr << "Can not get the query values successfully!\n";
      return queryStatus;
    }
}

bool VDBGPDFMapper::sliceQueryMap_callback(
    vdb_gpdf_msgs::SliceQueryMap::Request &reqQ,
    vdb_gpdf_msgs::SliceQueryMap::Response &resS) { 
        
    // xMin, xMax, yMin, yMax, zSlice, resolution
    std::vector<float> queryRanges(reqQ.slice_ranges.begin(), reqQ.slice_ranges.end());
    if (queryRanges.empty()) {
      std::cerr << "Input query ranges are empty. Please check again!\n";
      return false;
    }
    if (queryRanges.size() != 6) {
      std::cerr << "Wrong size for the input query ranges. Please check again!\n";
      return false;
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
    auto queryStatus = gsdf_volume.QueryMap(queryPoints.data(), N_pts, pRes.data(), pObs.data());
    m_fullmap.unlock();
    if (queryStatus == true){
      std::cout << "Finish query of " << N_pts << " query points in ms: " << (std::chrono::high_resolution_clock::now()-start).count()*1E-6 << std::endl << std::flush;
    
      resS.stamp = ros::Time::now();
      resS.distances.resize(N_pts, 0);
      resS.gradients.resize(N_pts*3, 0);
      resS.if_observed.resize(N_pts, 1);
      
      for (int index = 0; index < N_pts; index++) {
          int k8 = index * 8;
          if(isinf(pRes[k8])) pRes[k8] = 300;
          if(isnan(pRes[k8])) pRes[k8] = 300;
          resS.distances[index] = static_cast<double>(pRes[k8]);
          resS.gradients[(index*3)] = static_cast<double>(pRes[k8 + 1]);
          resS.gradients[(index*3)+1] = static_cast<double>(pRes[k8 + 2]);
          resS.gradients[(index*3)+2] = static_cast<double>(pRes[k8 + 3]);
          resS.if_observed[index] = static_cast<bool>(pObs[index]);
      }
      visualQueriedDistances(queryPoints,pRes);
      visualQueriedGradients(queryPoints,pRes);
      
      return queryStatus;
    }else{
      std::cerr << "Can not get the query values successfully!\n";
      return queryStatus;
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

    visualization_msgs::Marker cameraMarker;
    cameraMarker.header.frame_id = retrive_mpose.getWorldframe();
    cameraMarker.header.stamp = ros::Time::now();
    cameraMarker.ns = "camera_pose";
    cameraMarker.id = 0;
    cameraMarker.type = visualization_msgs::Marker::ARROW;
    cameraMarker.action = visualization_msgs::Marker::ADD;
    cameraMarker.scale.x = 0.05;
    cameraMarker.scale.y = 0.08;
    cameraMarker.scale.z = 0.1;
    cameraMarker.color.g = 1.0f;
    cameraMarker.color.a = 1.0;
    geometry_msgs::Point p1, p2;
    p1.x = pt_oc.x();
    p1.y = pt_oc.y();
    p1.z = pt_oc.z();
    p2.x = pt_oc1.x();
    p2.y = pt_oc1.y();
    p2.z = pt_oc1.z();
    cameraMarker.points.push_back(p1) ;
    cameraMarker.points.push_back(p2) ;
    camera_marker_pub.publish(cameraMarker);
}

void VDBGPDFMapper::visualQueriedDistances(const std::vector<float> queryPoints, const std::vector<double> pRes){
  if (globalQueryPointsDis_pub.getNumSubscribers() > 0) {
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
    auto map_ptr = queryPointsPCL.makeShared();
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = retrive_mpose.getWorldframe();
    globalQueryPointsDis_pub.publish(*map_msg_ptr);
  }
}

void VDBGPDFMapper::visualQueriedGradients(const std::vector<float> queryPoints, const std::vector<double> pRes){
  visualization_msgs::MarkerArray mArray;
  int N_pts = queryPoints.size()/3;
  for(int idx = 0; idx < N_pts; idx++) {
      int k3 = idx * 3;
      int k8 = idx * 8;
      // if you dont care about magnitude, normalize the gradient vector (looks better)
      geometry_msgs::Point start;
      start.x = queryPoints[k3]; 
      start.y = queryPoints[k3+1]; 
      start.z = queryPoints[k3+2];
      float vecLen1 = 0.4; // scales the vector to 40cm ASSUMING it was normalized before
      geometry_msgs::Point end;
      end.x = start.x + pRes[k8+1]*vecLen1; 
      end.y = start.y + pRes[k8+2]*vecLen1; 
      end.z = start.z + pRes[k8+3]*vecLen1;
      float colorGra[] = {0,1,1,1}; // RGBA. Calculate a colormap based on distance to color it according to distance field 
      mArray.markers.push_back(create_arrow(0.04, start, end, idx, colorGra));
  }
  globalQueryPointsGrd_pub.publish(mArray);
}

visualization_msgs::Marker VDBGPDFMapper::create_arrow(float scale, geometry_msgs::Point start, geometry_msgs::Point end,  int idnum, float color[]) {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::ADD;
    m.header.frame_id = retrive_mpose.getWorldframe();
    m.header.stamp = ros::Time::now();
    m.id = idnum;
    m.type = visualization_msgs::Marker::ARROW;
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
  nh_private_.getParam("lidar_topic", lidar_topic_);
  nh_private_.getParam("debug_print", debug_print_);
  nh_private_.getParam("enable_databuf", enable_databuf_);

  nh_private_.getParam("min_scan_range", config_.min_scan_range);
  nh_private_.getParam("max_scan_range", config_.max_scan_range);
  nh_private_.getParam("max_height", config_.max_height);

  nh_private_.getParam("fill_holes", config_.fill_holes_);
  nh_private_.getParam("use_color", config_.use_color_);
  nh_private_.getParam("recon_min_weight", config_.recon_min_weight_);

  LOG(INFO) << "==========> Setting Config Success, start for running";
}

}  // namespace vdb_gpdf_mapper