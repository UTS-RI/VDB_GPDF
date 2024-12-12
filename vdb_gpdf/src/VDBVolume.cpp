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

#include "VDBVolume.h"

// OpenVDB
#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/math/Transform.h>
#include <openvdb/tree/LeafManager.h>
#include <openvdb/tree/LeafNode.h>
#include <openvdb/tree/Tree.h>
#include <openvdb/math/Coord.h>
#include <openvdb/tree/LeafNodeBool.h>
#include <openvdb/tools/Activate.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>
#include "omp.h"
#include "timer.h"
#include "utils.h"

namespace {
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
  return {v_wf.x(), v_wf.y(), v_wf.z()};
}

openvdb::Vec3i BlendColors(const openvdb::Vec3i &color1, float weight1,
                           const openvdb::Vec3i &color2, float weight2) {
  float weight_sum = weight1 + weight2;
  weight1 /= weight_sum;
  weight2 /= weight_sum;
  return {static_cast<int>(round(color1[0] * weight1 + color2[0] * weight2)),
          static_cast<int>(round(color1[1] * weight1 + color2[1] * weight2)),
          static_cast<int>(round(color1[2] * weight1 + color2[2] * weight2))};
}
} // namespace

// Taken from <open3d/utility/Eigen.h> for marching cube 
namespace {
template <typename T>
struct hash_eigen {
  std::size_t operator()(T const &matrix) const {
    size_t seed = 0;
    for (int i = 0; i < (int)matrix.size(); i++) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};
} // namespace

namespace openvdb {
static const openvdb::Coord shift[8] = {
    openvdb::Coord(0, 0, 0), openvdb::Coord(1, 0, 0), openvdb::Coord(1, 1, 0),
    openvdb::Coord(0, 1, 0), openvdb::Coord(0, 0, 1), openvdb::Coord(1, 0, 1),
    openvdb::Coord(1, 1, 1), openvdb::Coord(0, 1, 1),
};
} // namespace openvdb

namespace vdb_gpdf {

inline double kf_se(double r, double a){return exp(-r*r*a*0.5);} // se kernel
inline double kf_se1(double r, double dx,double a){return -dx*a*exp(-r*r*a*0.5);} // 1 derivative of se kernel
inline double kf_Se2(double r, double dx1, double dx2, double delta, double a){ // 2 derivative of se kernel
  return (dx1*dx2*a*a-delta*a)*exp(-r*r*a*0.5);}

inline double kf_ma(double r, double a) {return (1.0+a*r)*exp(-a*r);} // matern kernel
inline double kf_ma1(double r, double dx, double a) {return a*a*dx*exp(-a*r);} // 1 derivative of se kernel
inline double kf_ma2(float r, float dx1, float dx2, float delta, float a){ // 2 derivative of se kernel
  return a*a*(delta-a*dx1*dx2/r)*exp(-a*r);}

VDBVolume::VDBVolume(const ros::NodeHandle &nh, 
                     const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private){

  nh_private_.getParam("debug_print", debug_print_);
  nh_private_.getParam("use_color", use_color_);
  nh_private_.getParam("sdf_trunc", sdf_trunc_);
  nh_private_.getParam("space_carving", space_carving_);

  nh_private_.getParam("distance_method", distance_method_);
  
  nh_private_.getParam("sensor_type", sensor_type_);
  nh_private_.getParam("voxel_size_local", voxel_size_lo_);
  nh_private_.getParam("voxel_overlapping", voxel_overlapping_);
  nh_private_.getParam("voxel_downsample", voxel_downsample_);
  nh_private_.getParam("voxel_size_global", voxel_size_gl_);

  nh_private_.getParam("variance_method", variance_method_);
  nh_private_.getParam("variance_cap", variance_cap_);
  nh_private_.getParam("variance_on_surface", variance_on_surface_);
  
  nh_private_.getParam("surface_normal_method", surface_normal_method_);
  nh_private_.getParam("surface_normal_num", surface_normal_num_);
  nh_private_.getParam("surface_value", surface_value_);
  nh_private_.getParam("query_iterval", query_iterval_);
  nh_private_.getParam("query_trunc_in", query_trunc_in_);
  nh_private_.getParam("query_trunc_out", query_trunc_out_);
  nh_private_.getParam("freespace_iterval", freespace_iterval_);
  nh_private_.getParam("freespace_trunc_out", freespace_trunc_out_);
  nh_private_.getParam("query_downsample", query_downsample_);

  nh_private_.getParam("map_lambda_scale", map_lambda_scale_);
  nh_private_.getParam("map_noise", map_noise_);
  nh_private_.getParam("color_scale", color_scale_);
  nh_private_.getParam("smooth_param", smooth_param_);

  gsdf_ = openvdb::FloatGrid::create(-100); // default value for each voxel
  gsdf_->setName("D(x): signed distance grid");
  gsdf_->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_gl_));
  gsdf_->setGridClass(openvdb::GRID_LEVEL_SET);

  weights_ = openvdb::FloatGrid::create(0.0f);
  weights_->setName("W(x): weights grid");
  weights_->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_gl_));
  weights_->setGridClass(openvdb::GRID_UNKNOWN);

  colors_ = openvdb::Vec3IGrid::create(openvdb::Vec3I(0, 0, 0));
  colors_->setName("C(x): colors grid");
  colors_->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_gl_));
  colors_->setGridClass(openvdb::GRID_UNKNOWN);
}

void VDBVolume::Integrate(
    const std::vector<Eigen::Vector3d> &points,
    const std::vector<openvdb::Vec3i> &colors, const Eigen::Vector3d &origin,
    const std::function<float(float)> &weighting_function, 
    std::vector<Eigen::Vector3d> &globalGPsPoints, //all GP points for all current active cubes
    std::vector<openvdb::Vec3i> &globalGPsPointsColor, // color of all GP points
    std::vector<Eigen::Vector3d> &localGPsPoints, // current points for current local GPDF
    std::vector<Eigen::Vector3d> &localQueryPoints, // local testing points in frustum
    std::vector<double> &localQueryDis // distances of local testing points in frustum
    ){
    TIC;
    if (points.empty()) {
      std::cerr << "Input pointcloud is empty! \n";
      return;
    }
    bool has_colors = !colors.empty();
    if (has_colors && points.size() != colors.size()) {
      std::cerr << "Input pointcloud and its color have to be the same size! \n";
      return;
    }

    std::vector<std::shared_ptr<OnGPDF>> localGPs;
    std::vector<Eigen::Vector3d> localGPsCenters;
    
    float localGridSize = voxel_size_lo_;
    CreateLocalDisantceField(points, colors, localGridSize, localGPs, localGPsCenters, localGPsPoints);
    LOG_IF(INFO, debug_print_) << "local_GPs_number: " << localGPsCenters.size() << " ;" << std::endl;
    LOG_IF(INFO, debug_print_) << "local_GPs_points_number: " << localGPsPoints.size() << " ;" << std::endl;
    TOC("local GP ready and train", debug_print_);
    
    std::vector<int> localQueryPointsSig;
    GenerateVoxelsToUpdate(localGPsPoints, origin, localQueryPoints, localQueryPointsSig);
    LOG_IF(INFO, debug_print_) << "all points to update: " << localQueryPoints.size() << " ;" << std::endl;
    TOC("genarate voxels to be updated", debug_print_);
    
    auto [disAllTesting, varAllTesting, rrAllTesting, ggAllTesting, bbAllTesting] = 
    ComputeVoxelsDistances(localGPs, localGPsCenters, localQueryPoints, localQueryPointsSig);

    localQueryDis = disAllTesting;
    Fusion(localQueryPoints, disAllTesting, varAllTesting, 
          rrAllTesting, ggAllTesting, bbAllTesting, weighting_function);

    PrepareGlobalDistanceField(globalGPsPoints, globalGPsPointsColor);
}

void VDBVolume::Integrate(
    const std::vector<Eigen::Vector3d> &points,
    const std::vector<openvdb::Vec3i> &colors, const Eigen::Vector3d &origin,
    const std::function<float(float)> &weighting_function) {
  if (points.empty()) {
    std::cerr << "PointCloud provided is empty\n";
    return;
  }
  bool has_colors = !colors.empty();
  if (has_colors && points.size() != colors.size()) {
    std::cerr
        << "PointCloud and ColorCloud provided do not have the same size\n";
    return;
  }

  // Get some variables that are common to all rays
  const openvdb::math::Transform &xform = gsdf_->transform();
  const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());

  // Get the "unsafe" version of the grid accessors
  auto tsdf_acc = gsdf_->getUnsafeAccessor();
  auto weights_acc = weights_->getUnsafeAccessor();
  auto colors_acc = colors_->getUnsafeAccessor();

  // Iterate points
  size_t points_size = points.size();
  for (size_t i = 0; i < points_size; ++i) {
    // Get the direction from the sensor origin to the point and normalize it
    const auto point = points[i];
    const Eigen::Vector3d direction = point - origin;
    openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
    dir.normalize();

    // Truncate the Ray before and after the source unless space_carving_ is
    // specified.
    const auto depth = static_cast<float>(direction.norm());
    const float t0 = space_carving_ ? 0.0f : depth - sdf_trunc_;
    const float t1 = depth + sdf_trunc_;

    // Create one DDA per ray(per thread), the ray must operate on voxel grid
    // coordinates.
    const auto ray =
        openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*gsdf_);
    openvdb::math::DDA<decltype(ray)> dda(ray);
    do {
      const auto voxel = dda.voxel();
      const auto voxel_center = GetVoxelCenter(voxel, xform);
      const auto sdf = ComputeSDF(origin, point, voxel_center);
      if (sdf > -sdf_trunc_) {
        const float tsdf = std::min(sdf_trunc_, sdf);
        const float weight = weighting_function(sdf);
        const float last_weight = weights_acc.getValue(voxel);
        const float last_tsdf = tsdf_acc.getValue(voxel);
        const float new_weight = weight + last_weight;
        const float new_tsdf =
            (last_tsdf * last_weight + tsdf * weight) / (new_weight);
        tsdf_acc.setValue(voxel, new_tsdf);
        weights_acc.setValue(voxel, new_weight);
        if (has_colors) {
          const auto color = colors_acc.getValue(voxel);
          openvdb::Vec3i new_color =
              BlendColors(color, last_weight, colors[i], weight);
          colors_acc.setValue(voxel, new_color);
        }
      }
    } while (dda.step());
  }
}

void VDBVolume::CreateLocalDisantceField(
  const std::vector<Eigen::Vector3d> &points,
  const std::vector<openvdb::Vec3i> &colors, 
  float localGridSize, 
  std::vector<std::shared_ptr<OnGPDF>> &localGPs, 
  std::vector<Eigen::Vector3d> &localGPsCenters,
  std::vector<Eigen::Vector3d> &localGPsPoints){
  openvdb::FloatGrid::Ptr local_pointsVDB;
  openvdb::Vec3IGrid::Ptr local_colorsVDB;
  local_pointsVDB = openvdb::FloatGrid::create(sdf_trunc_);
  local_pointsVDB->setName("LP(x): local points grid");
  
  local_pointsVDB->setTransform(
      openvdb::math::Transform::createLinearTransform(localGridSize));
  local_pointsVDB->setGridClass(openvdb::GRID_LEVEL_SET);
  const openvdb::math::Transform &xform_local = local_pointsVDB->transform();
  auto local_points_acc = local_pointsVDB->getUnsafeAccessor();

  local_colorsVDB = openvdb::Vec3IGrid::create(openvdb::Vec3I(0, 0, 0));
  local_colorsVDB->setName("LC(x): local colors grid");
  local_colorsVDB->setTransform(
      openvdb::math::Transform::createLinearTransform(localGridSize));
  local_colorsVDB->setGridClass(openvdb::GRID_UNKNOWN);
  auto local_colors_acc = local_colorsVDB->getUnsafeAccessor();

  // Iterate points and colors
  size_t points_size = points.size();
  for (size_t i = 0; i < points_size; ++i) {
    const auto point = points[i];
    openvdb::Vec3d voxeltemp(point.x(),point.y(),point.z()); 
    voxeltemp = xform_local.worldToIndex(voxeltemp);
    openvdb::math::Coord localPoint(voxeltemp.x(),voxeltemp.y(),voxeltemp.z()); 
    local_points_acc.setValue(localPoint, 1.0); 
    local_colors_acc.setValue(localPoint, colors[i]); 
  }

  for (openvdb::FloatGrid::TreeType::LeafIter iterL = local_pointsVDB->tree().beginLeaf(); iterL; ++iterL) {
      auto leaf = iterL.getLeaf();
      std::vector<Eigen::Vector3d> leafVoxels;
      std::vector<openvdb::Vec3i> leafVoxelsColors;
      const openvdb::CoordBBox bboxTest = iterL->getNodeBoundingBox();
      const auto coordBBoxIntO = xform_local.indexToWorld(bboxTest);
      
      if (leaf->onVoxelCount()>0){
        if(voxel_overlapping_ > 0){
          float overlapping_area = localGridSize*voxel_overlapping_; // 8*8*8 for each leaf, so overlapping is 8/2+1=5
          for (auto xb = -overlapping_area; xb <= overlapping_area; xb = xb + localGridSize) {
            for (auto yb = -overlapping_area; yb <= overlapping_area; yb = yb + localGridSize) {
              for (auto zb = -overlapping_area; zb <= overlapping_area; zb = zb + localGridSize) {
                openvdb::Vec3R leafCenter = coordBBoxIntO.getCenter();
                openvdb::math::Vec3d cubePoints(leafCenter.x()+xb,leafCenter.y()+yb,leafCenter.z()+zb);
                openvdb::math::Vec3d cubePointsI = xform_local.worldToIndex(cubePoints);
                openvdb::math::Coord voxelll(cubePointsI.x(),cubePointsI.y(),cubePointsI.z()); 
                if (local_points_acc.isValueOn(voxelll)){
                  Eigen::Vector3d tmp123(cubePoints.x(),cubePoints.y(),cubePoints.z());
                  leafVoxels.push_back(tmp123);
                  openvdb::Vec3i tempColor = local_colors_acc.getValue(voxelll); 
                  leafVoxelsColors.push_back(tempColor);
                }       
              }
            }
          }
        }else{
          for (auto iterLV = leaf->beginValueOn(); iterLV; ++iterLV){
            //auto& value = *iterLV;
            auto iterValueOnWorld = xform_local.indexToWorld(iterLV.getCoord());
            Eigen::Vector3d tmp123(iterValueOnWorld.x(),iterValueOnWorld.y(),iterValueOnWorld.z());
            leafVoxels.push_back(tmp123);
            //localGPsPoints.push_back(tmp123);
            openvdb::Vec3i tempColor = local_colors_acc.getValue(iterLV.getCoord()); 
            leafVoxelsColors.push_back(tempColor);
          }
        }
        
        auto downRate = voxel_downsample_; 
        int DownPoints = int(leafVoxels.size()/downRate); 
        std::vector<Eigen::Vector3d> leafVoxelsDown;
        std::vector<openvdb::Vec3i> leafVoxelsColorsDown;   
        for (int ii = 0; ii < DownPoints; ii++) {
          Eigen::Vector3d tmp123(leafVoxels[ii*downRate].x(),leafVoxels[ii*downRate].y(),leafVoxels[ii*downRate].z());
          openvdb::Vec3i tmpColor(leafVoxelsColors[ii*downRate][0],leafVoxelsColors[ii*downRate][1],leafVoxelsColors[ii*downRate][2]);
          leafVoxelsDown.push_back(tmp123);
          leafVoxelsColorsDown.push_back(tmpColor);
          localGPsPoints.push_back(tmp123);
        }
        
        if(leafVoxelsDown.size()>0){
          std::shared_ptr<OnGPDF> gp(new OnGPDF(leafVoxelsDown,leafVoxelsColorsDown,distance_method_,map_lambda_scale_,map_noise_,color_scale_));
          openvdb::Vec3R leafCenter = coordBBoxIntO.getCenter();
          Eigen::Vector3d gpCenter(leafCenter.x(),leafCenter.y(),leafCenter.z());
          std::vector<Eigen::Vector3d>::iterator iter=std::find(localGPsCenters.begin(),localGPsCenters.end(),gpCenter);
          if(iter == localGPsCenters.end())
          {
            localGPsCenters.push_back(gpCenter);
            localGPs.push_back(gp);
          } else {
            auto idxToUpdate = std::distance(localGPsCenters.begin(),iter);
            localGPs[idxToUpdate] = gp;
          }
        }
      }
      leaf->setValuesOff();
  }
  
  for (size_t gpIdx = 0; gpIdx < localGPs.size(); gpIdx ++) {
    std::vector<Eigen::Vector3d> leafVoxelss;
    std::vector<openvdb::Vec3i> leafVoxelsColorss;

    auto loopGP = localGPs[gpIdx];
    leafVoxelss = loopGP->getPoints();
    leafVoxelsColorss = loopGP->getColors();
    
    std::shared_ptr<OnGPDF> gp(new OnGPDF(leafVoxelss,leafVoxelsColorss,distance_method_,map_lambda_scale_,map_noise_,color_scale_));
    gp->train(leafVoxelss,leafVoxelsColorss,use_color_);
    localGPs[gpIdx] = gp;
  }
}

void VDBVolume::GenerateVoxelsToUpdate(
  const std::vector<Eigen::Vector3d> &localGPsPoints,
  const Eigen::Vector3d &origin,
  std::vector<Eigen::Vector3d> &voxelsToUpdate,
  std::vector<int> &voxelsToUpdateSig){
  
  const openvdb::math::Transform &xform_temp = gsdf_->transform();
  auto gsdf_temp_acc = gsdf_->getAccessor();
  
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  if(surface_normal_method_ == 0){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudN(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t targetIdx = 0; targetIdx < localGPsPoints.size(); targetIdx++) {
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(localGPsPoints[targetIdx].x());
      pt.y = static_cast<float>(localGPsPoints[targetIdx].y());
      pt.z = static_cast<float>(localGPsPoints[targetIdx].z());
      cloudN->push_back(pt);
    }
    
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nNormal;//omp
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeN(new pcl::search::KdTree<pcl::PointXYZ>());
    nNormal.setNumberOfThreads(12);//openmp threads
    nNormal.setViewPoint(origin.x(),origin.y(),origin.z());
    nNormal.setInputCloud(cloudN);
    nNormal.setSearchMethod(treeN);
    nNormal.setKSearch(surface_normal_num_);
    nNormal.compute(*normals);
  }

  std::vector<Eigen::Vector3d> voxelsCloseSurface;
  std::vector<int> voxelsCloseSurfaceSig;

  size_t localGPsPoints_size = localGPsPoints.size();
  for (size_t Idx = 0; Idx < localGPsPoints_size; Idx = Idx+1) {
    Eigen::Vector3d normalP;
    if(surface_normal_method_ == 0){
      normalP.x() = normals->points[Idx].normal_x;
      normalP.y() = normals->points[Idx].normal_y;
      normalP.z() = normals->points[Idx].normal_z;
      normalP.normalize();
    }else{
      normalP = origin - localGPsPoints[Idx];
      normalP.normalize();
    }

    voxelsCloseSurface.push_back(localGPsPoints[Idx]);
    voxelsCloseSurfaceSig.push_back(surface_value_); // -1 for infront 1 for behind

    for (auto j = 1; j <= query_trunc_out_; j++) { // behind the surface
      float local_intervel = query_iterval_*j;
      Eigen::Vector3d inPoints(localGPsPoints[Idx].x()-normalP.x()*local_intervel,
      localGPsPoints[Idx].y()-normalP.y()*local_intervel,
      localGPsPoints[Idx].z()-normalP.z()*local_intervel);
      voxelsCloseSurface.push_back(inPoints);
      voxelsCloseSurfaceSig.push_back(1);
    }
    for (auto j = 1; j <= query_trunc_in_; j++) { // infront the surface
      float local_intervel = query_iterval_*j;
      Eigen::Vector3d outPoints(localGPsPoints[Idx].x()+normalP.x()*local_intervel,
      localGPsPoints[Idx].y()+normalP.y()*local_intervel,
      localGPsPoints[Idx].z()+normalP.z()*local_intervel);
      voxelsCloseSurface.push_back(outPoints);
      voxelsCloseSurfaceSig.push_back(-1);
    }
  }
  LOG_IF(INFO, debug_print_) << "close surface points: " << voxelsCloseSurface.size() << " ;" << std::endl;
 
  auto voxel_size_gl_temp1 = freespace_iterval_;
  for (size_t Idx = 0; Idx < localGPsPoints_size; Idx = Idx+1) {
    Eigen::Vector3d normalP = origin - localGPsPoints[Idx];
    double distt = normalP.norm();
    //if (distt > 15) continue;
    int number = int(distt/voxel_size_gl_temp1);
    normalP.normalize();
    if (number>=1){
      for (auto Id = 1; Id < number; Id++) {
        Eigen::Vector3d outPoints(localGPsPoints[Idx].x()+normalP.x()*voxel_size_gl_temp1*Id,
          localGPsPoints[Idx].y()+normalP.y()*voxel_size_gl_temp1*Id,
          localGPsPoints[Idx].z()+normalP.z()*voxel_size_gl_temp1*Id);

        openvdb::math::Vec3d voxeltemp1(outPoints.x(),outPoints.y(),outPoints.z()); 
        openvdb::math::Vec3d v_wf1 = xform_temp.worldToIndex(voxeltemp1);
        openvdb::math::Coord voxel1(v_wf1.x(),v_wf1.y(),v_wf1.z()); 
        float last_distance1 = gsdf_temp_acc.getValue(voxel1);
        if(last_distance1 != -100){
          voxelsCloseSurface.push_back(outPoints);
          voxelsCloseSurfaceSig.push_back(-1);
        }
        if(freespace_trunc_out_ > 0 && Id <= freespace_trunc_out_){ // add query points behind the surface
          Eigen::Vector3d inPoints(localGPsPoints[Idx].x()-normalP.x()*voxel_size_gl_temp1*Id,
            localGPsPoints[Idx].y()-normalP.y()*voxel_size_gl_temp1*Id,
            localGPsPoints[Idx].z()-normalP.z()*voxel_size_gl_temp1*Id);
          openvdb::math::Vec3d voxeltemp2(inPoints.x(),inPoints.y(),inPoints.z()); 
          openvdb::math::Vec3d v_wf2 = xform_temp.worldToIndex(voxeltemp2);
          openvdb::math::Coord voxel2(v_wf2.x(),v_wf2.y(),v_wf2.z()); 
          float last_distance2 = gsdf_temp_acc.getValue(voxel2);
          if(last_distance2 != -100){
            voxelsCloseSurface.push_back(inPoints);
            voxelsCloseSurfaceSig.push_back(1);
          }
        }
      }
    }
  }

  if(query_downsample_ > 0){
    openvdb::FloatGrid::Ptr down_pointsVDB_IO;
    down_pointsVDB_IO = openvdb::FloatGrid::create(sdf_trunc_);
    down_pointsVDB_IO->setName("a grid for downsampling");
    down_pointsVDB_IO->setTransform(
        openvdb::math::Transform::createLinearTransform(query_downsample_));
    down_pointsVDB_IO->setGridClass(openvdb::GRID_LEVEL_SET);
    auto down_points_io_acc = down_pointsVDB_IO->getUnsafeAccessor();
    const openvdb::math::Transform &xform_down_points_io = down_pointsVDB_IO->transform();
    
    std::vector<Eigen::Vector3d> voxelsCloseSurfDown;
    std::vector<float> voxelsCloseSurfDownSig;
    
    size_t voxelsCloseSurface_size = voxelsCloseSurface.size();
    for (size_t i = 0; i < voxelsCloseSurface_size; ++i) {
      const auto point = voxelsCloseSurface[i];
      openvdb::Vec3d voxeltemp(point.x(),point.y(),point.z()); 
      voxeltemp = xform_down_points_io.worldToIndex(voxeltemp);
      openvdb::math::Coord localPoint(voxeltemp.x(),voxeltemp.y(),voxeltemp.z()); 
      down_points_io_acc.setValue(localPoint, voxelsCloseSurfaceSig[i]); 
    }
    
    voxelsCloseSurface.clear();
    voxelsCloseSurfaceSig.clear();
    for (auto iter = down_pointsVDB_IO->beginValueOn(); iter; ++iter) {
      auto& value = *iter;
      auto iterValueOnWorld = xform_down_points_io.indexToWorld(iter.getCoord());
      Eigen::Vector3d tmp123(iterValueOnWorld.x(),iterValueOnWorld.y(),iterValueOnWorld.z());
      voxelsCloseSurface.push_back(tmp123);
      voxelsCloseSurfaceSig.push_back(value);
    }
  LOG_IF(INFO, debug_print_) << "close surface points after downsampling: " << voxelsCloseSurface.size() << " ;" << std::endl;
  }
  voxelsToUpdate = voxelsCloseSurface;
  voxelsToUpdateSig = voxelsCloseSurfaceSig;
}

  std::tuple<std::vector<double>, //dis
             std::vector<double>, //var
             std::vector<double>, //red
             std::vector<double>, //green
             std::vector<double>> //blue
  VDBVolume::ComputeVoxelsDistances(const std::vector<std::shared_ptr<OnGPDF>> &localGPs, 
                   const std::vector<Eigen::Vector3d> &localGPsCenters, 
                   const std::vector<Eigen::Vector3d> &voxelsToUpdate,
                   const std::vector<int> &voxelsToUpdateSig){
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudS(new pcl::PointCloud<pcl::PointXYZRGBA>);
  size_t voxelsToUpdate_size = voxelsToUpdate.size();
  for (size_t testIdx = 0; testIdx < voxelsToUpdate_size; testIdx = testIdx+1) {
        pcl::PointXYZRGBA pt;
        pt.x = static_cast<float>(voxelsToUpdate[testIdx].x());
        pt.y = static_cast<float>(voxelsToUpdate[testIdx].y());
        pt.z = static_cast<float>(voxelsToUpdate[testIdx].z());
        // pt.a = static_cast<float>(voxelsToUpdateSig[testIdx]);
        cloudS->push_back(pt);
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZRGBA>);
  size_t localGPsCenters_size = localGPsCenters.size();
  for (size_t targetIdx = 0; targetIdx < localGPsCenters_size; targetIdx++) {
    pcl::PointXYZRGBA pt;
    pt.x = static_cast<float>(localGPsCenters[targetIdx].x());
    pt.y = static_cast<float>(localGPsCenters[targetIdx].y());
    pt.z = static_cast<float>(localGPsCenters[targetIdx].z());
    cloudT->push_back(pt);
  }

  std::vector<int> indicesKNN;
  std::vector<double> distancesKNN;
  int knne = 1; // we did not use smooth minimum in the local gpdf
  std::vector<double> disAllTesting, varAllTesting, rrAllTesting, ggAllTesting, bbAllTesting;

  for (size_t testIdx = 0; testIdx < voxelsToUpdate_size; testIdx = testIdx+1) {
    disAllTesting.push_back(10);
    rrAllTesting.push_back(10);
    ggAllTesting.push_back(10);
    bbAllTesting.push_back(10);
    if(variance_method_ > 0) varAllTesting.push_back(10);
    for (int temp = 0; temp < knne; temp++){
      indicesKNN.push_back(testIdx);
      distancesKNN.push_back(testIdx);
    }
  }
  
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	kdtree->setInputCloud(cloudT);
	
	for (size_t idxSource = 0; idxSource < voxelsToUpdate_size; idxSource++)
	{
    std::vector<int> pointIdxNKNSearch;
	  std::vector<float> pointNKNSquaredDistance;
		if (kdtree->nearestKSearch(cloudS->points[idxSource], knne, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
      for (size_t idxCloseSearch = 0; idxCloseSearch < pointIdxNKNSearch.size(); idxCloseSearch++){
        indicesKNN[idxSource*knne+idxCloseSearch] = pointIdxNKNSearch[idxCloseSearch];
        distancesKNN[idxSource*knne+idxCloseSearch] = pointNKNSquaredDistance[idxCloseSearch];
      }
		}
	}

  size_t indicesKNN_size = indicesKNN.size();
  omp_set_num_threads(12);
  #pragma omp parallel for
  for (size_t testIdx = 0; testIdx < indicesKNN_size; testIdx = testIdx+knne) {
    EVectorX xt(3);
    xt << voxelsToUpdate[testIdx/knne].x(),voxelsToUpdate[testIdx/knne].y(),voxelsToUpdate[testIdx/knne].z();
    double totalVal = 0;
    double totalVar = 0;
    // double totalExp = 0;
    // double totalExpd = 0;
    
    double rr = 0;
    double gg = 0;
    double bb = 0;
    #pragma omp parallel for
    for (auto idxCloseSearch = 0; idxCloseSearch < knne; idxCloseSearch++) {
      auto closeGP = localGPs[indicesKNN[testIdx+idxCloseSearch]];
      double val = 0;
      double var = 0;
      double value = 0;
      double variance = 0;
      closeGP->testSingleDistanceColorVariance(xt, val, rr, gg, bb, var, use_color_, variance_method_);
      
      if(distance_method_ == 0){
        double length_scale = 1/(sqrt(map_lambda_scale_));
        value = sqrt(abs(-2*length_scale*length_scale*log(abs(val))));
        
        if(variance_method_ > 0){
          double lamDa = -length_scale/(val*sqrt(-2*log(abs(val))));
          variance = lamDa*var*lamDa;

          // cap crazy variance in LogGPDF and RevertingGPDF
          if(variance >= variance_cap_) variance = variance_cap_;
          // nan means we have log(1) which is the exact surface, so the variance should be very low
          if(isnan(variance)) variance = variance_on_surface_;
        }
      }else if (distance_method_ == 1)
      {
        value = -(1/map_lambda_scale_)*log(abs(val));
        
        if(variance_method_ > 0){
          double lamDa = 1/(map_lambda_scale_*val);
          variance = lamDa*var*lamDa;

          // cap crazy variance in LogGPDF and RevertingGPDF
          if(variance >= variance_cap_) variance = variance_cap_;
          // nan means we have log(1) which is the exact surface, so the variance should be very low
          if(isnan(variance)) variance = variance_on_surface_;
        }
      }

      totalVal = value;
      if((variance_method_ == 1)||(variance_method_ == 3)){
        totalVar = variance; // distance variance
      }else if (variance_method_ == 2)
      {
        totalVar = var; // occupancy variance
      }
      
      //totalExpd = totalExpd + value*exp(-smooth_param_*value);
      //totalExp = totalExp + exp(-smooth_param_*value);
      if(isinf(totalVal)) totalVal = distancesKNN[testIdx+idxCloseSearch];
    }
    #pragma omp critical
    {
      //careful, use smooth minimum here can cause "nan" in the final value
      //disAllTesting[testIdx/knne]=(totalExpd/totalExp)*voxelsToUpdateSig[testIdx/knne];
      disAllTesting[testIdx/knne]=totalVal*voxelsToUpdateSig[testIdx/knne];
      if(variance_method_ > 0) varAllTesting[testIdx/knne]=totalVar;
      rrAllTesting[testIdx/knne]=rr;
      ggAllTesting[testIdx/knne]=gg;
      bbAllTesting[testIdx/knne]=bb;
    }
  }

  // if(use_variance_){
  //   double maxValue = *max_element(varAllTesting.begin(),varAllTesting.end()); 
  //   double minValue = *min_element(varAllTesting.begin(),varAllTesting.end());
  //   std::cout << "variance print: " << maxValue << " ;" << minValue << std::endl;
  // }

  return std::make_tuple(disAllTesting, varAllTesting, rrAllTesting, ggAllTesting, bbAllTesting);
}

void VDBVolume::Fusion(const std::vector<Eigen::Vector3d> &voxelsToUpdate, //voxels
                  const std::vector<double> &disAllTesting, //dis
                  const std::vector<double> &varAllTesting, //var
                  const std::vector<double> &rrAllTesting, //red
                  const std::vector<double> &ggAllTesting, //green
                  const std::vector<double> &bbAllTesting, //blue
                  const std::function<float(float)> &weighting_function){ 
  // Get the "unsafe" version of the grid accessors
  auto gsdf_acc = gsdf_->getUnsafeAccessor();
  auto weights_acc = weights_->getUnsafeAccessor();
  auto colors_acc = colors_->getUnsafeAccessor();
  const openvdb::math::Transform &xform = gsdf_->transform();
  
  size_t voxelsToUpdate_size = voxelsToUpdate.size();
  for (size_t i = 0; i < voxelsToUpdate_size; ++i) {
      if(isinf(disAllTesting[i])) continue;
  
      openvdb::math::Vec3d voxeltemp(voxelsToUpdate[i].x(),voxelsToUpdate[i].y(),voxelsToUpdate[i].z()); 
      openvdb::math::Vec3d v_wf = xform.worldToIndex(voxeltemp);
      openvdb::math::Coord voxel(v_wf.x(),v_wf.y(),v_wf.z()); 
      
      float last_weight = 0;
      float current_weight = 0;
      float new_weight = 0;
      
      float last_distance = 0;
      float current_distance = disAllTesting[i];
      float new_distance = 0;

      float last_variance = 0;
      float current_variance = 0;
      float new_variance = 0;

      if(variance_method_ == 0){ // tsdf fusion with constant weight
        current_weight = weighting_function(current_distance);
        last_weight = weights_acc.getValue(voxel);
        last_distance = gsdf_acc.getValue(voxel);

        new_weight = current_weight + last_weight;
        new_distance = (last_distance * last_weight + current_distance * current_weight) / (new_weight);

        gsdf_acc.setValueOn(voxel, new_distance);
        weights_acc.setValueOn(voxel, new_weight);
      }else if(variance_method_ == 1){ // tsdf fustion with 1-variance as weight
        current_weight = abs(1-varAllTesting[i]);
        last_weight = weights_acc.getValue(voxel);
        last_distance = gsdf_acc.getValue(voxel);

        new_weight = current_weight + last_weight;
        new_distance = (last_distance * last_weight + current_distance * current_weight) / (new_weight);

        gsdf_acc.setValueOn(voxel, new_distance);
        weights_acc.setValueOn(voxel, new_weight);
      }else if((variance_method_ == 2)||(variance_method_ == 3)){ // probabilistic fusion with variance, 2 is to use occupancy variance, 3 is to use distance variance
        current_variance = abs(varAllTesting[i]);
        current_weight = 1/current_variance;

        last_weight = weights_acc.getValue(voxel);
        last_variance = 1/last_weight;

        last_distance = gsdf_acc.getValue(voxel);
        if (last_distance == -100){
          gsdf_acc.setValueOn(voxel, current_distance);
          weights_acc.setValueOn(voxel, current_weight);
        }else{
          new_variance = (current_variance * last_variance)/(current_variance + last_variance);
          new_distance = (last_distance * current_variance + current_distance * last_variance)/(current_variance + last_variance);
          new_weight = 1/new_variance;

          gsdf_acc.setValueOn(voxel, new_distance);
          weights_acc.setValueOn(voxel, new_weight);
        }
      }
      
      //std::cout << "previous: " << last_variance << " ;" << last_weight << " ;" << last_distance << std::endl;
      //std::cout << "current: " << current_variance << " ;" << current_weight << " ;" << current_distance << std::endl;
      //std::cout << "new: " << new_variance << " ;" << new_weight << " ;" << new_distance << std::endl;

      if(use_color_){
        const auto pre_color = colors_acc.getValue(voxel);
        openvdb::Vec3i new_color(static_cast<int>(round(rrAllTesting[i])),
          static_cast<int>(round(ggAllTesting[i])),
          static_cast<int>(round(bbAllTesting[i])));
        openvdb::Vec3i blend_color =
            BlendColors(pre_color, last_weight, new_color, current_weight);
        colors_acc.setValue(voxel, blend_color);
      }
  }
}

void VDBVolume::PrepareGlobalDistanceField(std::vector<Eigen::Vector3d> &globalGPsPoints, std::vector<openvdb::Vec3i> &globalGPsPointsColor){
  auto gsdf_acc = gsdf_->getUnsafeAccessor();
  auto weights_acc = weights_->getUnsafeAccessor();
  auto colors_acc = colors_->getUnsafeAccessor();
  const openvdb::math::Transform &xform = gsdf_->transform();

  int active_leaf = 0;
  int active_voxel = 0;
  for (openvdb::FloatGrid::TreeType::LeafIter iterL = gsdf_->tree().beginLeaf(); iterL; ++iterL) {
      auto leaf = iterL.getLeaf();
      
      if (leaf->onVoxelCount()>=10){
        active_leaf = active_leaf + 1;
        const openvdb::CoordBBox bboxTest = iterL->getNodeBoundingBox();
        const auto coordBBoxIntO = xform.indexToWorld(bboxTest);
        std::vector<Eigen::Vector3d> leafValidVoxels;
        std::vector<openvdb::Vec3i> leafVoxelsColors;

        // implementation of marching cubes, based on Open3D
        std::vector<Eigen::Vector3d> verticesM;
        std::vector<Eigen::Vector3i> trianglesM;
        std::vector<openvdb::Vec3i> colorsM;
        double half_voxel_lengthM = voxel_size_gl_ * 0.5;
        std::unordered_map<
            Eigen::Vector4i, int, hash_eigen<Eigen::Vector4i>, std::equal_to<>,
            Eigen::aligned_allocator<std::pair<const Eigen::Vector4i, int>>>
            edgeindex_to_vertexindexM;
        //int edge_to_indexM[12];
        
        for (auto iterLV = leaf->beginValueAll(); iterLV; ++iterLV){
          auto getValue = iterLV.getValue();
          if (getValue == -100) continue;
          active_voxel = active_voxel + 1;
          float vertex_tsdfM[8];
          openvdb::math::Vec3<int> colors_fieldM[8];
          const openvdb::Coord &voxelM = iterLV.getCoord();
          const int32_t xM = voxelM.x();
          const int32_t yM = voxelM.y();
          const int32_t zM = voxelM.z();

          int cube_indexM = 0;
          // Iterate through all the 8 neighbour vertices...
          for (int vertexM = 0; vertexM < 8; vertexM++) {
            openvdb::Coord idxM = voxelM + openvdb::shift[vertexM];
            if (!true) {
              if (weights_acc.getValue(idxM) == 0.0f) {
                cube_indexM = 0;
                break;
              }
            }
            if (weights_acc.getValue(idxM) < 0.1) {
              cube_indexM = 0;
              break;
            }
            vertex_tsdfM[vertexM] = gsdf_acc.getValue(idxM);
            colors_fieldM[vertexM] = colors_acc.getValue(idxM);
            colors_fieldM[vertexM][0] = (int)colors_fieldM[vertexM][0];
            colors_fieldM[vertexM][1] = (int)colors_fieldM[vertexM][1];
            colors_fieldM[vertexM][2] = (int)colors_fieldM[vertexM][2];
            if (vertex_tsdfM[vertexM] < 0.0f) {
              cube_indexM |= (1 << vertexM);
            }
          }
          
          if (cube_indexM == 0 || cube_indexM == 255) {
            continue;
          }
          // Iterate trough all the edges..
          for (int edgeM = 0; edgeM < 12; edgeM++) {
            if ((edge_table[cube_indexM] & (1 << edgeM)) != 0) {
              Eigen::Vector4i edge_indexM =
                  Eigen::Vector4i(xM, yM, zM, 0) + edge_shift[edgeM];
              if (edgeindex_to_vertexindexM.find(edge_indexM) ==
                  edgeindex_to_vertexindexM.end()) {
                //edge_to_indexM[edgeM] = (int)verticesM.size();
                edgeindex_to_vertexindexM[edge_indexM] = (int)verticesM.size();
                // set point to source vertex (x, y, z) coordinates
                Eigen::Vector3d pointM(
                    half_voxel_lengthM + voxel_size_gl_ * edge_indexM(0),
                    half_voxel_lengthM + voxel_size_gl_ * edge_indexM(1),
                    half_voxel_lengthM + voxel_size_gl_ * edge_indexM(2));
                // source vertex TSDF
                double source_tsdfM =
                    std::abs((double)vertex_tsdfM[edge_to_vert[edgeM][0]]);
                // destination vertex TSDF
                double destination_tsdfM =
                    std::abs((double)vertex_tsdfM[edge_to_vert[edgeM][1]]);
                // adding delta to reach destination vertex
                pointM(edge_indexM(3)) +=
                    source_tsdfM * voxel_size_gl_ / (source_tsdfM + destination_tsdfM);
                verticesM.push_back(pointM /* + origin_*/);
                leafValidVoxels.push_back(pointM /* + origin_*/);
                globalGPsPoints.push_back(pointM /* + origin_*/);

                const auto &source_colorM = colors_fieldM[edge_to_vert[edgeM][0]];
                const auto &destination_colorM =
                    colors_fieldM[edge_to_vert[edgeM][1]];
                openvdb::Vec3i colorM;
                openvdb::math::Vec3<int> current_colorM =
                    (destination_tsdfM * source_colorM +
                    source_tsdfM * destination_colorM) /
                    (source_tsdfM + destination_tsdfM);
                colorM[0] = static_cast<int>(current_colorM[0]);
                colorM[1] = static_cast<int>(current_colorM[1]);
                colorM[2] = static_cast<int>(current_colorM[2]);
                leafVoxelsColors.push_back(colorM);
                globalGPsPointsColor.push_back(colorM);
              } else {
                //we don't care about the edge in here
                //edge_to_indexM[edgeM] =
                //edgeindex_to_vertexindexM.find(edge_indexM)->second;
              }
            }
          }
        }
        if(leafValidVoxels.size()>=1){
          std::shared_ptr<OnGPDF> gp1(new OnGPDF(leafValidVoxels,leafVoxelsColors,distance_method_,map_lambda_scale_,map_noise_,color_scale_));
          openvdb::Vec3R leafCenter = coordBBoxIntO.getCenter();
          Eigen::Vector3d gpCenter(leafCenter.x(),leafCenter.y(),leafCenter.z());
          
          std::vector<Eigen::Vector3d>::iterator iter=std::find(cubeCenters_.begin(),cubeCenters_.end(),gpCenter);
          if(iter == cubeCenters_.end())
          {
            cubeCenters_.push_back(gpCenter);
            allGPs_.push_back(gp1);
          } else {
            auto idxToUpdate = std::distance(cubeCenters_.begin(),iter);
            allGPs_[idxToUpdate] = gp1;
          }
        }else if(leafValidVoxels.size()==0){ // this part is important for a clean dynamic distance field
          openvdb::Vec3R leafCenter = coordBBoxIntO.getCenter();
          Eigen::Vector3d gpCenter(leafCenter.x(),leafCenter.y(),leafCenter.z());
          std::vector<Eigen::Vector3d>::iterator iter=std::find(cubeCenters_.begin(),cubeCenters_.end(),gpCenter);
          if(iter != cubeCenters_.end()){
            int idxToUpdate = std::distance(cubeCenters_.begin(),iter);
            cubeCenters_.erase( iter ); 
            allGPs_.erase(allGPs_.begin()+idxToUpdate);
          }
        }
      }
      leaf->setValuesOff();
  }
}

bool VDBVolume::QueryMap(float *points,
                int leng,
                double *res, int *if_observed) {
  
  std::vector<int> indicesKNNGlobal;
  std::vector<double> distancesKNNGlobal;
  int knne1 = 3;
  std::vector<Eigen::Vector3d> allTestingPointsGlobal;

  const openvdb::math::Transform &xform_query = gsdf_->transform();
  auto gsdf_query_acc = gsdf_->getUnsafeAccessor();

  // create a horizontal slice of points as testing points (world coordinates)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSouceGlobal(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < leng; ++i) {
    int k3 = 3 * i;

    Eigen::Vector3d tmp12(points[k3],points[k3 + 1],points[k3 + 2]);
    pcl::PointXYZ ptt;
    ptt.x = static_cast<float>(tmp12.x());
    ptt.y = static_cast<float>(tmp12.y());
    ptt.z = static_cast<float>(tmp12.z());
    cloudSouceGlobal->push_back(ptt);
    allTestingPointsGlobal.push_back(tmp12);

    for (int temp = 0; temp < knne1; temp++){
      indicesKNNGlobal.push_back(10);
      distancesKNNGlobal.push_back(10);
    }

    openvdb::math::Vec3d voxeltemp(tmp12.x(),tmp12.y(),tmp12.z()); 
    openvdb::math::Vec3d v_wf = xform_query.worldToIndex(voxeltemp);
    openvdb::math::Coord voxel(v_wf.x(),v_wf.y(),v_wf.z());
    const float last_distance = gsdf_query_acc.getValue(voxel);
    if(last_distance != -100){
      if_observed[i] = 1; // 1 for observed
    }else{
      if_observed[i] = 0;
    }
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTargetGlobal(new pcl::PointCloud<pcl::PointXYZ>);
  size_t cubeCenters_size = cubeCenters_.size();
  for (size_t targetIdx = 0; targetIdx < cubeCenters_size; targetIdx++) {
    pcl::PointXYZ ptt;
    ptt.x = static_cast<float>(cubeCenters_[targetIdx].x());
    ptt.y = static_cast<float>(cubeCenters_[targetIdx].y());
    ptt.z = static_cast<float>(cubeCenters_[targetIdx].z());
    cloudTargetGlobal->push_back(ptt);
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeG(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtreeG->setInputCloud(cloudTargetGlobal);
	
  //Note: we can enable all omp in this functionn to speed up the query procedure. roughly 3 times faster for large point cloud input.
  //omp_set_num_threads(12);
  //#pragma omp parallel for
	for (size_t idxSource = 0; idxSource < cloudSouceGlobal->size(); idxSource++)
	{
    //printf("now in parallel thread %d\n",idxSource);
    std::vector<int> pointIdxNKNSearch;
	  std::vector<float> pointNKNSquaredDistance;
		if (kdtreeG->nearestKSearch(cloudSouceGlobal->points[idxSource], knne1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
      //omp_set_num_threads(12);
      //#pragma omp parallel for
      for (size_t idxCloseSearch = 0; idxCloseSearch < pointIdxNKNSearch.size(); idxCloseSearch++){
        //#pragma omp critical
        {
        indicesKNNGlobal[idxSource*knne1+idxCloseSearch] = pointIdxNKNSearch[idxCloseSearch];
        distancesKNNGlobal[idxSource*knne1+idxCloseSearch] = pointNKNSquaredDistance[idxCloseSearch];
        }
      }
		}
	}
 
  //omp_set_num_threads(12);
  //#pragma omp parallel for
  for (size_t testIdx = 0; testIdx < indicesKNNGlobal.size(); testIdx = testIdx+knne1) {
    EVectorX xt(3);
    xt << allTestingPointsGlobal[testIdx/knne1].x(),allTestingPointsGlobal[testIdx/knne1].y(),allTestingPointsGlobal[testIdx/knne1].z();
    double totalExp = 0;
    double totalExpd = 0;
    double totalGrad[3] = {0,0,0};
    
    //#pragma omp parallel for
    for (auto idxCloseSearch = 0; idxCloseSearch < knne1; idxCloseSearch++) {
      std::vector<Eigen::Vector3d> pointsCube;
      std::vector<openvdb::Vec3i> pointColor;
      auto closeGP = allGPs_[indicesKNNGlobal[testIdx+idxCloseSearch]];
      if(!closeGP->isTrained()){
        pointsCube = closeGP->getPoints();
        pointColor = closeGP->getColors();
        //#pragma omp critical
        {
        closeGP->train(pointsCube,pointColor,false);
        }
      }
      double val = 0;
      double grad[3] = {0,0,0};
      closeGP->testSingleDistanceGradient(xt, val, grad);
      
      if(distance_method_ == 0){
        double length_scale = 1/(sqrt(map_lambda_scale_));
        val = sqrt(abs(-2*length_scale*length_scale*log(abs(val))));
      }else if(distance_method_ == 1){
        val = -(1/map_lambda_scale_)*log(abs(val));
      }
      
      // if numerical issue happens, try to give the distance value from knn search, do not use it for distance evluation
      if(isinf(val)) val = sqrt(distancesKNNGlobal[testIdx+idxCloseSearch]);
      
      //#pragma omp critical
      {
        totalExpd = totalExpd + val*exp(-smooth_param_*val);
        totalExp = totalExp + exp(-smooth_param_*val);
        totalGrad[0] = totalGrad[0]+grad[0];
        totalGrad[1] = totalGrad[1]+grad[1];
        totalGrad[2] = totalGrad[2]+grad[2];
      }
    }
    //#pragma omp critical
    {
      res[testIdx/knne1*8] = totalExpd/totalExp;
      res[testIdx/knne1*8 + 1] = totalGrad[0]/knne1; //grad1
      res[testIdx/knne1*8 + 2] = totalGrad[1]/knne1; //grad2
      res[testIdx/knne1*8 + 3] = totalGrad[2]/knne1; //grad3
    }
  }
  return true;
}

openvdb::FloatGrid::Ptr VDBVolume::Prune(float min_weight) const {
  const auto weights = weights_->tree();
  const auto gsdf = gsdf_->tree();
  const auto background = sdf_trunc_;
  openvdb::FloatGrid::Ptr clean_gsdf = openvdb::FloatGrid::create(-100);
  clean_gsdf->setName("D(x): Pruned signed distance grid");
  clean_gsdf->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size_gl_));
  clean_gsdf->setGridClass(openvdb::GRID_LEVEL_SET);
  clean_gsdf->tree().combine2Extended(
      gsdf, weights, [=](openvdb::CombineArgs<float> &args) {
        if (args.aIsActive() && args.b() > min_weight) {
          args.setResult(args.a());
          args.setResultIsActive(true);
        } else {
          args.setResult(background);
          args.setResultIsActive(false);
        }
      });
  return clean_gsdf;
}

void OnGPDF::reset(){
    trained = false;
    return;
}

void OnGPDF::train(const std::vector<Eigen::Vector3d> leafVoxels, std::vector<openvdb::Vec3i> leafVoxelsColor, bool trainColor){
    reset();

    int N = leafVoxels.size();
    int dim = 3;

    if (N > 0){
        x = EMatrixX::Zero(dim,N);
        EVectorX f = EVectorX::Zero(N);
        EVectorX rr = EVectorX::Zero(N);
        EVectorX gg = EVectorX::Zero(N);
        EVectorX bb = EVectorX::Zero(N);
        double sigx = map_noise;

        int k=0;
        for (auto it = leafVoxels.begin(); it!=leafVoxels.end(); it++, k++){
            x(0,k) = (*it).x();
            x(1,k) = (*it).y();
            x(2,k) = (*it).z();
            f(k) = 1; // refer to our paper
            rr(k) = static_cast<double>(leafVoxelsColor[k][0]);
            gg(k) = static_cast<double>(leafVoxelsColor[k][1]);
            bb(k) = static_cast<double>(leafVoxelsColor[k][2]);
        }
        
        EVectorX y(N);
        y << f;
        EVectorX y0(N);
        y0 << rr;
        EVectorX y1(N);
        y1 << gg;
        EVectorX y2(N);
        y2 << bb;
        EMatrixX K = se_kernel_3D(x, map_lambda_scale, sigx);

        L = K.llt().matrixL();
        alpha = y;
        L.template triangularView<Eigen::Lower>().solveInPlace(alpha);
        L.transpose().template triangularView<Eigen::Upper>().solveInPlace(alpha);

        if(trainColor == true){
          EMatrixX K1 = se_kernel_3D(x, color_scale, sigx);
          L_rgb = K1.llt().matrixL();
          alphaR = y0;
          L_rgb.template triangularView<Eigen::Lower>().solveInPlace(alphaR);
          L_rgb.transpose().template triangularView<Eigen::Upper>().solveInPlace(alphaR);
          alphaG = y1;
          L_rgb.template triangularView<Eigen::Lower>().solveInPlace(alphaG);
          L_rgb.transpose().template triangularView<Eigen::Upper>().solveInPlace(alphaG);
          alphaB = y2;
          L_rgb.template triangularView<Eigen::Lower>().solveInPlace(alphaB);
          L_rgb.transpose().template triangularView<Eigen::Upper>().solveInPlace(alphaB);
        }

        trained = true;
    }
    return;
}

void OnGPDF::testSingleDistanceGradient(const EVectorX& xt, double& val, double grad[])
{
    if (!isTrained())
        return;

    if (x.rows() != xt.size())
        return;

    EMatrixX K = se_kernel_3D(x, xt, map_lambda_scale);
    EVectorX res = K.transpose()*alpha;
    val = res(0);

    int n = x.cols();
    int m = xt.cols();

    double a = map_lambda_scale;
    EMatrixX Grx = EMatrixX::Zero(n,m);
    EMatrixX Gry = EMatrixX::Zero(n,m);
    EMatrixX Grz = EMatrixX::Zero(n,m);

    for (int k=0;k<n;k++){
      for (int j=0;j<m;j++){
          double r = (x.col(k)-xt.col(j)).norm();
          if(distance_method == 0){
            Grx(k, j) = kf_se1(r,x(0, k) - xt(0, j),a);
            Gry(k, j) = kf_se1(r,x(1, k) - xt(1, j),a);
            Grz(k, j) = kf_se1(r,x(2, k) - xt(2, j),a);
          }else if(distance_method == 1){
            Grx(k, j) = kf_ma1(r,x(0, k) - xt(0, j),a);
            Gry(k, j) = kf_ma1(r,x(1, k) - xt(1, j),a);
            Grz(k, j) = kf_ma1(r,x(2, k) - xt(2, j),a);
          }
      }
    }

    EVectorX gradx = Grx.transpose()*alpha;
    EVectorX grady = Gry.transpose()*alpha;
    EVectorX gradz = Grz.transpose()*alpha;

    grad[0] = gradx(0);  
    grad[1] = grady(0);
    grad[2] = gradz(0);

    double gradLen = sqrt(pow(grad[0], 2) + pow(grad[1], 2) + pow(grad[2], 2)); 
    
    if(gradLen != 0){
        grad[0]/=gradLen;
        grad[1]/=gradLen;
        grad[2]/=gradLen;
    } else {
        grad[0]=0;
        grad[1]=0;
        grad[2]=0;
    }
    return;
}

void OnGPDF::testSingleDistanceColorVariance(const EVectorX& xt, double& val, double& rr, double& gg, double& bb, 
                                             double& var, bool colorInfer, int varInfer){
    if (!isTrained())
        return;

    if (x.rows() != xt.size())
        return;

    EMatrixX K = se_kernel_3D(x, xt, map_lambda_scale);
    EVectorX res = K.transpose()*alpha;
    val = res(0);
    
    if(colorInfer == true){
      EMatrixX K1 = se_kernel_3D(x, xt, color_scale);
      EVectorX res0 = K1.transpose()*alphaR;
      rr = res0(0);
      EVectorX res1 = K1.transpose()*alphaG;
      gg = res1(0);
      EVectorX res2 = K1.transpose()*alphaB;
      bb = res2(0);

      if (rr>=255) rr = 255;
      if (gg>=255) gg = 255;
      if (bb>=255) bb = 255;
      if (rr<=0) rr = 0;
      if (gg<=0) gg = 0;
      if (bb<=0) bb = 0;
    }

    if(varInfer > 0){
      L.template triangularView<Eigen::Lower>().solveInPlace(K);
      K = K.array().pow(2);
      EVectorX v = K.colwise().sum();
      var = 1.0-v(0);

      // TODO: add variance for color in here
    }
    return;
}

// 3D train
EMatrixX OnGPDF::se_kernel_3D(EMatrixX const& x1, double scale_param, double sigx)
{
    int n = x1.cols();
    double a = scale_param;
    EMatrixX K = EMatrixX::Zero(n,n);

    for (int k=0;k<n;k++){
        for (int j=k;j<n;j++){
            if (k==j){
                K(k,k) = 1.0+sigx;
            }
            else{
                double r = (x1.col(k)-x1.col(j)).norm();
                if(distance_method == 0){
                  K(k,j) = kf_se(r,a);
                }else if(distance_method == 1){
                  K(k,j) = kf_ma(r,a);
                }
                K(j,k) = K(k,j);
            }
        }
     }
    return K;
}

// 3D test
EMatrixX OnGPDF::se_kernel_3D(EMatrixX const& x1, EMatrixX const& x2, double scale_param)
{
    int n = x1.cols();
    int m = x2.cols();
    double a = scale_param;
    EMatrixX K = EMatrixX::Zero(n,m);

     for (int k=0;k<n;k++){
        for (int j=0;j<m;j++){
            double r = (x1.col(k)-x2.col(j)).norm();
            if(distance_method == 0){
              K(k,j) = kf_se(r,a);
            }else if(distance_method == 1){
              K(k,j) = kf_ma(r,a);
            }
        }
     }
    return K;
}
} // namespace vdb_gpdf