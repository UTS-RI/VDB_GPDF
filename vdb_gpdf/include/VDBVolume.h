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

#pragma once
#include <openvdb/openvdb.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Core>
#include <functional>
#include <tuple>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

#include "MarchingCubesConst.h"

typedef Eigen::MatrixXd EMatrixX;
typedef Eigen::VectorXd EVectorX;
typedef Eigen::RowVectorXd ERowVectorX;

namespace vdb_gpdf {
class PointList
{
public:
    typedef openvdb::Vec3R  PosType;

    PointList(const std::vector<PosType>& points)
        : mPoints(&points)
    {
    }

    size_t size() const {
        return mPoints->size();
    }

    void getPos(size_t n, PosType& xyz) const {
        xyz = (*mPoints)[n];
    }

protected:
    std::vector<PosType> const * const mPoints;
}; // PointList

class OnGPDF{

public:
    OnGPDF(std::vector<Eigen::Vector3d> trainingPoints,
           std::vector<openvdb::Vec3i> trainingColors,
           int distance_method_type,
           double para_map_lambda_scale,
           double para_map_noise,
           double para_color_scale):
           distance_method(distance_method_type),
           map_lambda_scale(para_map_lambda_scale),
           map_noise(para_map_noise),
           color_scale(para_color_scale),
           trained(false),
           points(trainingPoints),
           colors(trainingColors){}

    void reset();
    bool isTrained(){return trained;}
    std::vector<Eigen::Vector3d> getPoints(){return points;}
    std::vector<openvdb::Vec3i> getColors(){return colors;}

    void train(const std::vector<Eigen::Vector3d> leafVoxels, std::vector<openvdb::Vec3i> leafVoxelsColor, bool trainColor);
    void testSingleDistanceColorVariance(const EVectorX& xt, double& val, double& rr, double& gg, double& bb, double& var, bool colorInfer, int varInfer);
    void testSingleDistanceGradient(const EVectorX& xt, double& val, double grad[]);

    // covariances for x1 (input points) with different noise params for inputs
    EMatrixX se_kernel_3D(EMatrixX const& x1, double scale_param, double sigx);

    // covariances for x1 (input points) and x2 (test points)
    EMatrixX se_kernel_3D(EMatrixX const& x1, EMatrixX const& x2, double scale_param);

private:
    EMatrixX x;
    EMatrixX L;
    EMatrixX L_rgb;
    EVectorX alpha;
    EVectorX alphaR;
    EVectorX alphaG;
    EVectorX alphaB;
    
    int distance_method = 0;
    double map_lambda_scale = 0;
    double map_noise = 0;
    double color_scale = 0;
    bool trained = false;
    std::vector<Eigen::Vector3d> points;
    std::vector<openvdb::Vec3i> colors;
}; // OnGPDF

class VDBVolume {

public:
    VDBVolume(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~VDBVolume() = default;

    /// @brief Integrates a new (globally aligned) PointCloud into the current
    /// gsdf_ volume.
    void Integrate(const std::vector<Eigen::Vector3d> &points,
                    const std::vector<openvdb::Vec3i> &colors,
                    const Eigen::Vector3d &origin,
                    const std::function<float(float)> &weighting_function, 
                    std::vector<Eigen::Vector3d> &globalGPsPoints,
                    std::vector<openvdb::Vec3i> &globalGPsPointsColor,
                    std::vector<Eigen::Vector3d> &localGPsPoints,
                    std::vector<Eigen::Vector3d> &localQueryPoints,
                    std::vector<double> &localQueryDis);

    /// @brief Integrates a new (globally aligned) PointCloud into the current
    /// gsdf_ volume.
    void inline Integrate(const std::vector<Eigen::Vector3d> &points,
                            const std::vector<openvdb::Vec3i> &colors,
                            const Eigen::Matrix4d &extrinsics,
                            const std::function<float(float)> &weighting_function, 
                            std::vector<Eigen::Vector3d> &globalGPsPoints, //all GP points for all current active cubes
                            std::vector<openvdb::Vec3i> &globalGPsPointsColor, // color of all GP points
                            std::vector<Eigen::Vector3d> &localGPsPoints, // current points for current local GPDF
                            std::vector<Eigen::Vector3d> &localQueryPoints, // local testing points in frustum
                            std::vector<double> &localQueryDis // distances of local testing points in frustum
                            ){
        const Eigen::Vector3d &origin = extrinsics.block<3, 1>(0, 3);
        Integrate(points, colors, origin, weighting_function, globalGPsPoints, globalGPsPointsColor,
                    localGPsPoints, localQueryPoints, localQueryDis);
    }

    /// @brief Integrates a new (globally aligned) PointCloud into the current
    /// gsdf_ volume.
    void Integrate(const std::vector<Eigen::Vector3d> &points,
                    const std::vector<openvdb::Vec3i> &colors,
                    const Eigen::Vector3d &origin,
                    const std::function<float(float)> &weighting_function);

    /// @brief Integrates a new (globally aligned) PointCloud into the current
    /// gsdf_ volume.
    void inline Integrate(const std::vector<Eigen::Vector3d> &points,
                            const std::vector<openvdb::Vec3i> &colors,
                            const Eigen::Matrix4d &extrinsics,
                            const std::function<float(float)> &weighting_function) {
        const Eigen::Vector3d &origin = extrinsics.block<3, 1>(0, 3);
        Integrate(points, colors, origin, weighting_function);
    }
    
    /// @brief query the global distance field for points
    bool QueryMap(float *points,
                    int leng,
                    double *res,
                    int *if_observed);

    /// @brief Prune TSDF grids, ideal utility to cleanup a D(x) volume before
    /// exporting it
    openvdb::FloatGrid::Ptr Prune(float min_weight) const;

    /// @brief Extracts a TriangleMesh as the iso-surface in the actual volume
    [[nodiscard]] std::tuple<std::vector<Eigen::Vector3d>,
                            std::vector<Eigen::Vector3i>,
                            std::vector<openvdb::Vec3i>>
    ExtractTriangleMesh(bool fill_holes = true, float min_weight = 0.5) const;

    /// OpenVDB Grids modeling the signed distance, weight and color
    openvdb::FloatGrid::Ptr gsdf_;
    openvdb::FloatGrid::Ptr weights_;
    openvdb::Vec3IGrid::Ptr colors_;

private:
    /// @brief Create the local distance field using raw input points and colors, based on a local VDB
    void CreateLocalDisantceField(const std::vector<Eigen::Vector3d> &points,
                                    const std::vector<openvdb::Vec3i> &colors, 
                                    float localGridSize, 
                                    std::vector<std::shared_ptr<OnGPDF>> &localGPs, 
                                    std::vector<Eigen::Vector3d> &localGPsCenters,
                                    std::vector<Eigen::Vector3d> &localGPsPoints);
    
    /// @brief Generate voxels around the surface and in free space to be updated
    void GenerateVoxelsToUpdate(const std::vector<Eigen::Vector3d> &localGPsPoints,
                                const Eigen::Vector3d &origin,
                                std::vector<Eigen::Vector3d> &voxelsToUpdate,
                                std::vector<int> &voxelsToUpdateSig);
    
    /// @brief Compute distance and more for voxels around the surface and in free space
    [[nodiscard]] std::tuple<std::vector<double>, //dis
                std::vector<double>, //var
                std::vector<double>, //red
                std::vector<double>, //green
                std::vector<double>> //blue
    ComputeVoxelsDistances(const std::vector<std::shared_ptr<OnGPDF>> &localGPs, 
                    const std::vector<Eigen::Vector3d> &localGPsCenters, 
                    const std::vector<Eigen::Vector3d> &voxelsToUpdate,
                    const std::vector<int> &voxelsToUpdateSig);

    /// @brief fuse all voxels information to the global VDB
    void Fusion(const std::vector<Eigen::Vector3d> &voxelsToUpdate, //voxels
                const std::vector<double> &disAllTesting, //dis
                const std::vector<double> &varAllTesting, //var
                const std::vector<double> &rrAllTesting, //red
                const std::vector<double> &ggAllTesting, //green
                const std::vector<double> &bbAllTesting, //blue
                const std::function<float(float)> &weighting_function);

    /// @brief Prepare the global distance field based on global VDB
    void PrepareGlobalDistanceField(std::vector<Eigen::Vector3d> &globalGPsPoints, std::vector<openvdb::Vec3i> &globalGPsPointsColor);

    /// VDBVolume private properties
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    bool debug_print_ = false;
    bool use_color_ = false;

    float sdf_trunc_ = 0.0;
    bool space_carving_ = false;
    
    int distance_method_ = 0;

    int sensor_type_ = 0;
    float voxel_size_lo_ = 0.0;
    int voxel_overlapping_ = 0;
    int voxel_downsample_ = 0;
    float voxel_size_gl_ = 0.0;

    int variance_method_ = 0;
    float variance_cap_ = 0;
    float variance_on_surface_ = 0;

    int surface_normal_method_ = 0;
    int surface_normal_num_ = 0;
    float surface_value_ = 0;
    float query_iterval_ = 0.0;
    int query_trunc_in_ = 0;
    int query_trunc_out_ = 0;
    float freespace_iterval_ = 0.0;
    int freespace_trunc_out_ = 0;
    float query_downsample_ = 0.0;

    double map_lambda_scale_ = 0.0;
    double map_noise_ = 0.0;
    double color_scale_ = 0.0;
    double smooth_param_ = 0.0;

    std::vector<std::shared_ptr<OnGPDF>> allGPs_;
    std::vector<Eigen::Vector3d> cubeCenters_;
}; // VDBVolume

}  // namespace vdb_gpdf