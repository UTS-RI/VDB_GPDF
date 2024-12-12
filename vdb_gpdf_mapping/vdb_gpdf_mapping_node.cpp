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

#include <glog/logging.h>
#include <ros/ros.h>
#include <thread>
//#include "omp.h"

#include "vdb_gpdf_mapper.h"
//#include "igl/write_triangle_mesh.h"

int main(int argc, char **argv) {
  
  // Start Ros.
  ros::init(argc, argv, "vdb_gpdf_mapping",
            ros::init_options::NoSigintHandler);

  // Setup logging.
  // google::InitGoogleLogging(argv[0]);
  // google::InstallFailureSignalHandler();
  // google::SetStderrLogging(google::INFO);
  // FLAGS_colorlogtostderr = true;

  // Setup node.
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  vdb_gpdf_mapping::VDBGPDFMapper mapper(nh, nh_private);
  std::thread integrate_thread{
      &vdb_gpdf_mapping::VDBGPDFMapper::mapIntegrateProcess, &mapper};

  // Setup spinning.
  ros::AsyncSpinner spinner(mapper.getConfig().ros_spinner_threads);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}