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
#include <glog/logging.h>
#include <thread>
#include "vdb_gpdf_mapper.h"

int main(int argc, char **argv) {
  
  // Init ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vdb_gpdf_mapping_node");
  RCLCPP_INFO(node->get_logger(), "Starting vdb_gpdf_mapping_node...");

  // Mapper
  auto mapper = std::make_shared<vdb_gpdf_mapping::VDBGPDFMapper>(node);
  
  std::thread integrate_thread{
      [&mapper, &node]() {
        RCLCPP_INFO(node->get_logger(), "Starting map integration thread...");
        mapper->mapIntegrateProcess();
        RCLCPP_INFO(node->get_logger(), "Map integration thread finished.");
      }
  };

  // ROS2 spinning
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Executor spinning...");
  executor.spin();

  integrate_thread.join();
  RCLCPP_INFO(node->get_logger(), "Integration thread joined. Exiting.");

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Shutting down ROS2...");

  return 0;
}