/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//-------------------- 一条轨迹的基础参数配置-------------------
#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer_ros {

struct TrajectoryOptions {
  //来自于 src/cartographer/configuration_files/trajectory_builder.lua
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
      
  std::string tracking_frame;//由SLAM算法追踪的ROS坐标系ID，如果使用IMU，应该使用其坐标系
  std::string published_frame;//用于发布位姿子坐标系的ROS坐标系ID
                              //例如“odom”坐标系，如果一个“odom”坐标系由系统的不同部分提供，在这种情况下，map_frame中的“odom”姿势将被发布。
                              //否则，将其设置为“base_link”可能是合适的。
  std::string odom_frame;//在provide_odom_frame为真才启用，坐标系在published_frame和map_frame之间用于发布局部SLAM结果，通常是“odom”
  bool provide_odom_frame;//如果启用，局部，非闭环，持续位姿会作为odom_frame发布在map_frame中发布。
  bool use_odometry;
  bool use_nav_sat;
  bool use_landmarks;
  bool publish_frame_projected_to_2d;
  bool ignore_out_of_order_messages;
  int num_laser_scans; //订阅的激光扫描话题数量。在一个激光扫描仪的“scan”话题上订sensor_msgs/LaserScan或在多个激光扫描仪上订阅话题“scan_1”，“scan_2”等。
  int num_multi_echo_laser_scans;//订阅的多回波激光扫描主题的数量。
  int num_subdivisions_per_laser_scan;//将每个接收到的（多回波）激光扫描分成的点云数。分扫描可以在扫描仪移动时取消扫描获取的扫描
                                      //有一个相应的轨迹构建器选项可将细分扫描累积到将用于扫描匹配的点云中。
  int num_point_clouds; //要订阅的点云话题的数量。
  
  //传感器数据的采样频率
  double rangefinder_sampling_ratio;
  double odometry_sampling_ratio;
  double fixed_frame_pose_sampling_ratio;
  double imu_sampling_ratio;
  double landmarks_sampling_ratio;
};

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
