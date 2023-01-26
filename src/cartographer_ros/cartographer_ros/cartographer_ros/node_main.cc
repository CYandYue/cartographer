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

/********************* cartographer_node节点的源文件 ******************/

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h" //参数命令行解析工具，例如下面的 DEFINE_bool宏就是来自此头文件
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",                                  //gflags语法的三个参数：命令行要传入的参数，默认值，备注
              "First directory in which configuration files are searched, " //如左configration_directory在调用cartographer_node要传入
              "second is always the Cartographer installation to allow "    //如果想引用这些参数，在参数的前面加入FLAGS_即可
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)}; //创建一个缓存 tf 的类对象，其缓存 kTfBufferCacheTimeInSeconds(10) 秒内的 tf
  tf2_ros::TransformListener tf(tf_buffer); //tf2_ros::TransformListener 开启监听 tf 的独立线程

  //创建这两个类对象 来加载.lua文件中的参数
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;


  // 根据Lua配置文件中的内容, 为node_options, trajectory_options 赋值
  std::tie(node_options, trajectory_options) =                       // c++11: std::tie()函数可以将变量连接到一个给定的tuple上,生成一个元素类型全是引用的tuple
      //LoadOptions 源文件在 node_options.cc ,gflags语法传入文件夹与文件名，读取参数，返回的是tuple类型
      //再使用c++11提供的tie函数给多个变量赋值，赋值给node_options 与 trajectory_options
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);


  // MapBuilder类是完整的SLAM算法类
  // 包含前端(TrajectoryBuilders,scan to submap) 与 后端(用于查找回环的PoseGraph) 
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);

  // Node类的初始化, 将ROS的topic传入SLAM, 也就是MapBuilder
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);

  // 如果加载了pbstream文件, 就执行这个函数
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }


  // 使用默认topic 开始轨迹
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::spin();//单线程,回调函数执行有一定次序，不能并发。

  // 结束所有处于活动状态的轨迹
  node.FinishAllTrajectories();

  // 当所有的轨迹结束时, 再执行一次全局优化
  node.RunFinalOptimization();

  // 如果save_state_filename非空, 就保存pbstream文件
  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  //初始化glog库
  google::InitGoogleLogging(argv[0]); 

  // 使用gflags进行参数的初始化. 其中第三个参数为remove_flag
  // 如果为true, gflags会移除parse过的参数, 否则gflags就会保留这些参数, 但可能会对参数顺序进行调整.
  google::ParseCommandLineFlags(&argc, &argv, true);

  //来自glog库定义的一些宏，用于日志输出
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  //节点初始化
  ::ros::init(argc, argv, "cartographer_node");
  // 一般不需要在自己的代码中显式调用，但是若想在创建任何NodeHandle实例之前启动ROS相关的线程, 网络等, 可以显式调用该函数.
  ::ros::start(); 

  //ScopedRosLogSink 类对象 用于将glog信息转化为ros消息输出，ros_log_sink.h与ros_log_sink.cc
  //使用 ROS_INFO 进行glog消息的输出。
  cartographer_ros::ScopedRosLogSink ros_log_sink;


  // 开始运行cartographer_ros
  cartographer_ros::Run();
  // 结束ROS相关的线程, 网络等
  ::ros::shutdown();
}
