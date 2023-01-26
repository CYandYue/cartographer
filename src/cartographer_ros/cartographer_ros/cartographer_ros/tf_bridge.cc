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

#include "cartographer_ros/tf_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"

namespace cartographer_ros {

TfBridge::TfBridge(const std::string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}




//传入到 TfBridge::LookupToTracking() 函数的 tracking_frame_ 
//实际为 .lua 文件中的 tracking_frame = “imu_link” 参数
//也就是说当前 trajectory_id 轨迹下的所有数据，都会转换到 imu_link 节点下
// LookupToTracking 查找从frame_id 到frame_id tracking_frame_ 的坐标变换
// imu_link(tracking_frame_) 在 base_link(frame_id) 上方 0.1m处
// 那这里返回的坐标变换的z是 -0.1
std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,//查询该时间点的tf数据s
    const std::string& frame_id) const { //对应配置文件中的 tracking_frame 参数
  ::ros::Duration timeout(lookup_transform_timeout_sec_);//查询超时时间
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
  try {
    //查询tracking_frame_坐标系与frame_id坐标系发布时间
    //最近数据的时间戳。::ros::Time(0.)表示查询最新的数据。
    const ::ros::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
                              timeout)
            .header.stamp;
    const ::ros::Time requested_time = ToRos(time);
    //查询到tf数据的时间，需要比time大，简单的说，就是有新生成
    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      //因为已经有了更新的数据，所以不等待。设置为0
      //否则如果没有新数据，将会进行等待，直到超时
      timeout = ::ros::Duration(0.);
    }
    return absl::make_unique<::cartographer::transform::Rigid3d>(
        ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
                                           requested_time, timeout)));
  } catch (const tf2::TransformException& ex) {//等待超时，即表示没有新数据产生
    LOG(WARNING) << ex.what();
  }
  return nullptr;//超时返回空指针
}

}  // namespace cartographer_ros
