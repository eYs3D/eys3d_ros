// Copyright 2020 eYs3D Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef EYS3D_WRAPPER_POINTCLOUD_GENERATOR_H_
#define EYS3D_WRAPPER_POINTCLOUD_GENERATOR_H_
#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <opencv2/core/core.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "eys3d_depth/util/rate.h"
#include "eys3d_depth/stubs/types_calib.h"
#include "eys3d_depth/camera.h"

EYS3D_DEPTH_BEGIN_NAMESPACE

#define DEFAULT_POINTS_FREQUENCE (0)
#define DEFAULT_POINTS_FACTOR (1000.0)

class PointCloudGenerator {
 public:
  using Callback = std::function<void(sensor_msgs::PointCloud2&)>;

  PointCloudGenerator(Callback callback,
                      double factor = DEFAULT_POINTS_FACTOR,
                      std::int32_t frequency = DEFAULT_POINTS_FREQUENCE);
  PointCloudGenerator(Callback callback,
                      eYs3DPlyParameters ply_parmeters,
                      OpenParams params,
                      double factor = DEFAULT_POINTS_FACTOR,
                      std::int32_t frequency = DEFAULT_POINTS_FREQUENCE);
  ~PointCloudGenerator();

  bool Push(const cv::Mat& color, const cv::Mat& depth, ros::Time stamp);

  double factor() { return factor_; }
  void set_factor(double factor) { factor_ = factor; }

 private:
  void Start();
  void Stop();

  void Run();

  CameraIntrinsics in_; 

  sensor_msgs::PointCloud2 msg_;
  std::shared_ptr<sensor_msgs::PointCloud2Modifier> modifier_;

  int frequency_;

  eYs3DPlyParameters ply_parmeters_;
  OpenParams params_;

  struct PointCloudField
  {
    float W = 0.0f;
    float Z = 0.0f;
  };

  std::vector<PointCloudField> pointColudTable_;
  float centerX_;
  float centerY_;

  Callback callback_;

  std::unique_ptr<EYS3D_DEPTH_NAMESPACE::Rate> rate_;

  std::mutex mutex_;
  std::condition_variable condition_;

  bool running_;
  std::thread thread_;

  cv::Mat color_;
  cv::Mat depth_;
  ros::Time stamp_;

  double factor_;

  bool generating_;
};

EYS3D_DEPTH_END_NAMESPACE

#endif  // EYS3D_WRAPPER_POINTCLOUD_GENERATOR_H_
