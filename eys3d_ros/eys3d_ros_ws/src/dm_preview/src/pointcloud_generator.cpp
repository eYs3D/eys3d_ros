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
#include "pointcloud_generator.h"

#include <utility>

EYS3D_DEPTH_USE_NAMESPACE

PointCloudGenerator::PointCloudGenerator(Callback callback,
                                         double factor, std::int32_t frequency)
    : callback_(std::move(callback)),
      rate_(nullptr),
      running_(false),
      generating_(false),
      factor_(1.0 / factor),
      frequency_(frequency)
{
  if (frequency > 0) {
    rate_.reset(new EYS3D_DEPTH_NAMESPACE::Rate(frequency));
  }
  Start();
}

PointCloudGenerator::PointCloudGenerator(Callback callback,
                                         eYs3DPlyParameters ply_parmeters,
                                         OpenParams params,
                                         double factor,
                                         std::int32_t frequency)
    : PointCloudGenerator(callback, factor, frequency)
{
  ply_parmeters_ = std::move(ply_parmeters);
  params_ = params;

  int outputWidth = 0, outputHeight = 0;
  switch (params_.stream_mode)
  {
    case StreamMode::STREAM_640x480:
      outputWidth = 640;
      outputHeight = 480;
      break;
    case StreamMode::STREAM_640x400:
      outputWidth = 640;
      outputHeight = 480;
      break;
    case StreamMode::STREAM_1280x480:
      outputWidth = 1280;
      outputHeight = 480;
      break;
    case StreamMode::STREAM_1280x720:
    case StreamMode::STREAM_2560x720:
      outputWidth = 1280;
      outputHeight = 720;
      break;
    default:
      outputWidth = params.depth_width;
      outputHeight = params.depth_height;
      break;
  }

  float ratio_Mat = (float)outputHeight / ply_parmeters_.out_height;
  float focalLength = ply_parmeters_.ReProjectMat[11] * ratio_Mat;
  float baseline = 1.0f / ply_parmeters_.ReProjectMat[14];
  float diff = ply_parmeters_.ReProjectMat[15] * ratio_Mat;

  float disparityToW[2048];
  int tableSize;
  switch (params_.depth_data_type)
  {
    case DepthDataType::DEPTH_DATA_8_BITS:
      tableSize = 1 << 8;
      for (int i = 0; i < tableSize; i++)
      {
        disparityToW[i] = (i * ratio_Mat) / baseline + diff;
        disparityToW[i] = 1.0 / disparityToW[i];
      }
      break;
    case DepthDataType::DEPTH_DATA_11_BITS:
    {
      tableSize = 1 << 11;
      for (int i = 0; i < tableSize; i++)
      {
        disparityToW[i] = (i * ratio_Mat / 8.0f) / baseline + diff;
        disparityToW[i] = 1.0 / disparityToW[i];
      }
      break;
    }
    case DepthDataType::DEPTH_DATA_14_BITS:
      break;
  }

  switch (params_.depth_data_type)
  {
    case DepthDataType::DEPTH_DATA_8_BITS:
      pointColudTable_.resize(1 << 8); break;
    case DepthDataType::DEPTH_DATA_11_BITS :
      pointColudTable_.resize(1 << 11); break;
    case DepthDataType::DEPTH_DATA_14_BITS:
      pointColudTable_.resize(1 << 14); break;
  }

  for (size_t d = 1; d < pointColudTable_.size() ; ++d)
  {
    float W = 0.0f, X = 0.0f, Y = 0.0f, Z = 0.0f;
    switch (params_.depth_data_type)
    {
      case DepthDataType::DEPTH_DATA_8_BITS:
      case DepthDataType::DEPTH_DATA_11_BITS:
        if (d >= tableSize)
          break;
        W = disparityToW[d];
        if (W > 0.0f)
        {
          Z = focalLength * W;
        }
        break;
      case DepthDataType::DEPTH_DATA_14_BITS:
        W = d / focalLength;
        Z = d;
        break;
    }

    if (0.0f == W || Z > params_.colour_depth_value)
    {
      continue;
    }

    pointColudTable_[d].W = W * factor_;
    pointColudTable_[d].Z = Z * factor_;
  }

  centerX_ = -1.0f * ply_parmeters_.ReProjectMat[3] * ratio_Mat;
  centerY_ = -1.0f * ply_parmeters_.ReProjectMat[7] * ratio_Mat;

  msg_.width = outputWidth;
  msg_.height = outputHeight;
  msg_.is_dense = true;

  modifier_ = std::make_shared<sensor_msgs::PointCloud2Modifier>(msg_);
  modifier_->setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "rgb", 1, sensor_msgs::PointField::FLOAT32);
  modifier_->setPointCloud2FieldsByString(2, "xyz", "rgb");
}

PointCloudGenerator::~PointCloudGenerator() {
  Stop();
}

bool PointCloudGenerator::Push(const cv::Mat& color, const cv::Mat& depth,
    ros::Time stamp) {
  if (!running_) {
    throw new std::runtime_error("Start first!");
  }

  if (generating_) return false;

  {
    std::lock_guard<std::mutex> _(mutex_);
    if (generating_) return false;
    generating_ = true;
    color_ = color.clone();
    depth_ = depth.clone();
    stamp_ = stamp;
  }

  condition_.notify_one();
  return true;
}

void PointCloudGenerator::Start() {
  if (running_) return;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (running_) return;
    running_ = true;
  }

  thread_ = std::thread(&PointCloudGenerator::Run, this);
}

void PointCloudGenerator::Stop() {
  if (!running_) return;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (!running_) return;
    running_ = false;
    generating_ = true;
  }

  condition_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void PointCloudGenerator::Run() {
  while (running_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return generating_; });
      if (!running_) break;
    }

    msg_.header.stamp = stamp_;

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg_, "z");

    sensor_msgs::PointCloud2Iterator<uchar> iter_r(msg_, "r");
    sensor_msgs::PointCloud2Iterator<uchar> iter_g(msg_, "g");
    sensor_msgs::PointCloud2Iterator<uchar> iter_b(msg_, "b");

    ushort d;
    for (float m = 0, Y = -centerY_; m < depth_.rows; ++m, ++Y)
    {
      for (float n = 0, X = -centerX_ ; n < depth_.cols; ++n, ++X)
      {
        d = depth_.ptr<ushort>(m)[(int)n];
        if (pointColudTable_[d].W)
        {
          *iter_x = pointColudTable_[d].W * X;
          *iter_y = pointColudTable_[d].W * Y;
        }else
        {
          *iter_x = *iter_y = 0.0f;
        }

        *iter_z = pointColudTable_[d].Z;
        if (DeviceMode::DEVICE_DEPTH == params_.dev_mode)
        {
          *iter_r = 0;
          *iter_g = 255;
          *iter_b = 0;
        }else
        {
          *iter_r = color_.ptr<uchar>(m)[(int)n * 3 + 2];
          *iter_g = color_.ptr<uchar>(m)[(int)n * 3 + 1];
          *iter_b = color_.ptr<uchar>(m)[(int)n * 3];
        }

        ++iter_x; ++iter_y; ++iter_z; ++iter_r; ++iter_g; ++iter_b;
      }
    }

    if (callback_) {
      callback_(msg_);
    }

    if (frequency_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(frequency_));
    }
    
    {
      std::lock_guard<std::mutex> _(mutex_);
      generating_ = false;
    }
  }
}
