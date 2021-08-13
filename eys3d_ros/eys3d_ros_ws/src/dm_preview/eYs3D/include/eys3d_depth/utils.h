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
#ifndef EYS3D_UTILS_H_
#define EYS3D_UTILS_H_
#pragma once

#include "eys3d_depth/camera.h"

EYS3D_DEPTH_BEGIN_NAMESPACE

namespace util {

EYS3D_API
bool select(const Camera& cam, DeviceInfo* info);

EYS3D_API
void print_stream_infos(const Camera& cam, const std::int32_t& dev_index);

EYS3D_API
bool is_right_color_supported(const StreamMode& mode);

}  // namespace util

EYS3D_DEPTH_END_NAMESPACE

#endif  // EYS3D_UTILS_H_
