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
#ifndef EYS3D_UTIL_RATE_H_
#define EYS3D_UTIL_RATE_H_
#pragma once

#include "eys3d_depth/util/times.h"

EYS3D_DEPTH_BEGIN_NAMESPACE

class EYS3D_API Rate {
 public:
  using clock = times::clock;

  explicit Rate(std::int32_t frequency);
  ~Rate();

  void Sleep();

  void Reset();

  clock::duration CycleTime();

  clock::duration ExpectedCycleTime();

 private:
  clock::time_point time_beg_;
  clock::duration expected_cycle_time_, actual_cycle_time_;
};

EYS3D_DEPTH_END_NAMESPACE

#endif  // EYS3D_UTIL_RATE_H_
