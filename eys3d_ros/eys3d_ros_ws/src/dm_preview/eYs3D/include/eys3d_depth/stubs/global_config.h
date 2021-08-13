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
#ifndef EYS3D_DEPTH_STUBS_GLOBAL_CONFIG_H_
#define EYS3D_DEPTH_STUBS_GLOBAL_CONFIG_H_
#pragma once

#define EYS3D_DEPTH_VERSION_MAJOR 1
#define EYS3D_DEPTH_VERSION_MINOR 0
#define EYS3D_DEPTH_VERSION_PATCH 3
#define EYS3D_DEPTH_VERSION_TWEAK 2

#define BUILD_TIMESTAMP "2021-08-11 16:56:51"
#define GIT_BRANCH "master"
#define GIT_HASH "573dc66"

/* EYS3D_DEPTH_VERSION is (major << 16) + (minor << 8) + patch */
#define EYS3D_DEPTH_VERSION \
EYS3D_DEPTH_VERSION_CHECK( \
  EYS3D_DEPTH_VERSION_MAJOR, \
  EYS3D_DEPTH_VERSION_MINOR, \
  EYS3D_DEPTH_VERSION_PATCH \
)

/* Can be used like
 *   #if (EYS3D_DEPTH_VERSION >= EYS3D_DEPTH_VERSION_CHECK(1, 0, 0)) */
#define EYS3D_DEPTH_VERSION_CHECK(major, minor, patch) \
  ((major<<16)|(minor<<8)|(patch))  // NOLINT

/* EYS3D_DEPTH_VERSION in "X.Y.Z" format */
#define EYS3D_DEPTH_VERSION_STR (EYS3D_DEPTH_STRINGIFY(EYS3D_DEPTH_VERSION_MAJOR.EYS3D_DEPTH_VERSION_MINOR.EYS3D_DEPTH_VERSION_PATCH))  // NOLINT

#define SW_VERSION "1.0.3.2"

#define EYS3D_DEPTH_NAMESPACE eys3d_depth
#if defined(EYS3D_DEPTH_NAMESPACE)
# define EYS3D_DEPTH_BEGIN_NAMESPACE namespace EYS3D_DEPTH_NAMESPACE {
# define EYS3D_DEPTH_END_NAMESPACE }
# define EYS3D_DEPTH_USE_NAMESPACE using namespace ::EYS3D_DEPTH_NAMESPACE;  // NOLINT
#else
# define EYS3D_DEPTH_BEGIN_NAMESPACE
# define EYS3D_DEPTH_END_NAMESPACE
# define EYS3D_DEPTH_USE_NAMESPACE
#endif

#endif  // EYS3D_DEPTH_STUBS_GLOBAL_CONFIG_H_
