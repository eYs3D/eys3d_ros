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
#ifndef EYS3D_STUBS_GLOBAL_H_
#define EYS3D_STUBS_GLOBAL_H_
#pragma once

#ifdef _WIN32
  #define EYS3D_OS_WIN
  #ifdef _WIN64
    #define EYS3D_OS_WIN64
  #else
    #define EYS3D_OS_WIN32
  #endif
  #if defined(__MINGW32__) || defined(__MINGW64__)
    #define EYS3D_OS_MINGW
    #ifdef __MINGW64__
      #define EYS3D_OS_MINGW64
    #else
      #define EYS3D_OS_MINGW32
    #endif
  #elif defined(__CYGWIN__) || defined(__CYGWIN32__)
    #define EYS3D_OS_CYGWIN
  #endif
#elif __APPLE__
  #include <TargetConditionals.h>
  #if TARGET_IPHONE_SIMULATOR
    #define EYS3D_OS_IPHONE
    #define EYS3D_OS_IPHONE_SIMULATOR
  #elif TARGET_OS_IPHONE
    #define EYS3D_OS_IPHONE
  #elif TARGET_OS_MAC
    #define EYS3D_OS_MAC
  #else
    #error "Unknown Apple platform"
  #endif
#elif __ANDROID__
  #define EYS3D_OS_ANDROID
#elif __linux__
  #define EYS3D_OS_LINUX
#elif __unix__
  #define EYS3D_OS_UNIX
#elif defined(_POSIX_VERSION)
  #define EYS3D_OS_POSIX
#else
  #error "Unknown compiler"
#endif

#if defined(EYS3D_OS_WIN) && !defined(EYS3D_OS_MINGW)
  #define EYS3D_OS_SEP "\\"
#else
  #define EYS3D_OS_SEP "/"
#endif

#if defined(EYS3D_OS_WIN)
  #define EYS3D_DECL_EXPORT __declspec(dllexport)
  #define EYS3D_DECL_IMPORT __declspec(dllimport)
  #define EYS3D_DECL_HIDDEN
#else
  #define EYS3D_DECL_EXPORT __attribute__((visibility("default")))
  #define EYS3D_DECL_IMPORT __attribute__((visibility("default")))
  #define EYS3D_DECL_HIDDEN __attribute__((visibility("hidden")))
#endif

#ifdef DOXYGEN_WORKING
  #define EYS3D_API
#else
  #ifdef EYS3D_DEPTH_EXPORTS
    #define EYS3D_API EYS3D_DECL_EXPORT
  #else
    #define EYS3D_API EYS3D_DECL_IMPORT
  #endif
#endif

#if (defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L || \
    (defined(_MSC_VER) && _MSC_VER >= 1900))
#define EYS3D_LANG_CXX11 1
#endif

#define EYS3D_STRINGIFY_HELPER(X) #X
#define EYS3D_STRINGIFY(X) EYS3D_STRINGIFY_HELPER(X)

#define EYS3D_DISABLE_COPY(Class) \
  Class(const Class&) = delete; \
  Class& operator=(const Class&) = delete;

#define EYS3D_DISABLE_MOVE(Class) \
  Class(Class&&) = delete; \
  Class& operator=(Class&&) = delete;

#include "eys3d_depth/stubs/global_config.h"

EYS3D_DEPTH_BEGIN_NAMESPACE

template <typename... T>
void UNUSED(T&&...) {}

#define EYS3D_IMU_DEVICE

#define EYS3D_DEPRECATED_COMPAT

EYS3D_DEPTH_END_NAMESPACE

#endif  // EYS3D_STUBS_GLOBAL_H_
