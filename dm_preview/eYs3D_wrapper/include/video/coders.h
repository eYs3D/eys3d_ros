/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#ifdef WIN32
#  include "eSPDI_Common.h"
#else
#  include "eSPDI_def.h"
#endif
//#include "devices/CameraDevice.h"

#include <stdint.h>

#define COLOR_ERR_NUM 3
// forward declaration
namespace libeYs3D    {
    namespace devices    {
        class CameraDevice;
    }
}

class CameraDevice;

namespace libeYs3D    {
namespace video    {

struct Frame;

static inline APCImageType::Value depth_raw_type_to_depth_image_type(uint32_t depth_raw_type)    {
    return APCImageType::DepthDataTypeToDepthImageType((unsigned short)depth_raw_type);
}

static inline int get_color_image_format_byte_length_per_pixel(APCImageType::Value format)    {
    switch (format){
        case APCImageType::COLOR_MJPG:
        case APCImageType::COLOR_YUY2:
            return 2;
        case APCImageType::COLOR_RGB24:
            return 3;
        default:
            return 0;
    }
}

static inline int get_depth_image_format_byte_length_per_pixel(APCImageType::Value format)    {
    switch (format){
        case APCImageType::DEPTH_8BITS:
            return 1;
        case APCImageType::DEPTH_8BITS_0x80:
            return 2;
        case APCImageType::DEPTH_11BITS:
            return 2;
        case APCImageType::DEPTH_14BITS:
            return 2;
        default:
            return 0;
    }
}

int color_image_produce_bgr_frame(const CameraDevice *cameraDevice, Frame *frame);
int depth_image_produce_rgb_frame(const CameraDevice *cameraDevice, Frame *frame);

int convert_yuv_to_rgb_buffer(uint8_t *yuv, uint8_t *rgb,
                              int32_t width, int32_t height,
                              uint64_t* rgb_actual_size);

#if 0
int turbo_jpeg_jpeg2yuv(uint8_t* jpeg_buffer, uint64_t jpeg_size, uint8_t *yuv_buffer,
                        uint64_t* yuv_actual_size, int32_t* yuv_type);
int turbo_jpeg_yuv2rgb(uint8_t* yuv_buffer, uint64_t yuv_size, int32_t width, int32_t height,
                       int sub_sample, uint8_t* rgb_buffer, uint64_t* rgb_actual_size);
#endif
} // endo of namespace video
} // endo of namespace libeYs3D
