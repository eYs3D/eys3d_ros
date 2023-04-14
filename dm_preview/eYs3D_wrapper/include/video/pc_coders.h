/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "video/Frame.h"
#ifdef WIN32
#  include <eSPDI_Common.h>
#else
#  include <eSPDI_def.h>
#endif
#include "DMPreview_utility/PlyWriter.h"

#include <stdint.h>
#include <vector>

// forward declaration
namespace libeYs3D    {
    namespace devices    {
        class CameraDevice;
    }
}

class CameraDevice;

namespace libeYs3D    {
namespace video    {

int generate_point_cloud(std::shared_ptr<CameraDevice> cameraDevice, const char *plyFilePath,
                         std::vector<uint8_t> &depthData,
                         int nDepthWidth, int nDepthHeight,
                         std::vector<uint8_t> &colorData,
                         int nColorWidth, int nColorHeight,
                         bool usePlyFilter);

int save_bitmap(const char *filePath, uint8_t *buffer, int width, int height);
int save_yuv(const char *filePath, uint8_t *buffer, int width, int height, int bytePerPixel);
int save_ply(const char *filePath, std::vector<CloudPoint> &cloudPoints, bool launchMeshlab = true);

} // endo of namespace video
} // endo of namespace libeYs3D
