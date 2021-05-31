/*
 * Copyright (C) 2015-2017 ICL/ITRI
 * All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of ICL/ITRI and its suppliers, if any.
 * The intellectual and technical concepts contained
 * herein are proprietary to ICL/ITRI and its suppliers and
 * may be covered by Taiwan and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from ICL/ITRI.
 */

#pragma once

#include "video/Frame.h"
#include "eSPDI_def.h"
#include "DMPreview_utility/PlyWriter.h"

#include <stdint.h>
#include <vector>

// forward declaration
namespace libeYs3D    {
    namespace devices    {
        class CameraDevice;
    }
}

using libeYs3D::devices::CameraDevice;

namespace libeYs3D    {
namespace video    {

int generate_point_cloud(CameraDevice *cameraDevice, const char *plyFilePath,
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
