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

#include "eSPDI_def.h"

namespace libeYs3D    {
namespace video    {

typedef enum    {
    COLOR_RAW_DATA_YUY2 = EtronDIImageType::COLOR_YUY2,
    COLOR_RAW_DATA_MJPG = EtronDIImageType::COLOR_MJPG,
} COLOR_RAW_DATA_TYPE;

typedef enum    {
    DEPTH_RAW_DATA_8_BITS = ETronDI_DEPTH_DATA_8_BITS,
    DEPTH_RAW_DATA_11_BITS = ETronDI_DEPTH_DATA_11_BITS,
    DEPTH_RAW_DATA_14_BITS = ETronDI_DEPTH_DATA_14_BITS,
} DEPTH_RAW_DATA_TYPE;

} // namespace video
} // namespace libeYs3D
