/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "video/Frame.h"
#include "video/PCFrame.h"

namespace libeYs3D {
namespace video    {

// A small structure to encapsulate frame data.
struct FrameSet    {
    struct Frame colorFrame;
    struct Frame depthFrame;
    struct PCFrame pcFrame;
};

} // namespace video
} // namespace libeYs3D
