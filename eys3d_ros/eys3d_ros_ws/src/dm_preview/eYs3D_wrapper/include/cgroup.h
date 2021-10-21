/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#define CAMERA_READER_CGROUP      "eYs3D/readers"
#define IMU_READER_CGROUP         "eYs3D/readers"

#define COLOR_CODER_CGROUP        "eYs3D/coders"
#define DEPTH_RGB_CODER_CGROUP    "eYs3D/depth_rgb_coder"
#define DEPTH_FILTER_CODER_CGROUP "eYs3D/depth_filter_coder"
#define PC_CODER_CGROUP           "eYs3D/coders"

#define COLOR_CALLBACK_CGROUP     "eYs3D/callbacks"
#define DEPTH_CALLBACK_CGROUP     "eYs3D/callbacks"
#define PC_CALLBACK_CGROUP        "eYs3D/callbacks"
#define IMU_CALLBACK_CGROUP       "eYs3D/callbacks"

void initialize_cgroup();
void attach_to_cgroup(const char *cgroupName, const char *log_tag);
