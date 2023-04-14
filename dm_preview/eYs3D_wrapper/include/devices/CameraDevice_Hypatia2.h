/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "CameraDevice.h"

namespace libeYs3D
{
    namespace devices
    {

        class CameraDeviceHypatia2 : public CameraDevice
        {
        public:
            friend class CameraDeviceFactory;

            virtual int initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE colorFormat,
                                   int32_t colorWidth, int32_t colorHeight, int32_t actualFps,
                                   libeYs3D::video::DEPTH_RAW_DATA_TYPE depthFormat,
                                   int32_t depthWidth, int32_t depthHeight,
                                   DEPTH_TRANSFER_CTRL depthDataTransferCtrl,
                                   CONTROL_MODE ctrlMode,
                                   int rectifyLogIndex,
                                   libeYs3D::video::Producer::Callback colorImageCallback,
                                   libeYs3D::video::Producer::Callback depthImageCallback,
                                   libeYs3D::video::PCProducer::PCCallback pcFrameCallback,
                                   libeYs3D::sensors::SensorDataProducer::AppCallback imuDataCallback = nullptr) override;

            virtual ~CameraDeviceHypatia2() = default;

            explicit CameraDeviceHypatia2(DEVSELINFO *devSelInfo, DEVINFORMATION *deviceInfo);
            explicit CameraDeviceHypatia2(DEVSELINFO *devSelInfo, DEVINFORMATION *deviceInfo, COLOR_BYTE_ORDER colorByteOrder);
        };

    } // end of namespace devices
} // end of namespace libeYs3D
