/*
 * Copyright (C) 2022 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#ifndef EYS3DPY_CAMERADEVICE_8073_H
#define EYS3DPY_CAMERADEVICE_8073_H


#include "CameraDevice.h"

namespace libeYs3D {
namespace devices {

class CameraDevice_8073 : public CameraDevice {
public:
    friend class CameraDeviceFactory;
    const char* LOG_TAG = "CameraDevice_8073";
    explicit CameraDevice_8073(DEVSELINFO *devSelInfo, DEVINFORMATION *deviceInfo);
    explicit CameraDevice_8073(DEVSELINFO *devSelInfo, DEVINFORMATION *deviceInfo, COLOR_BYTE_ORDER colorByteOrder);

    int initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE colorFormat, int32_t colorWidth, int32_t colorHeight,
                   int32_t actualFps, libeYs3D::video::DEPTH_RAW_DATA_TYPE depthFormat, int32_t depthWidth,
                   int32_t depthHeight, DEPTH_TRANSFER_CTRL depthDataTransferCtrl, CONTROL_MODE ctrlMode,
                   int rectifyLogIndex, libeYs3D::video::Producer::Callback colorImageCallback,
                   libeYs3D::video::Producer::Callback depthImageCallback,
                   libeYs3D::video::PCProducer::PCCallback pcFrameCallback,
                   libeYs3D::sensors::SensorDataProducer::AppCallback imuDataCallback) override;

    bool isInterleaveModeSupported() override;

    int setIRMax(bool ExtendIREnabled) override;

protected:
    int getZDTableIndex() override;

};

} // -- namespace libeYs3D
} // -- namespace devices

#endif //EYS3DPY_CAMERADEVICE_8073_H
