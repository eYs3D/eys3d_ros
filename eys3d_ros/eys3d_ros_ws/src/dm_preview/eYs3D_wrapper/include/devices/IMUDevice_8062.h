/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "devices/IMUDevice.h"

namespace libeYs3D    {

namespace sensors    {
    // forward declaration
    class IMUDataProducer;
}

namespace devices    {

// forward declaration
class CameraDevice;
class CameraDevice8062;

class IMUDevice8062 : public IMUDevice    {
public:
    virtual ~IMUDevice8062();
    
    virtual const char* getName() override { return "IMUDevice 8062"; }

protected:
    IMUDevice8062(IMUDeviceInfo info, CameraDevice *cameraDevice);

    virtual std::vector<IMUDevice::IMU_DATA_FORMAT> getSupportDataFormat() override;
    virtual int selectDataFormat(IMU_DATA_FORMAT format) override;
    virtual void readDataOutputFormat() override;

public:
    friend class CameraDevice;
    friend class CameraDevice8062;
};

} // end of namespace devices
} // end of namespace libeYs3D
