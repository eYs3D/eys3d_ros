/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "sensors/SensorDataProducer.h"

namespace libeYs3D    {
namespace sensors    {

class IMUDataProducer : public SensorDataProducer    {
public:
    virtual ~IMUDataProducer()   {};

    virtual const char* getName() override    { return "IMUDataProducer"; }
    // returns the actual number of bytes read and negative on error. 
    virtual int readSensorData(SensorData* sensorData) override;
    
    virtual void logProducerTick(const char *FMT, ...) override;
    
protected:
    void virtual attachReaderWorkerCGgroup() override;
    void virtual attachCallbackWorkerCGgroup() override;

protected:
    IMUDataProducer(libeYs3D::devices::IMUDevice *imuDevice);
    
protected:
    libeYs3D::devices::IMUDevice *mIMUDevice;
    
public:
    friend class libeYs3D::devices::IMUDevice;
};

}  // namespace sensors
}  // namespace libeYs3D
