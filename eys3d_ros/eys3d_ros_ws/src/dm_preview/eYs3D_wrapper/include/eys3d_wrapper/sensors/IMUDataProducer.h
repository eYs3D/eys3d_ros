/*
 * Copyright (C) 2015-2019 ICL/ITRI
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
