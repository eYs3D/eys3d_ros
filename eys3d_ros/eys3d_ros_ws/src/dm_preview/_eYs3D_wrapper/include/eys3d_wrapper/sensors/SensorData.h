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

#include "DMPreview_utility/IMUData.h"

#include <stdint.h>
#include <vector>

namespace libeYs3D    {
namespace sensors    {

struct SensorData    {
    enum SensorDataType    {
        IMU_DATA = 0,
        UNKNOWN,
    };

    union SensorDataRawUnion    {
       IMUData imuData;
    };

    uint8_t data[((sizeof(SensorDataRawUnion) + 32) & (~32))];
    SensorDataType type;
    uint32_t serialNumber; // serial number of this data
    
    SensorData(SensorDataType type = SensorDataType::UNKNOWN);
    
    // Move constructor
    SensorData(SensorData&& sd) = default;
    // Move assignment
    SensorData& operator=(SensorData&& sd) = default;
    
    int toString(char *buffer, int bufferLength) const;
    int toStringFull(char *buffer, int bufferLength) const;

    // hidden attributes for internal only
    bool toAppCallback = false;
    
    void clone(const SensorData *sensorData);
};

struct SensorDataSet    {
    int64_t tsUs = 0ll;    // timestamp(microsec)
    uint32_t serialNumber; // serial number of this sensor data
    SensorData::SensorDataType sensorDataType;

    int actualDataCount;
    std::vector<SensorData> sensorDataVec;
    
    // Allocates the space with capacity |cap|
    SensorDataSet(SensorData::SensorDataType type = SensorData::SensorDataType::UNKNOWN, int32_t cap = 0);
    
    // Move constructor
    SensorDataSet(SensorDataSet&& sds) = default;
    // Move assignment
    SensorDataSet& operator=(SensorDataSet&& sds) = default;
    
    int toString(char *buffer, int bufferLength) const;
    void reset();
    void addClone(const SensorData *sensorData);
    void addClone(const SensorDataSet *sensorDataSet);
    void clone(const SensorDataSet *sensorDataSet);

    // hidden attributes for internal only
    bool toColorCallback = false;
    bool toDepthCallback = false;
};

}  // namespace sensors
}  // namespace libeYs3D
