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

#include "devices/MemoryAllocator.h"
#include "sensors/SensorData.h"

#include <cstdint>                // for uint8_t, uint32_t, uint64_t
#include <vector>                 // for vector
#include <string>

namespace libeYs3D {
namespace video    {

// A small structure to encapsulate frame data.
struct PCFrame    {
    int64_t tsUs = 0llu;  // timestamp(microsec)
    uint32_t serialNumber; // serial number of this frame
    int32_t width;
    int32_t height;

#ifdef DEVICE_MEMORY_ALLOCATOR
    std::vector<uint8_t, libeYs3D::devices::MemoryAllocator<uint8_t>> rgbDataVec;
    std::vector<float, libeYs3D::devices::MemoryAllocator<float>> xyzDataVec;
#else
    std::vector<uint8_t> rgbDataVec;
    std::vector<float> xyzDataVec;
#endif    
    /* for performance benchmark purpose in micro seconds*/
    int64_t transcodingTime;
    
    libeYs3D::sensors::SensorDataSet sensorDataSet;

    // Allocates the space with capacity |cap| and sets each element to |val|
    PCFrame(uint32_t pixelCount = 0, uint8_t val = 0, float fVal = 0.0);

#ifdef DEVICE_MEMORY_ALLOCATOR
    PCFrame(uint32_t pixelCount, uint8_t val, float fVal,
            libeYs3D::devices::MemoryAllocator<uint8_t> &byteAllocator,
            libeYs3D::devices::MemoryAllocator<float> &floatAllocator);
#endif

    int toString(char *buffer, int bufferLength) const;
    int toStringSimple(char *buffer, int bufferLength) const;
    int toString(std::string &string) const;

    // Move constructor
    PCFrame(PCFrame&& f) = default;
    // Move assignment
    PCFrame& operator=(PCFrame&& f) = default;
    
    // hidden attributes for internal only
    bool interleaveMode = false;
};

} // namespace video
} // namespace libeYs3D
