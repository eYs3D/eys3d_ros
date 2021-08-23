/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "devices/MemoryAllocator.h"
#include "sensors/SensorData.h"
#include "GeneralFrame.h"

#include <cstdint>                // for uint8_t, uint32_t, uint64_t
#include <vector>                 // for vector
#include <string>

namespace libeYs3D {
namespace video    {

// A small structure to encapsulate frame data.
class PCFrame : public GeneralFrame {
public:
    int64_t tsUs = 0llu;  // timestamp(microsec)
    uint32_t serialNumber; // serial number of this frame
    int32_t width;
    int32_t height;

#ifdef DEVICE_MEMORY_ALLOCATOR
    std::vector<uint8_t, libeYs3D::devices::MemoryAllocator<uint8_t>> drgbDataVec;
    std::vector<uint8_t, libeYs3D::devices::MemoryAllocator<uint8_t>> rgbDataVec;
    std::vector<float, libeYs3D::devices::MemoryAllocator<float>> xyzDataVec;
#else
    std::vector<uint8_t> drgbDataVec;
    std::vector<uint8_t> rgbDataVec;
    std::vector<float> xyzDataVec;
#endif
    /* for performance benchmark purpose in micro seconds*/
    int64_t transcodingTimeUs;
    int64_t colorFrameTsUs;
    int64_t depthFrameTsUs;
    
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
#ifndef WIN32
    int toStringSHA256(char *buffer, int bufferLength) const;
#endif
    int toString(std::string &string) const;
    
    void clone(const PCFrame *pcFrame);

    // Move constructor
    PCFrame(PCFrame&& f) = default;
    // Move assignment
    PCFrame& operator=(PCFrame&& f) = default;
    
     // Do not define default assignment operator to prevent from misusing PCFrame assignment
    PCFrame& operator=(PCFrame& f) = delete;
    
    // hidden attributes for internal only
    bool interleaveMode = false;
};

} // namespace video
} // namespace libeYs3D
