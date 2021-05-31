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
#include "devices/model/DepthAccuracyOptions.h"
#include "sensors/SensorData.h"
#include "base/synchronization/Lock.h"

#include <cstdint>                // for uint8_t, uint32_t, uint64_t
#include <vector>                 // for vector
#include <string>

namespace libeYs3D {
namespace video    {

//forward declaration
class FrameProducer; 

// A small structure to encapsulate frame data.
struct Frame    {
    int64_t tsUs = 0ll;    // timestamp(microsec)
    uint32_t serialNumber; // serial number of this frame
    int32_t width;
    int32_t height;
    uint64_t actualDataBufferSize;  // the actual buffer size getting from device
    uint64_t dataBufferSize;        // the data length of dataVec
    
#ifdef DEVICE_MEMORY_ALLOCATOR
    std::vector<uint8_t, libeYs3D::devices::MemoryAllocator<uint8_t>> dataVec;
#else
    std::vector<uint8_t> dataVec;
#endif

    uint64_t actualZDDepthBufferSize;   // the actual buffer size when converting from raw data to ZD table depth
    uint64_t zdDepthBufferSize;         // the image buffer size of zdDepthVec

//TODO:  DEVICE_MEMORY_ALLOCATOR support is not ready for zdDepthVec
#ifdef DEVICE_MEMORY_ALLOCATOR
    std::vector<uint16_t, libeYs3D::devices::MemoryAllocator<uint16_t>> zdDepthVec;
#else
    std::vector<uint16_t> zdDepthVec;
#endif

    uint64_t actualRGBBufferSize;   // the actual buffer size when converting the image
    uint64_t rgbBufferSize;         // the image buffer size of imageVec

#ifdef DEVICE_MEMORY_ALLOCATOR
    std::vector<uint8_t, libeYs3D::devices::MemoryAllocator<uint8_t>> rgbVec;
#else
    std::vector<uint8_t> rgbVec;
#endif

    /*
     * for color frame:
     *     libeYs3D::video::COLOR_RAW_DATA_TYPE
     * for depth frame:
     *     ibeYs3D::video::DEPTH_RAW_DATA_TYPE
     */
    uint32_t dataFormat;
    
    /* TODO:
     * the format of RGB transcoding format 
     */
    uint32_t rgbFormat;

    /* for performance benchmark purpose in micro seconds */
    int64_t rgbTranscodingTime;
    int64_t filteringTime;
    
    libeYs3D::sensors::SensorDataSet sensorDataSet;
    
    union extra    {
        struct libeYs3D::devices::DepthAccuracyInfo depthAccuracyInfo;
        
        extra() { memset(this, 0, sizeof(extra)); }
        ~extra() = default;
    } extra;

    // Allocates the space with capacity |cap| and sets each element to |val|
    Frame(uint64_t dataBufferSize = 0, uint8_t initDataVal = 0,
          uint64_t zdDepthBufferSize = 0, uint16_t initZDDepthVal = 0,
          uint64_t rgbBufferSize = 0, uint8_t initRGBVal = 0);
    
#ifdef DEVICE_MEMORY_ALLOCATOR
    Frame(uint64_t dataBufferSize, uint64_t rgbBufferSize, uint8_t val,
          libeYs3D::devices::MemoryAllocator<uint8_t> &allocator);
#endif

    int toString(char *buffer, int bufferLength) const;
    int toStringSimple(char *buffer, int bufferLength) const;
    int toStringFull(char *buffer, int bufferLength) const;
    int toString(std::string &string) const;
    int saveToFile(const char *dirPath) const;
    
    void clone(const Frame *frame);

    // Move constructor
    Frame(Frame&& f) = default;
    // Move assignment
    Frame& operator=(Frame&& f) = default;
    
    // hidden attributes for internal only
    bool toCallback = false;
    bool toPCCallback = false;
    bool interleaveMode = false;
};

} // namespace video
} // namespace libeYs3D
