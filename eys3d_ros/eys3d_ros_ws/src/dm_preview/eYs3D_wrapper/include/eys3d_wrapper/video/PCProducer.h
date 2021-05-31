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

#include "base/threads/Thread.h"
#include "video/PCFrame.h"

namespace libeYs3D    {
namespace video    {

class PCProducer : public base::Thread    {
public:
    virtual ~PCProducer()   {};

    // Callback to pass the contents of a new point cloud frame.
    using PCCallback = std::function<bool(const PCFrame* frame)>;

    // Attach the callback used to receiving new Frames
    void attachCallback(PCCallback cb) { mPCCallback = std::move(cb); }
    virtual void enableCallback() = 0;
    virtual void pauseCallback() = 0;
    
    virtual void dumpFrameInfo(int frameCount) = 0;
    virtual void doSnapshot() = 0;
    
    virtual void logProducerTick(const char *FMT, ...) = 0;

    // Stops the producer
    virtual void stop() = 0;
    
    virtual const char* getName() = 0;

protected:
    PCProducer() = default;
    PCProducer(PCProducer&& p) = delete;

protected:
    PCCallback mPCCallback;
    uint32_t mFormat;
};

}  // namespace video
}  // namespace libeYs3D
