/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
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
