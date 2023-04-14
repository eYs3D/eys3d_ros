/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "base/threads/Thread.h"
#include "video/Frame.h"

namespace libeYs3D    {
namespace video    {

class Producer : public base::Thread    {
public:
    virtual ~Producer()   {};

    // Callback to pass the contents of a new color/depth frame.
    using Callback = std::function<bool(const Frame* frame)>;

    // Attach the callback used to receiving new Frames
    void attachCallback(Callback cb) { mCallback = std::move(cb); }

    // Stops the producer
    virtual void stop() = 0;
    
    virtual const char* getName() = 0;

protected:
    Producer() = default;
    Producer(Producer&& p) = delete;

protected:
    Callback mCallback;
    uint32_t mPixelRawFormat;
};

}  // namespace video
}  // namespace libeYs3D
