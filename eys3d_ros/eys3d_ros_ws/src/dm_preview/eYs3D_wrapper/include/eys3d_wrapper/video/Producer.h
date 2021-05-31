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
