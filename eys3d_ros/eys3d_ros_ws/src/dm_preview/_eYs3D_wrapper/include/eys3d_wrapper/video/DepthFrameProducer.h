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

#include <stdint.h>                                   // for uint32_t, uint8_t
#include <memory>                                     // for unique_ptr
#include <list>

#include "video/FrameProducer.h"
#include "devices/CameraDevice.h"

using libeYs3D::devices::CameraDevice;

namespace libeYs3D    {
namespace video    {

class DACalculateWorkItem    {
public:
    std::function<void(Frame *)> callback;
    Frame *frame;

public:
    DACalculateWorkItem(std::function<void(Frame *)> cb, Frame *frame)    {
        this->callback = cb;
        this->frame = frame;
    }

    ~DACalculateWorkItem()    {}
};

class DepthFrameProducer : public FrameProducer    {
public:
    friend std::unique_ptr<FrameProducer> createDepthFrameProducer(CameraDevice *cameraDevice);
    
    virtual const char* getName() override    { return "DepthFrameProducer"; }
    virtual ~DepthFrameProducer();

protected:
    DepthFrameProducer(CameraDevice *cameraDevice);

    virtual int getRawFormatBytesPerPixel(uint32_t format) override;
    virtual int readFrame(Frame *frame) override;
    virtual int produceRGBFrame(Frame *frame) override;
    virtual int performFiltering(Frame *frame) override;
    virtual int performInterleave(Frame *frame) override;
    virtual int performAccuracyComputation(Frame *frame) override;
    
    virtual void checkIMUDeviceCBEnablement() override;
    
    virtual void performSnapshotWork(Frame *frame) override;
    
private:
    std::vector<uint16_t> mTableZ14ToD11;
    std::vector<uint16_t> mZ14ToD11;
    
    std::list<std::vector<int16_t>> mDepthList; // for calculateDepthTemporalNoise
    
    libeYs3D::base::ThreadPool<DACalculateWorkItem> mDACalculateThreadPool;
    libeYs3D::base::MessageChannel<int, 3> mFinishSignal;
    std::function<void(Frame *)> mCalculateDepthAccuracyInfo;
    std::function<void(Frame *)> mCalculateDepthSpatialNoise;
    std::function<void(Frame *)> mCalculateDepthTemporalNoise;

    void calculateDepthAccuracyInfo(Frame *frame);
    void calculateDepthSpatialNoise(Frame *frame);
    void calculateDepthTemporalNoise(Frame *frame);
    uint16_t getDepth(const Frame *frame, int x, int y);
    uint16_t getZValue(const Frame *frame, uint16_t depth);
};  // class FrameProducer

}  // namespace video
}  // namespace libeYs3D
