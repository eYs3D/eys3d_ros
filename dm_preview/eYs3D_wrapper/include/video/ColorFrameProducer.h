/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <stdint.h>                                   // for uint32_t, uint8_t
#include <memory>                                     // for unique_ptr

#include "video/FrameProducer.h"
#include "devices/CameraDevice.h"

namespace libeYs3D    {
namespace video    {

using libeYs3D::devices::CameraDevice;

class ColorFrameProducer : public FrameProducer {
public:
    friend std::unique_ptr<FrameProducer> createColorFrameProducer(CameraDevice *cameraDevice);
    
    virtual const char* getName() override    { return "ColorFrameProducer"; }
    virtual ~ColorFrameProducer() {}

protected:
    ColorFrameProducer(CameraDevice *cameraDevice);
    virtual int getRawFormatBytesPerPixel(uint32_t format) override;
    virtual int readFrame(Frame *frame) override;
    virtual int produceRGBFrame(Frame *frame) override;
    virtual int performPostProcessFilter(Frame *frame) override;
    int getFilteredWidth() override;
    int getFilteredHeight() override;
    virtual int performFiltering(Frame *frame) override;
    virtual int performInterleave(Frame *frame) override;
    virtual int performAccuracyComputation(Frame *frame) override;
    virtual int performROIComputation(Frame *frame) override;
    int m_nLastInterLeaveColorSerial;
    
    virtual void checkIMUDeviceCBEnablement() override;

    virtual void performSnapshotWork(Frame *frame) override;

    virtual void logProducerTick(const char *FMT, ...) override;
    
protected:
    void virtual attachReaderWorkerCGgroup() override;
    void virtual attachRGBWorkerCGgroup() override;
    void virtual attachFilterWorkerCGgroup() override;
    void virtual attachCallbackWorkerCGgroup() override;

private:
    std::unique_ptr<AbstractPostProcessFactory> mColorProcessFactory;
    std::unique_ptr<IImageProcess> mColorProcessHandle;
    PostProcessHandleCallback mColorProcessCameraParamsUpdateCallback;
};  // class FrameProducer


}  // namespace video
}  // namespace libeYs3D
