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
    virtual int performFiltering(Frame *frame) override;
    virtual int performInterleave(Frame *frame) override;
    virtual int performAccuracyComputation(Frame *frame) override;
    virtual int performROIComputation(Frame *frame) override;
    
    virtual void checkIMUDeviceCBEnablement() override;
    
    virtual void performSnapshotWork(Frame *frame) override;
    
    virtual void logProducerTick(const char *FMT, ...) override;
    
protected:
    void virtual attachReaderWorkerCGgroup() override;
    void virtual attachRGBWorkerCGgroup() override;
    void virtual attachFilterWorkerCGgroup() override;
    void virtual attachCallbackWorkerCGgroup() override;
};  // class FrameProducer


}  // namespace video
}  // namespace libeYs3D
