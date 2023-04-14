/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <stdint.h>                                   // for uint32_t, uint8_t
#include <memory>                                     // for unique_ptr
#include <list>

#include "video/FrameProducer.h"
#include "devices/CameraDevice.h"
#include "coders.h"
#include "PostProcessOptions.h"
#include "IImageProcess.h"
#include "factory.h"

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
    virtual ~DepthFrameProducer() override;

protected:
    DepthFrameProducer(CameraDevice *cameraDevice);

    virtual int getRawFormatBytesPerPixel(uint32_t format) override;
    virtual int readFrame(Frame *frame) override;
    virtual int performPostProcessFilter(Frame *frame) override;
    int getFilteredWidth() override;
    int getFilteredHeight() override;
    virtual int produceRGBFrame(Frame *frame) override;
    virtual int performFiltering(Frame *frame) override;
    virtual int performInterleave(Frame *frame) override;
    virtual int performAccuracyComputation(Frame *frame) override;
    virtual int performROIComputation(Frame *frame) override;
    
    virtual void checkIMUDeviceCBEnablement() override;

    virtual void performSnapshotWork(Frame *frame) override;

    virtual void logProducerTick(const char *FMT, ...) override;
    int m_nLastInterLeaveDepthSerial;
    
protected:
    void virtual attachReaderWorkerCGgroup() override;
    void virtual attachRGBWorkerCGgroup() override;
    void virtual attachFilterWorkerCGgroup() override;
    void virtual attachCallbackWorkerCGgroup() override;
    
private:
    libeYs3D::base::Lock mLock;
    std::vector<uint16_t> mTableZ14ToD11;
    std::vector<uint16_t> mZ14ToD11;
    
    std::list<std::vector<int16_t>> mDepthList; // for calculateDepthTemporalNoise
    std::unique_ptr<AbstractPostProcessFactory> mPostProcessFactory;
    std::unique_ptr<IImageProcess> mPostProcessHandle;
    PostProcessHandleCallback mPostProcessCameraParamsUpdateCallback;
    libeYs3D::base::ThreadPool<DACalculateWorkItem> mDACalculateThreadPool;
    libeYs3D::base::MessageChannel<int, 3> mFinishSignalForAccuracy;
    libeYs3D::base::MessageChannel<int, 1> mFinishSignalForROI;
    std::function<void(Frame *)> mCalculateDepthAccuracyInfo;
    std::function<void(Frame *)> mCalculateDepthSpatialNoise;
    std::function<void(Frame *)> mCalculateDepthTemporalNoise;
    //std::function<void(Frame *)> mCalculateROIComputation;
    
    void calculateDepthAccuracyInfo(Frame *frame);
    void calculateDepthSpatialNoise(Frame *frame);
    void calculateDepthTemporalNoise(Frame *frame);
    //void calculateROIComputation(Frame *frame);

    bool mIsFinishedForAccuracy;
    int mMessageCountForAccuracy;
    libeYs3D::video::Frame mFrameForAccuracy;
    libeYs3D::video::Frame mFrameTempForAccuracy;
    int64_t mCurrentTimeForAccuracy;
    int64_t mNewTimeForAccuracy;
    int mSignalControlForAccuracy;
    std::vector< WORD > GetDepthZOfROI(Frame *frame , int &nWidth, int &nHeight);
    void CalculateFittedPlane(double &a, double &b, double &d,
                              std::vector< WORD > &vecDepthZ, int nWidth , int nHeigth);
    void SortZ(std::vector< WORD > &vecDepthZ, double dblDeleteBoundaryRatio = 0.005);
    std::vector< double > vectorCrossProduct(std::vector< double > vecBefore , std::vector< double > vecAfter);
    double vectorDotProduct(std::vector< double > vecBefore , std::vector< double > vecAfter);


    double CalculateZAccuracy(std::vector< WORD > &vecDepthZ,
                                                 int nWidth, int nHeight,
                                                 double grandtrue,
                                                 double m_fDepthAccuracyDistanceMM,
                                                 std::vector< double > vecBefore, std::vector< double > vecAfter);
    //int mSignalMessageForAccuracy;

    //bool mIsFinishedForROI;
    //int mMessageCountForROI;
    //libeYs3D::video::Frame mFrameForROI;
    //libeYs3D::video::Frame mFrameTempForROI;
    //int64_t mCurrentTimeForROI;
    //int64_t mNewTimeForROI;
    //int mSignalControlForROI;
    //int mSignalMessageForROI;


};  // class FrameProducer

}  // namespace video
}  // namespace libeYs3D
