/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <stdint.h>                                   // for uint32_t, uint8_t
#include <memory>                                     // for unique_ptr

#include "video/Producer.h"
#include "base/synchronization/MessageChannel.h"
#include "sensors/SensorDataProducer.h"
#include "base/threads/ThreadPool.h"

#define FP_INITIALIZED            0x0001
#define FB_APP_STREAM_ACTIVATED   0x0100
#define FB_PC_STREAM_ACTIVATED    0x0200

namespace libeYs3D    {

namespace devices    {
    // forward declaration
    class CameraDevice;
}

namespace video    {

class CallbackWorkItem    {
public:
    //std::function<void(const Frame *)> callback;
    Producer::Callback callback;
    Frame *frame;
    
public:
    //CallbackWorkItem(std::function<void(const Frame *)> cb, Frame *frame)    {
    CallbackWorkItem(Producer::Callback cb, Frame *frame)    {
        this->callback = cb;
        this->frame = frame;
    }
    
    ~CallbackWorkItem()    {}
};

// Real implementation of Producer for video data
// There are two threads at work: one is grabbing the frames at a user-specified
// fps and another thread is sending each frame received to the callback. We
// separate the frame fetching and encoding because sometimes, the encoder will
// need extra time (usually frames marked as key frames). By separating the
// fetching and encoding and also using multiple buffers we can feed the encoder
// a relatively constant rate of frames.
class FrameProducer : public Producer    {
public:
    virtual ~FrameProducer() {}

    intptr_t main() final;

    void pokeReceiver();
    void waitForPoke();

    virtual void stop() override;
    
    virtual const char* getName() override    { return "FrameProducer"; }
    
    // Attach the point cloud callback used to receiving new Frames
    virtual void attachPCCallback(Producer::Callback cb) { mPCCallback = std::move(cb); }
    
    // These are synchronous API
    virtual void enableCallback();
    virtual void pauseCallback();
    virtual void enablePCCallback();
    virtual void pausePCCallback();
    
    void dumpFrameInfo(int frameCount = 60);
    void doSnapshot(int StreamType);
    bool doColorSnapShot = false;
    bool doDepthSnapShot = false;

protected:
    FrameProducer(libeYs3D::devices::CameraDevice *cameraDevice,
                  int32_t fWidth, int32_t fHeight, int32_t fFormat, uint8_t fps);

    virtual int getRawFormatBytesPerPixel(uint32_t format) = 0;
    virtual int readFrame(Frame *frame) = 0;
    virtual int produceRGBFrame(Frame *frame) = 0;
    virtual int performPostProcessFilter(Frame *frame) = 0;

    /**
     * This value is set after Camera::initStream by decimation factor.
     * It will keep the same as depth width if PostProcessOption !isEnable, and will be resized width if decimation
     * factor is set >= 2.
     * @return Decimation filter resized width if PostProcessOption::isEnable().
     */
    virtual int getFilteredWidth() = 0;
    /**
     * This value is set after Camera::initStream by decimation factor.
     * It will keep the same as depth height if PostProcessOption !isEnable, and will be resized width if decimation
     * factor is set >= 2.
     * @return Decimation filter resized height if PostProcessOption::isEnable().
     */
    virtual int getFilteredHeight() = 0;

    virtual int performFiltering(Frame *frame) = 0;
    virtual int performInterleave(Frame *frame) = 0;
    virtual int performAccuracyComputation(Frame *frame) = 0;
    virtual int performROIComputation(Frame *frame) = 0;

    bool callbackWrapper(const Frame *frame);
    bool pcCallbackWrapper(const Frame *frame);
    
    bool imuDataSetCallback(const libeYs3D::sensors::SensorDataSet *sensorDataSet);
    virtual void checkIMUDeviceCBEnablement() = 0;
    
    virtual void logProducerTick(const char *FMT, ...) = 0;

protected:
    void virtual attachReaderWorkerCGgroup() = 0;
    void virtual attachRGBWorkerCGgroup() = 0;
    void virtual attachFilterWorkerCGgroup() = 0;
    void virtual attachCallbackWorkerCGgroup() = 0;

protected:
    Producer::Callback mCallbackWrapper;
    Producer::Callback mPCCallbackWrapper;
    Producer::Callback mPCCallback;
    
    libeYs3D::devices::CameraDevice *mCameraDevice;
    int32_t mFrameWidth;
    int32_t mFrameHeight;

    base::MessageChannel<int, 1> mSnapshotFinishedSignal;
    
    libeYs3D::sensors::SensorDataProducer::Callback mIMUDataCallback;
    
    uint32_t mFrameProducerState;
    bool mIsStopped = false;

private:
    void initialize();

    // Helper to send frames at the user-specified FPS
    void sendFramesWorker();
    // Helper to produce RGB frames
    void rgbFramesWorker();
    // Helper to perfrom filetering
    void frameFilteringWorker();
  
    // Heper to perfrom snapshot
    virtual void performSnapshotWork(Frame *frame) = 0;

    uint32_t mTimeDeltaMs;
    uint8_t  mFps;
    uint8_t  mTimeLimitSecs;

    // mDataQueue contains filled video frames and mFreeQueue has available
    // video frames that the producer can use. The workflow is as follows:
    // Producer:
    //   1) get an video frame from mFreeQueue
    //   2) write data into frame
    //   3) push frame to mDataQueue
    // Consumer:
    //   1) wait for video frame in mDataQueue
    //   2) process video frame
    //   3) Push frame back to mFreeQueue
    static constexpr int kMaxFrames = 2;
    base::MessageChannel<Frame, kMaxFrames> mDataQueue;
    base::MessageChannel<Frame, kMaxFrames> mStageQueue;
    base::MessageChannel<Frame, kMaxFrames> mStage2Queue;
    base::MessageChannel<Frame, kMaxFrames> mFreeQueue;
	base::MessageChannel<Frame, 1> mSnapQueue;
    base::MessageChannel<int, 4> mSignal;
    base::MessageChannel<int, 1> mCBFinishSignal; // for callback notification
	base::MessageChannel<int, 1> mPCCBFinishSignal;
    base::MessageChannel<int, 1> mPauseSignal;
    base::MessageChannel<int, 1> mPauseBackSignal;
    base::MessageChannel<int, 1> mSnapshotSignal;
    
    base::MessageChannel<libeYs3D::sensors::SensorDataSet, kMaxFrames> mSensorDataSetQueue;
    base::MessageChannel<libeYs3D::sensors::SensorDataSet, kMaxFrames> mFreeSensorDataSetQueue;
    

    
    libeYs3D::base::ThreadPool<libeYs3D::video::CallbackWorkItem> mCBThreadPool;
    libeYs3D::base::ThreadPool<libeYs3D::video::CallbackWorkItem> mPCCBThreadPool;
    
private:
    int mFrameDumpCount = 0;
    FILE *mFrameLogFile = nullptr;
    
    // for latency performance info
    uint64_t mStartTimeUs = 0llu;
    uint64_t mLastStartTimeUs = 0llu;
    uint64_t mTotalFrameCount = 0llu;
    uint64_t mTotalLatencyUs = 0llu;
    uint64_t mLastTotalLatencyUs = 0llu;
    uint64_t mTotalRGBTranscodingTimeUs = 0llu;
    uint64_t mLastTotalRGBTranscodingTimeUs = 0llu;
    uint64_t mTotalFilteringTimeUs = 0llu;
    uint64_t mLastTotalFilteringTimeUs = 0llu;
    
public:
    friend class libeYs3D::devices::CameraDevice;
};  // class FrameProducer

}  // namespace video
}  // namespace libeYs3D
