/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <stdint.h>                                   // for uint32_t, uint8_t
#include <memory>                                     // for unique_ptr

#include "devices/CameraDevice.h"
#include "PCProducer.h"
#include "video/Frame.h"
#include "video/PCFrame.h"
#include "video/FrameProducer.h"
#include "base/synchronization/Lock.h"
#include "base/synchronization/MessageChannel.h"

namespace libeYs3D    {
namespace video    {

using libeYs3D::devices::CameraDevice;

// Real implementation of Producer for point cloude
class PCFrameProducer : public PCProducer   {
public:
    friend std::unique_ptr<PCProducer> createPCFrameProducer(std::shared_ptr<CameraDevice> cameraDevice);
    virtual ~PCFrameProducer() {}

    intptr_t main() final;

    void pokeReceiver();
    void waitForPoke();
    
    void enableCallback() override;
    void pauseCallback() override;
    
    virtual void dumpFrameInfo(int frameCount = 60) override;
    void doSnapshot() override;
    
    virtual void logProducerTick(const char *FMT, ...) override;

    virtual void stop() override;
    
    virtual const char* getName() override    { return "PCFrameProducer"; }

protected:
    PCFrameProducer(std::shared_ptr<CameraDevice> cameraDevice);
    
    bool colorProducerCallback(const Frame* frame);
    bool depthProducerCallback(const Frame* frame);
    
protected:
    void virtual attachCoderCGgroup();
    void virtual attachCallbackWorkerCGgroup();
    
private:
    void initialize();

    // Helper to send frames at the user-specified FPS
    void sendFramesWorker();
    
    // Helper to perform snapshot in another working thread
    void performSnapshotWork(const         Frame * colorFrame, const Frame * depthFrame);

#ifdef WIN32	
	uint32_t mTimeDeltaMs;
    uint8_t  mFps;
	int64_t canUseTimeMs = 0;
	int color_drop_count = 0;
	int depth_drop_count = 0;
	std::vector<uint8_t> vGreen;
#endif    
    int producePCFrame(PCFrame *pcFrame);

    //++support point cloud for depth only mode
    int produceDepthOnlyPCFrame(PCFrame *pcFrame);
    //--support point cloud for depth only mode

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
    base::MessageChannel<PCFrame, kMaxFrames> mPCDataQueue;
    base::MessageChannel<PCFrame, kMaxFrames> mPCFreeQueue;
    base::MessageChannel<Frame, kMaxFrames> mColorFrameQueue;
    base::MessageChannel<Frame, kMaxFrames> mFreeColorFrameQueue;
    base::MessageChannel<Frame, kMaxFrames> mDepthFrameQueue;
    base::MessageChannel<Frame, kMaxFrames> mFreeDepthFrameQueue;
	base::MessageChannel<Frame, 1> mColorSnapQueue;
	base::MessageChannel<Frame, 1> mDepthSnapQueue;
    base::MessageChannel<int, 2> mSignal;
    base::MessageChannel<int, 1> mSnapshotSignal;
	base::MessageChannel<int, 1> mSnapshotBackSignal;
    base::MessageChannel<int, 1> mSnapshotFinishedSignal;
    bool mIsStopped = false;
    bool dosnapshot_flag = false;
    
    Frame *mColorFrame;
    Frame *mDepthFrame;
    uint32_t mExpectedFrameIndex = 0;
    Frame *color_temp;
    Frame *depth_temp;
    libeYs3D::video::Frame color_with_IR_Frame;
    libeYs3D::video::Frame depth_without_IR_Frame;
    int color_current_SN = 0;
    int depth_current_SN = 0;
    uint16_t IRvalue_original = 0;
    libeYs3D::devices::IRProperty property;
    bool IR_lock = false;


    
    Producer::Callback mColorProducerCallback;
    Producer::Callback mDepthProducerCallback;
    
    int mFrameDumpCount = 0;
    FILE *mFrameLogFile = nullptr;
    
    std::shared_ptr<CameraDevice> mCameraDevice;

    // for latency performance info
    uint64_t mStartTimeUs = 0llu;
    uint64_t mLastStartTimeUs = 0llu;
    uint64_t mTotalFrameCount = 0llu;
    uint64_t mTotalLatencyUs = 0llu;
    uint64_t mLastTotalLatencyUs = 0llu;
    uint64_t mTotalTranscodingTimeUs = 0llu;
    uint64_t mLastTotalTranscodingTimeUs = 0llu;
    uint64_t mTotalColorFrameLatencyUs = 0llu;
    uint64_t mLastTotalColorFrameLatencyUs = 0llu;
    uint64_t mTotalDepthFrameLatencyUs = 0llu;
    uint64_t mLastTotalDepthFrameLatencyUs = 0llu;
};  // class FrameProducer

}  // namespace video
}  // namespace libeYs3D
