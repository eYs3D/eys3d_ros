/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "CameraDevice.h"
#include "video/FrameProducer.h"
#include "video/PCProducer.h"
#include "video/FrameSet.h"
#include "base/synchronization/Lock.h"
#include "base/synchronization/ConditionVariable.h"
#include "utils.h"
#include "debug.h"
#include "macros.h"

#define FS_DEFAULT_TIMEOUT_MS   3200
#define FS_DEFAULT_TIMEOUT_US   3200000

namespace libeYs3D    {
namespace devices    {

class FrameSetPipeline    {
public:
    enum TYPE    {
        COLOR_FRAME_ONLY,
        DEPTH_FRAME_ONLY,
        ALL
    };

    enum RESULT    {
        SYNC_ERROR = -2,
        STOPPED = -1,
        OK = 0,
        TIMEOUT,
        QUEUE_EMPTY,
        QUEUE_FULL
    };

template <typename T, size_t CAPACITY>
class CircularQueue    {
public:
    RESULT enQueue(const T *item, int32_t timeoutMs = FS_DEFAULT_TIMEOUT_MS)    {
        libeYs3D::base::AutoLock lock(mLock);
        
        while(true)    {
            if(mStopped)    return RESULT::STOPPED;

            if((mRear != mFront) || (mCount == 0))    {
                mCount += 1;
                mRear = (mRear + 1) % mCapacity;
                mItems[mRear].clone(item);
            
                if(mCount == 1)    mCond.signal();
                                
                break;
            } else    { // queue full, ((mRear == mFront) && (mCount == mCapacity))
                if(timeoutMs == 0)    {
                    mFront = (mFront + 1) % mCapacity;
                    mRear = (mRear + 1) % mCapacity;
                    mItems[mRear].clone(item);

                    break;
                } else if(timeoutMs > 0)    {
#ifndef WIN32
                    if(false == mCond.timedWaitDebug(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          (timeoutMs * 1000)))
#else
                    if(false == mCond.timedWait(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          (timeoutMs * 1000)))
#endif
                        return RESULT::SYNC_ERROR;
                    else
                        if(mCount == mCapacity)    return RESULT::TIMEOUT;
                } else    { // timeoutMs < 0, wait forever
                    // instead of using mCond.wait(), checking if queue is stopped is required
#ifndef WIN32
                    if(false == mCond.timedWaitDebug(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          FS_DEFAULT_TIMEOUT_US))
#else
                    if(false == mCond.timedWait(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          FS_DEFAULT_TIMEOUT_US))
#endif
                        return RESULT::SYNC_ERROR;
                }
            }
        } // end of while(true)
        
        return RESULT::OK;
    }
    
    RESULT deQueue(T *item, uint32_t sn = UINT32_MAX, /* don't care */
                   int32_t timeoutMs = FS_DEFAULT_TIMEOUT_MS)    {
        libeYs3D::base::AutoLock lock(mLock);
        
        if(mStopped)    return RESULT::STOPPED;
        
        while(true)    {
            while(mCount == 0)    {
                if(mStopped)    return RESULT::STOPPED;
            
                if(timeoutMs == 0)    {
                    return RESULT::QUEUE_EMPTY;
                } else if(timeoutMs > 0)    {
#ifndef WIN32
                    if(false == mCond.timedWaitDebug(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          (timeoutMs * 1000)))
#else
                    if(false == mCond.timedWait(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          (timeoutMs * 1000)))
#endif
                        return RESULT::SYNC_ERROR;

                    if(mCount == 0)    return RESULT::TIMEOUT;
                } else    { // wait forever
                    // instead of using mCond.wait() to check if the queue is stopped
#ifndef WIN32
                    if(false == mCond.timedWaitDebug(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          (FS_DEFAULT_TIMEOUT_MS * 1000)))
#else
                    if(false == mCond.timedWait(&mLock,
                                          now_in_microsecond_high_res_time_REALTIME() +
                                          (FS_DEFAULT_TIMEOUT_MS * 1000)))
#endif
                        return RESULT::SYNC_ERROR;
                }
            }
        
            if(mCount == mCapacity)    mCond.signal();
            
            mCount -= 1;
            mFront = (mFront + 1) % mCapacity;
            
            if(sn != UINT32_MAX)    {
                if(mItems[mFront].serialNumber == sn)    {
                    break;
                } else if(mItems[mFront].serialNumber < sn)    {
                    if(IS_SN_ROTATE(mItems[mFront].serialNumber, sn))    break;
                    
                    continue;
                } else    { // (mItems[mFront].serialNumber > sn)
                    if(IS_SN_ROTATE(mItems[mFront].serialNumber, sn))    continue;

                    break;
                }
            }
            
            break;
        } // end of while(true)
        
        item->clone(&mItems[mFront]);
        
        return RESULT::OK;
    }
    
    void reset()    {
        libeYs3D::base::AutoLock lock(mLock);
        
        mFront = 0;
        mRear = 0;
        mCount = 0;
    }
    
    void stop()    {
        libeYs3D::base::AutoLock lock(mLock);
        
        if(mStopped)    return;
        
        mStopped = true;
        mCond.broadcast();
    }
    
    CircularQueue()    {}

    ~CircularQueue()    { stop(); }
    
private:
    T mItems[CAPACITY];
    libeYs3D::base::Lock mLock;
    libeYs3D::base::ConditionVariable mCond;
    size_t mFront = 0;
    size_t mRear = 0;
    size_t mCount = 0;
    size_t mCapacity = CAPACITY;
    
    bool mStopped = false;
};

public:
    /**
     * Retrieves and removes the head of this queue, or returns null if this queue is empty.
     * \return
     *     STOPPED: The pipeline has been stopped
     *     OK:  OK
     *     QUEUE_EMPTY: The pipeline is empty
     */
    RESULT pollFrameSet(libeYs3D::video::FrameSet *frameSet);

    /**
     * Retrieves and removes the head of this queue if available.
     * If not avaiable, it waits until timeoutMs elapse
     * \param[in] timeoutMs   Max time in milliseconds to wait
     *     timeoutMs == 0: don't wait, == pull ....
     *     timeoutMs < 0: wait forever until a queue slot is available
     * \return
     *     STOPPED: The pipeline has been stopped
     *     OK:  OK
     *     TIMEOUT: Timeout of timeoutMs exceeded
     */
    RESULT waitForFrameSet(libeYs3D::video::FrameSet *frameSet,
                           int32_t timeoutMs = FS_DEFAULT_TIMEOUT_MS);

    void clear();

    virtual ~FrameSetPipeline();

private:
    explicit FrameSetPipeline(CameraDevice *cameraDevice, TYPE type = TYPE::ALL);
    
    void stop();

    bool colorImageCallback(const libeYs3D::video::Frame* frame);
    bool depthImageCallback(const libeYs3D::video::Frame* frame);
    bool pcFrameCallback(const libeYs3D::video::PCFrame *pcFrame);

private:
    CameraDevice *mCameraDevice;
    
    TYPE mPipelineType;
    bool mStopped = false;
    
    libeYs3D::video::Producer::Callback mColorImageCallback;
    libeYs3D::video::Producer::Callback mDepthImageCallback;
    libeYs3D::video::PCProducer::PCCallback mPCFrameCallback;
    
    static constexpr int kMaxFrameCount = 2; // 1 seconds @ 60FPS
    CircularQueue<libeYs3D::video::Frame, kMaxFrameCount> mColorFrameQueue;
    CircularQueue<libeYs3D::video::Frame, kMaxFrameCount> mDepthFrameQueue;
    CircularQueue<libeYs3D::video::PCFrame, (kMaxFrameCount >> 1)> mPCFrameQueue;
    
    uint32_t mCurrentSN = 0;

public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
