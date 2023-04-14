/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "sensors/SensorData.h"
#include "base/threads/Thread.h"
#include "base/threads/ThreadPool.h"
#include "base/synchronization/MessageChannel.h"

#include <stdint.h>
#include <vector>

namespace libeYs3D    {

namespace devices    {
    // forward declaration
    class IMUDevice;
}

namespace sensors    {

class CallbackWorkItem    {
public:
    std::function<bool(const SensorDataSet *sensorDataSet)> mCallback;
    SensorDataSet *mSensorDataSet;

public:
    CallbackWorkItem(std::function<bool(const SensorDataSet *sensorDataSet)> cb,
                     SensorDataSet *sensorDataSet)    {
        this->mCallback = cb;
        this->mSensorDataSet = sensorDataSet;
    }

    ~CallbackWorkItem()    {}
};

class SensorDataProducer : public base::Thread    {
public:
    // Callback to pass the contents of sensor data
    using Callback = std::function<bool(const SensorDataSet *sensorDataSet)>;
    using AppCallback = std::function<bool(const SensorData *sensorData)>;

    virtual ~SensorDataProducer()   {};

    intptr_t main() final;

    // Stops the producer
    virtual void stop();
    virtual const char* getName() = 0;
    
    // returns the actual number of bytes read and negative on error. 
    virtual int readSensorData(SensorData* sensorData) = 0;
    
    virtual void logProducerTick(const char *FMT, ...) = 0;
    
    // Attach the callbacks used to receiving new sensor data
    void attachAppCallback(AppCallback cb) { mAppCallback = std::move(cb); }
    void attachColorCallback(Callback cb) { mColorCallback = std::move(cb); }
    void attachDepthCallback(Callback cb) { mDepthCallback = std::move(cb); }

    // These are  synchronous API
    void enableAppCallback();
    void pauseAppCallback();
    void enableColorCallback();
    void pauseColorCallback();
    void enableDepthCallback();
    void pauseDepthCallback();
    
protected:
    void virtual attachReaderWorkerCGgroup() = 0;
    void virtual attachCallbackWorkerCGgroup() = 0;

protected:
    SensorDataProducer(SensorData::SensorDataType type);
    SensorDataProducer(SensorDataProducer&& p) = delete;

    bool colorCallbackWrapper(const SensorDataSet *sensorDataSet);
    bool depthCallbackWrapper(const SensorDataSet *sensorDataSet);

protected:
    AppCallback mAppCallback;
    Callback mColorCallbackWrapper;
    Callback mColorCallback;
    Callback mDepthCallbackWrapper;
    Callback mDepthCallback;

    SensorData::SensorDataType mSensorDataType;

private:
    void initialize();

    // Helper to send sensor data
    void sendSensorDataWorker();
    // Helper to call App sensor data callback
    void appCallbackWorker();

private:
    static constexpr int kMaxDataItems = 8;
    base::MessageChannel<SensorDataSet, kMaxDataItems> mDataQueue;
    base::MessageChannel<SensorDataSet, kMaxDataItems> mFreeQueue;
    base::MessageChannel<int, 3> mSignal;
    base::MessageChannel<int, 2> mCBFinishSignal; // for callback notification
    base::MessageChannel<int, 1> mPauseSignal;
    base::MessageChannel<int, 1> mPauseBackSignal;

    base::MessageChannel<SensorData, (kMaxDataItems << 3)> mAppDataQueue;
    base::MessageChannel<SensorData, (kMaxDataItems << 3)> mAppFreeDataQueue;

    bool mIsStopped = false;

    libeYs3D::base::ThreadPool<libeYs3D::sensors::CallbackWorkItem> mCBThreadPool;

public:
    friend class libeYs3D::devices::IMUDevice;
};

}  // namespace sensors
}  // namespace libeYs3D

