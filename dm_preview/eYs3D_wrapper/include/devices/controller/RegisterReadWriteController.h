/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "base/threads/ThreadPool.h"
#include "base/synchronization/MessageChannel.h"
#include "devices/model/RegisterReadWriteOptions.h"

namespace libeYs3D    {
namespace devices    {

// forward declaration
class CameraDevice;

class RegisterReadWriteController : public libeYs3D::base::Thread    {
public:
    virtual ~RegisterReadWriteController();

    void commitReadRegisters();
    void commitWriteRegisters();

    intptr_t main() final;
    
    // These are synchronous API
    void stop();
    void pause();
    void resume();

protected:
    RegisterReadWriteController(CameraDevice *cameraDevice);

    CameraDevice *mCameraDevice;

private:
    void commandExecutor();

    void readRegisters();
    void writeRegisters();
    void logRegisters();

    int64_t mLastestReadTimeMs;

    bool mIsStopped;
    libeYs3D::base::MessageChannel<int, 2> mSignal;
    libeYs3D::base::MessageChannel<int, 1> mCommandSignal;
    libeYs3D::base::MessageChannel<int, 1> mCommandBackSignal;
    libeYs3D::base::MessageChannel<int, 3> mJobSignal;
    libeYs3D::base::MessageChannel<int, 1> mReadCommitSignal;
    libeYs3D::base::MessageChannel<int, 1> mWriteCommitSignal;

    enum COMMAND    {
        FORCE_READ_REGISTER,
        READ_REGISTER,
        FORCE_WRITE_REGISTER,
        WRITE_REGISTER,
        LOG_REGISTER,
        PAUSE,
        RESUME,
        EXIT
    };

public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
