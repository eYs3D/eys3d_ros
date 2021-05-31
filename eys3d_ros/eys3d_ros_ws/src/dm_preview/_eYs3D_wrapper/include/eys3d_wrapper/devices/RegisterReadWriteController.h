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

#include "base/threads/ThreadPool.h"
#include "base/synchronization/MessageChannel.h"
#include "devices/model/RegisterReadWriteOptions.h"

namespace libeYs3D    {
namespace devices    {

// forward declaration
class CameraDevice;

class RegisterReadWriteController : public libeYs3D::base::Thread    {
public:
    virtual ~RegisterReadWriteController()   {};

    void commitReadRegisters();
    void commitWriteRegisters();

    intptr_t main() final;
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
    libeYs3D::base::MessageChannel<int, 1> mPauseSignal;
    libeYs3D::base::MessageChannel<int, 1> mPauseBackSignal;
    libeYs3D::base::MessageChannel<int, 1> mResumeSignal;
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
        EXIT
    };

public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
