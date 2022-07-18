/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <map>
#include <vector>
#include <string>
#include <limits.h>
#include <memory>
#ifdef WIN32
#  include "eSPDI_Common.h"
#  include <magic.h>
#else
#  include "eSPDI.h"
#endif
#include "hidapi/hidapi.h"

namespace libeYs3D {

namespace devices    { // forward declararion
    struct CameraDeviceInfo;
    class CameraDevice;
}

using ::libeYs3D::devices::CameraDeviceInfo;
using ::libeYs3D::devices::CameraDevice;

struct DeviceSellectInfo    {
    DEVSELINFO devSelInfo;

    DeviceSellectInfo(DEVSELINFO *info)    {
        this->devSelInfo = *info;
    }

    DeviceSellectInfo(int index)    { devSelInfo.index = index; }

    DeviceSellectInfo()    {}

    bool operator==(const DeviceSellectInfo &rhs) const {
        return this->devSelInfo.index == rhs.devSelInfo.index;
    }

    bool operator<(const DeviceSellectInfo &rhs)  const {
        return this->devSelInfo.index < rhs.devSelInfo.index;
    }

    int toString(char *buffer, int bufferLength) const;
    int toString(std::string) const;
};

class EYS3DSystem    {
public:
    enum class COLOR_BYTE_ORDER {
        COLOR_RGB24,
        COLOR_BGR24
    };
    COLOR_BYTE_ORDER mColorByteOrder;
    static std::shared_ptr<EYS3DSystem> getEYS3DSystem();
    static std::shared_ptr<EYS3DSystem> getEYS3DSystemRGBOrder(COLOR_BYTE_ORDER colorByteOrder);

    int getCameraDeviceCount()    { return mDeviceCount; }
    std::shared_ptr<CameraDevice> getCameraDevice(int index);
    void *getEYS3DDIHandle();
	int getDeviceCount();

    std::vector<std::string> getHIDDeviceList(uint16_t nVID, uint16_t nPID);
    uint8_t generateModelID();
    
    void dumpSystemInfo();

    const char *getSDKHomePath();
    const char *getHomePath() const   { return mHomePath; }
    const char *getLogPath() const   { return mLogPath; }
    const char *getFramePath() const    { return mFramePath; }
    const char *getVideoRecordingPath() const    { return mVideoRecordingPath; }
    const char *getSnapshotPath() const    { return mSnapshotPath; }
    const char *getIMULogPath() const    { return mIMULogPath; }
    const char *getReadRegisterLogsPath() const {return mReadRegisterLogsPath;}
    
    int getFPSWindowSize()    { return mFPSWindowSize; }
    
    virtual ~EYS3DSystem();

private:
    explicit EYS3DSystem();
    explicit EYS3DSystem(COLOR_BYTE_ORDER colorByteOrder);

    int initialize();
    int createEYS3DHome();

    void initHID();
    void clearHID();

private:
    static std::shared_ptr<EYS3DSystem> sEYS3DSystem;

    void *mEYS3DDIHandle;
    int mDeviceCount;
    std::map<DeviceSellectInfo, std::shared_ptr<CameraDevice>>mDeviceMap;

    std::map<std::pair<uint16_t, uint16_t>, std::vector<std::string>> mHidDeviceMap;
    
    char mSDKHomePath[PATH_MAX];
    char mHomePath[PATH_MAX];
    char mLogPath[PATH_MAX];
    char mFramePath[PATH_MAX];
    char mVideoRecordingPath[PATH_MAX];
    char mSnapshotPath[PATH_MAX];
    char mIMULogPath[PATH_MAX];
    char mReadRegisterLogsPath[PATH_MAX];
    
    int mFPSWindowSize;

};

} // end of namespace libeYs3D
