/*
 * Copyright (C) 2015-2019 ICL/ITRI
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

#include <map>
#include <vector>
#include <string>
#include <limits.h>
#include <memory>

#include "eSPDI.h"
#include "DMPreview_utility/hidapi/hidapi.h"

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
    static std::shared_ptr<EYS3DSystem> getEYS3DSystem();

    int getCameraDeviceCount()    { return mDeviceCount; }
    std::shared_ptr<CameraDevice> getCameraDevice(int index);
    void *getEYS3DDIHandle();
    
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
    
    virtual ~EYS3DSystem();

private:
    explicit EYS3DSystem();

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
};

} // end of namespace libeYs3D
