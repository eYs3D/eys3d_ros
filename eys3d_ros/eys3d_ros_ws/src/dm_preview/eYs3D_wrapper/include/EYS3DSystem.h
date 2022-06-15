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

    int getCameraDeviceCount()    { return mDeviceCount; }
    std::shared_ptr<CameraDevice> getCameraDevice(int index);
    static void *getEYS3DDIHandle();
	int getDeviceCount();

    static uint8_t generateModelID();
    
    void dumpSystemInfo();

    static char *getSDKHomePath();
    static char *getHomePath();
    static char *getLogPath();
    static char *getFramePath();
    static char *getVideoRecordingPath();
    static char *getSnapshotPath();
    static char *getIMULogPath();
    static char *getReadRegisterLogsPath();
    
    static int getFPSWindowSize();

    explicit EYS3DSystem();
    explicit EYS3DSystem(COLOR_BYTE_ORDER colorByteOrder);
	
    virtual ~EYS3DSystem();

	static void initHID();
	static void clearHID();
	static std::vector<std::string> getHIDDeviceList(uint16_t nVID, uint16_t nPID);

	int initialize();

private:

    int createEYS3DHome();

private:

    int mDeviceCount;
    std::map<DeviceSellectInfo, std::shared_ptr<CameraDevice>>mDeviceMap;

};

} // end of namespace libeYs3D
