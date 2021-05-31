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

#include "eSPDI_def.h"

#include <inttypes.h>
#include <string>
#include <linux/limits.h>

namespace libeYs3D    {
namespace devices    {

// forward declaration
class CameraDevice;

class CameraDeviceProperties    {
public:
    struct CameraPropertyItem    {
        bool bSupport = true;
        bool bValid = false;
        int32_t nValue;
        int32_t nMin;
        int32_t nMax;
        int32_t nDefault;
        int32_t nFlags;
        int32_t nStep;
    };

    enum CAMERA_PROPERTY    {
        AUTO_EXPOSURE = 0,
        AUTO_WHITE_BLANCE,
        LOW_LIGHT_COMPENSATION,
        LIGHT_SOURCE,
        EXPOSURE_TIME, // unit 100 microsecond
        WHITE_BLANCE_TEMPERATURE,
        CAMERA_PROPERTY_COUNT
    };

    enum LIGHT_SOURCE_VALUE    {
        VALUE_50HZ,
        VALUE_60HZ
    };

public:
    ~CameraDeviceProperties();

    int init();
    int update();
    int reset();

    int initCameraProperties();
    int updateCameraProperty(CAMERA_PROPERTY type);
    int setDefaultCameraProperties();
    int setCameraPropertyValue(CAMERA_PROPERTY type, int32_t nValue);
    int setCameraPropertySupport(CAMERA_PROPERTY type, bool bSupport);

    float getManuelExposureTimeMs();
    void setManuelExposureTimeMs(float fMs);
    float getManuelGlobalGain();
    void setManuelGlobalGain(float fGlobalGain);

    void setDeviceName(const char *deviceName);
    std::string getDeviceName();
    CameraPropertyItem getCameraProperty(CAMERA_PROPERTY type);
    
    int toString(char *buffer, int bufferLength) const;
    int toString(std::string &string) const;
    
protected:
    CameraDeviceProperties(CameraDevice *cameraDevice);
    
private:
    void getCameraPropertyFlag(CAMERA_PROPERTY type, int32_t *nID, bool *bIsCTProperty);
    void dataToInfo(CAMERA_PROPERTY type, int32_t *nValue);
    void infoToData(CAMERA_PROPERTY type, int32_t *nValue);

private:
    char mDeviceName[PATH_MAX];
    CameraPropertyItem mCameraPropertyItems[CAMERA_PROPERTY_COUNT];
    CameraDevice *mCameraDevice;
    
public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
