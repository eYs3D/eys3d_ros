/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#ifdef WIN32
#  include "eSPDI_Common.h"
#  include "magic.h"
#include <ks.h>
#include <KsMedia.h>	//for PropertyPage item R/W
#else
#  include "eSPDI_def.h"
#  include <linux/limits.h>
#endif

#include <inttypes.h>
#include <string>

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
    int toPrettyString(char *buffer, int bufferLength) const;
    
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
