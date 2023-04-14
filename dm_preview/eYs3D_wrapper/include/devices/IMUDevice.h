/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "sensors/SensorDataProducer.h"
#include "sensors/SensorData.h"
#include "hidapi/hidapi.h"

#include <stdio.h>
#include <stdint.h>
#include <vector>

#define IMUD_INITIALIZED            0x0001
#define IMUD_STREAM_INITIALIZED     0x0002
#define IMUD_COLOR_STREAM_CB        0x0010
#define IMUD_DEPTH_STREAM_CB        0x0020
#define IMUD_APP_STREAM_CB          0x0040
#define IMUD_COLOR_STREAM_ACTIVATED 0x0100
#define IMUD_DEPTH_STREAM_ACTIVATED 0x0200
#define IMUD_APP_STREAM_ACTIVATED   0x0400

#define IMUD_IS_CAPTURING_DATA      0x1000

namespace libeYs3D    {

namespace sensors    {
    // forward declaration
    class IMUDataProducer;
}

namespace devices    {

// forward declaration
class CameraDevice;
class CameraDevice8062;

class IMUDevice    {
public:
    enum IMU_TYPE    {
        IMU_6_AXIS,
        IMU_9_AXIS,
        IMU_UNKNOWN
    };

    enum IMU_DATA_FORMAT    {
        RAW_DATA_WITHOUT_OFFSET = 1,
        RAW_DATA_WITH_OFFSET,
        OFFSET_DATA,
        DMP_DATA_WITHOT_OFFSET,
        DMP_DATA_WITH_OFFSET,
        
        QUATERNION_DATA // CameraDevice 8062
    };

    typedef struct IMUDeviceInfo {
        uint16_t nVID;
        uint16_t nPID;
        IMU_TYPE nType;
        
        char serialNumber[256];
        char fwVersion[256];
        char moduleName[256];
        char imuLogPath[4096];
        bool status;
        bool isValid;
        
        int toString(char *buffer, int bufferLength);
    } IMUDeviceInfo;

    // returns the actual number of bytes read and negative on error. 
    virtual int readIMUData(IMUData *imuData);
 
    virtual void checkCalibratingStatus(char *calibratingStatus);
    virtual void startCalibration();
    virtual void readCalibrated(char *calibrated);

    IMUDeviceInfo getIMUDeviceInfo();
    virtual const char *getFWVersion();
    virtual const char *getModuleName();
    virtual int getStatus();
    virtual IMU_TYPE getType();
    virtual const char *getSerialNumber();
    virtual void setSerialNumber(const char *sn);
    
    // common API similar to CameraDevice
    virtual int initStream(libeYs3D::sensors::SensorDataProducer::AppCallback appCallback,
                           libeYs3D::sensors::SensorDataProducer::Callback colorFrameProducer,
                           libeYs3D::sensors::SensorDataProducer::Callback depthFrameProducer);

    bool isStreamEnabled();
    void enableStream();
    void enableAppStream();
    void enableColorStream();
    void enableDepthStream();
    void pauseStream();
    void pauseAppStream();
    void pauseColorStream();
    void pauseDepthStream();
    
    virtual void dumpIMUData(int recordCount = 256);
    char* getIMULogPath() {return mIMUDeviceInfo.imuLogPath;}
                                
    virtual void closeStream();

    virtual const char* getName() { return "Default IMU Device"; }
    int toString(char *buffer, int bufferLength);

    virtual ~IMUDevice();

protected:
    IMUDevice(IMUDeviceInfo info, CameraDevice *cameraDevice);
    
    virtual bool isValid();
    virtual void readDataOutputFormat();
    virtual int getIMUDataOutputBytes(IMU_DATA_FORMAT format);
    virtual std::vector<IMU_DATA_FORMAT> getSupportDataFormat();
    virtual void enableDataOutout(bool bIsEnbale);
    virtual int selectDataFormat(IMU_DATA_FORMAT format);
    virtual IMU_DATA_FORMAT getDataFormat();

private:
    int initialize();
    int postInitialize();
    
    void getFeatureReport(char *pData, size_t data_lenght);
    void sendFeatureReport(const char *pData, size_t data_lenght);
    
    bool isIMUConnectedWithCamera(hid_device *device);
    
    void logIMUDataRecord(uint8_t *imuRawData, int length);

protected:
    IMUDeviceInfo mIMUDeviceInfo;
    CameraDevice *mCameraDevice;
    
    hid_device *mHidHandle;
    IMU_DATA_FORMAT mCurrentIMUDataFormat;

    uint32_t mIMUDeviceState;

public:
    friend class CameraDevice;
    friend class CameraDevice8062;

private:
    libeYs3D::sensors::IMUDataProducer *mIMUDataProducer;
    int mDataDumpCount;
    FILE *mDataLogFile;
    
    struct FeatureConfigItem    {
        const char *pData;
        int nDataLength;
    };
};

} // end of namespace devices
} // end of namespace libeYs3D
