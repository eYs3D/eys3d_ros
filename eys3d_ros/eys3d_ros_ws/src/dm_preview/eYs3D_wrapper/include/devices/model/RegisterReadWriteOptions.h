/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once
#ifdef WIN32
#  include <magic.h>
#  include <eSPDI_Common.h>
#else
#  include <eSPDI_def.h>
#endif

#include <inttypes.h>
#include <cstdio>

#define REGISTER_REQUEST_MAX_COUNT 8

namespace libeYs3D    {
namespace devices    {

// forward declaration
class CameraDevice;

class RegisterReadWriteOptions    {
public:
    enum TYPE    {
        I2C,
        ASIC,
        FW,
        TYPE_NONE
    };  

public:
    TYPE getType()    { return mRegisterType; }
    void setType(TYPE type)    { mRegisterType = type; }

    int getSlaveID()    { return mSlaveID; }
    void setSlaveID(int id)    { mSlaveID = id; }

    uint16_t getAddressSize()    { return mAddressSize; }
    void setAddressSize(unsigned short nSize)    { mAddressSize = nSize; }

    uint16_t getValueSize()    { return mValueSize; }
    void setValueSize(unsigned short nSize)    { mValueSize = nSize; }

    SENSORMODE_INFO getSensorMode()    { return mSensorMode; }
    void setSensorMode(SENSORMODE_INFO sensorMode)    { mSensorMode = sensorMode; }

    int getRequestAddress(int nIndex)    { return mRequestAddress[nIndex]; }
    void setRequestAddress(int nIndex, int nAddress)    { mRequestAddress[nIndex] = nAddress; }

    int getRequestValue(int nIndex)    { return mRequestValue[nIndex]; }
    void setRequestValue(int nIndex, int nValue)    { mRequestValue[nIndex] = nValue; }

    bool isPeriodicRead()    { return mPeriodicRead; }
    void enablePeriodicRead(bool bEnable)    { mPeriodicRead = bEnable; }

    int getPeriodTimeMs()    { return mPeriodTimeMs; }
    void setPeriodTimeMs(int nMs)    { mPeriodTimeMs = nMs;}

    bool isSaveLog()    { return mSaveLog; }
    void enableSaveLog(bool bEnable)    { mSaveLog = bEnable; }
    
    int toString(char *buffer, int bufferLength) const;
    
    bool operator==(const RegisterReadWriteOptions &rhs) const;
    bool operator>=(const RegisterReadWriteOptions &rhs) const;
    void operator<<(const RegisterReadWriteOptions &rhs);

protected:
    RegisterReadWriteOptions();

private:
    TYPE mRegisterType = I2C;
    int mSlaveID = EOF;
    uint16_t mAddressSize = FG_Address_1Byte;
    uint16_t mValueSize = FG_Value_1Byte;
    SENSORMODE_INFO mSensorMode = SENSOR_A;
    int32_t mRequestAddress[REGISTER_REQUEST_MAX_COUNT];
    int32_t mRequestValue[REGISTER_REQUEST_MAX_COUNT];

    bool mPeriodicRead = false;
    int32_t  mPeriodTimeMs = 1;
    bool mSaveLog = false;
    
public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
