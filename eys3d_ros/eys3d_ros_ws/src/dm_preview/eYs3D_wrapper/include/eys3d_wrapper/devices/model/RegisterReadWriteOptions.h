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

#include <eSPDI_def.h>

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
        IC2,
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
    TYPE mRegisterType = IC2;
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
