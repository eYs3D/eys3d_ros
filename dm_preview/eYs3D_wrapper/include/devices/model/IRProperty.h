/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#ifdef WIN32
#  include <eSPDI_Common.h>
#else
#  include <eSPDI_def.h>
#endif

#include <inttypes.h>

#define DEFAULT_IR_MAX    6
#define EXTEND_IR_MAX     15

namespace libeYs3D    {
namespace devices    {

// forward declaration
class CameraDevice;

class IRProperty    {
public:
    ~IRProperty()    {} 
    
    void enableExtendIR(bool enabled)    { mExtendIREnabled = enabled; }
    bool isExtendIREnabled()    { return mExtendIREnabled; }
    void enableExtendIRSupport(bool enabled)   { mExtendIRSupport = enabled; }
    bool isExtendIRSupport()    { return mExtendIRSupport; }
    
    void setIRValue(uint16_t value)    { mIRValue = value; }
    uint16_t getIRValue()    { return mIRValue; }
    
    uint16_t getIRMax()    { return mIRMax; }
    uint16_t getIRMin()    { return mIRMin; }

    int toString(char *buffer, int bufferLength) const;
    
    bool operator==(const IRProperty &rhs) const;

//protected:
    IRProperty(uint16_t initValue = (1 << 16 - 1), bool extendIR = false);

private:
    bool mExtendIREnabled;
    bool mExtendIRSupport;
    uint16_t mIRMax;
    uint16_t mIRMin;
    uint16_t mIRValue;

public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
