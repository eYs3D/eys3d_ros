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
    
    void setIRValue(uint16_t value)    { mIRValue = value; }
    uint16_t getIRValue()    { return mIRValue; }
    
    uint16_t getIRMax()    { return mIRMax; }
    uint16_t getIRMin()    { return mIRMin; }

    int toString(char *buffer, int bufferLength) const;
    
    bool operator==(const IRProperty &rhs) const;

protected:
    IRProperty(uint16_t initValue = (1 << 16 - 1), bool extendIR = false);

private:
    bool mExtendIREnabled;
    uint16_t mIRMax;
    uint16_t mIRMin;
    uint16_t mIRValue;

public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
