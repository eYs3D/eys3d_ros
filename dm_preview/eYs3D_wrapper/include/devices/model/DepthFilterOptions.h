/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <stdint.h>
#include "string"

namespace libeYs3D    {
namespace devices    {

// forward declaration
class CameraDevice;

class DepthFilterOptions    {
public:
    ~DepthFilterOptions() = default;

    void resetDefault();
    
    void enable(bool enable);
    bool isEnabled()    { return mEnabled; }

    //Common
protected:
    void setBytesPerPixel(int32_t bytesPerPixel);
public:
    int32_t getBytesPerPixel(){ return mBytesPerPixel; }

    //SubSample
    void enableSubSample(bool enable);
    bool isSubSampleEnabled()    { return mSubSampleEnabled; }
    
    void setSubSampleMode(int32_t mode);
    int32_t getSubSampleMode()    { return mSubSampleMode; }

    void setSubSampleFactor(int32_t subSampleFactor);
    int32_t getSubSampleFactor()    { return mSubSampleFactor; }

    //EdgePreServingFilter
    void enableEdgePreServingFilter(bool enable);
    bool isEdgePreServingFilterEnabled()    { return mEdgePreServingFilterEnabled; }

    void setType(int type);
    int32_t getType()    { return mType; }

    void setEdgeLevel(int edgeLevel);
    int32_t getEdgeLevel()    { return mEdgeLevel;}

    void setSigma(float sigma);
    float getSigma()    { return mSigma; }

    void setLumda(float lumda);
    float getLumda()    { return mLumda; }

    //HoleFill
    void enableHoleFill(bool enable);
    bool isHoleFillEnabled()    { return mHoleFillEnabled; }

    void setKernelSize(int32_t kernelSize);
    int32_t getKernelSize()    { return mKernelSize; }

    void setLevel(int32_t level);
    int32_t getLevel()    { return mLevel; }

    void setHorizontal(bool horizontal);
    bool isHorizontal()    { return mHorizontal; }

    //TemporalFilter
    void enableTemporalFilter(bool enable);
    bool isTemporalFilterEnabled()    { return mTemporalFilterEnabled; }

    void setAlpha(float alpha);
    float getAlpha()    { return mAlpha; }

    void setHistory(int32_t history);
    int32_t getHistory()    { return mHistory; }
    
    ////////
    void enableFlyingDepthCancellation(bool enable);
    bool isFlyingDepthCancellationEnabled()    { return mFlyingDepthCancellationEnabled; }
    void setFlyingDepthCancellationLocked(bool lock);
    bool isFlyingDepthCancellationLocked()    { return mFlyingDepthCancellationLocked; }
    
    int toString(char *buffer, int bufferLength) const;
    int toString(std::string &string) const;
    
protected:
    

protected:
    DepthFilterOptions();

    bool mEnabled;

    //Common
    int32_t mBytesPerPixel;

    //SubSample
    bool mSubSampleEnabled;
    int32_t mSubSampleMode;
    int32_t mSubSampleFactor;

    //EdgePreServingFilter
    bool mEdgePreServingFilterEnabled;
    int32_t mType;
    int32_t mEdgeLevel;
    float mSigma;
    float mLumda;

    //HoleFill
    bool mHoleFillEnabled;
    int32_t mKernelSize;
    int32_t mLevel;
    bool mHorizontal;

    //TemporalFilter
    bool mTemporalFilterEnabled;
    float mAlpha;
    int32_t mHistory;
    
    //
    bool mFlyingDepthCancellationEnabled;
    bool mFlyingDepthCancellationLocked;
    
public:
    friend class CameraDevice;
};

} // end of namespace devices
} // end of namespace libeYs3D
