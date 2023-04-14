/*
 * Copyright (C) 2022 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#ifndef EYS3DPY_POSTPROCESSOPTIONS_H
#define EYS3DPY_POSTPROCESSOPTIONS_H

class PostProcessOptions {
private:
    int mSpatialFilterKernelSize = 5;          // 3 - 15 Larger smoother. This value should be in odd.
    float mSpatialFilterOutlierThreshold = 16; // 1 - 64 Smaller filter more
    int mDecimationFactor = 0;                 // Ceil(resolution / factor / 4) * 4

    int mFilteredWidth = 0;
    int mFilteredHeight = 0;

    bool mEnabledDecimation = false;
    bool mEnabledPostProcess = false;

    // Color post process parameter
    int mResizedWidth = 0;
    int mResizedHeight = 0;
    float mColorResizeFactor = 1.0f;
    bool mColorPostProcessEnabled = false;

public:
    PostProcessOptions() : PostProcessOptions(5, 16.0f, 1) { }

    PostProcessOptions(const int spatialKernelSize, const float spatialOutlierThreshold, const int decimationFactor) {
        mSpatialFilterKernelSize = spatialKernelSize;
        mSpatialFilterOutlierThreshold = spatialOutlierThreshold;
        mDecimationFactor = decimationFactor;
    }

    void setSpatialFilterKernelSize (const int spatialKernelSize) {
        mSpatialFilterKernelSize = spatialKernelSize;
    }

    inline int getSpatialFilterKernelSize () const {
        return mSpatialFilterKernelSize;
    }

    void setSpatialOutlierThreshold(const float spatialOutlierThreshold) {
        mSpatialFilterOutlierThreshold = spatialOutlierThreshold;
    }

    inline float getSpatialOutlierThreshold () const {
        return mSpatialFilterOutlierThreshold;
    }

    /**
     * Set current resolution divisor. Set before Camera::initStream and after initStream is completed.
     * Developer might invoke getFilteredHeight() / getFilteredWidth() to know the resized resolution.
     */
    void setDecimationFactor(unsigned short factor) {
        mDecimationFactor = factor;
    }
    /**
     * Get current resolution divisor. It won't affect anything when streaming.
     * @return current decimation factor.
     */
    inline int getDecimationFactor () const {
        return mDecimationFactor;
    }

    /**
     *  This value will be filled after Camera::initStream, ceil(width / DecimationFactor / 4) * 4
     *  e.g. ceil(1280 / 3 / 4) * 4 = 428
     *  And will be the same as Frame::width
     * @return Decimation filter resized width.
     */
    inline int getFilteredWidth () const {
        return mFilteredWidth;
    }

    /**
     *  This value will be filled after Camera::initStream. And will be the same as Frame::height.
     * @return Decimation filter resized height.
     */
    inline int getFilteredHeight () const {
        return mFilteredHeight;
    }

    /**
     * Control over depth post process filter enablement. Could be switched when device is streaming.
     * @param enable
     */
    inline bool isEnabledPostProcess() {
        return mEnabledPostProcess;
    }

    void enablePostProcess(const bool enable) {
        mEnabledPostProcess = enable;
    }

    /**
     * Control over depth decimation subsample filter enablement. Could be switched when device is streaming.
     * @param enable
     */
    void enableDepthDecimation(const bool enable) {
        mEnabledDecimation = enable;
    }

    inline bool isEnabledDepthDecimation() const {
        return mEnabledDecimation;
    }

    /**
     *
     * @param w Internal updated by CameraDevice, which inform user currently decimation height.
     * @return
     */
    inline void setFilteredWidth (int w) {
        mFilteredWidth = w;
    }

    /**
     *
     * @param h Internal updated by CameraDevice, which inform user currently decimation height.
     * @return
     */
    inline void setFilteredHeight (int h) {
        mFilteredHeight = h;
    }

    /**
     *
     * @param w Internal updated by CameraDevice, which inform user currently decimation height.
     * @return
     */
    inline void setResizedWidth (int w) {
        mResizedWidth = w;
    }

    /**
     *
     * @param h Internal updated by CameraDevice, which inform user currently resized color height.
     * @return
     */
    inline void setResizedHeight (int h) {
        mResizedHeight = h;
    }

    /**
     * Control over color post process filter enablement. Could be switched when device is streaming.
     * @param enable
     */
    void enableColorPostProcess(const bool enable) {
        mColorPostProcessEnabled = enable;
    }

    /**
     * Control over color post process filter enablement. Could be switched when device is streaming.
     * @return Is currently resizing color stream?
     */
    inline bool isEnabledColorPostProcess() const {
        return mColorPostProcessEnabled;
    }

    /**
     * Set color resolution factor. Set before Camera::initStream and after initStream is completed.
     */
    void setColorResizeFactor(float factor) {
        mColorResizeFactor = factor;
    }

    /**
     * Get current resolution factor. It won't affect anything when streaming.
     * @return current color resize factor.
     */
    inline float getColorResizeFactor () const {
        return mColorResizeFactor;
    }
};

#endif //EYS3DPY_POSTPROCESSOPTIONS_H
