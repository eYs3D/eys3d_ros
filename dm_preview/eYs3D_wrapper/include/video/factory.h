/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */
#pragma once
 #include "devices/CameraDevice.h"
 #include "video/FrameProducer.h"
 #include "video/PCProducer.h"

#include "PostProcessHandle.h"
#include "ColorProcessHandle.h"
#include <memory>                                     // for unique_ptr

namespace libeYs3D    {
namespace video    {

// To prevent circular include, factory methods are defined separately here...
std::unique_ptr<FrameProducer> createColorFrameProducer(libeYs3D::devices::CameraDevice *cameraDevice);
std::unique_ptr<FrameProducer> createDepthFrameProducer(libeYs3D::devices::CameraDevice *cameraDevice);
std::unique_ptr<PCProducer> createPCFrameProducer(std::shared_ptr<libeYs3D::devices::CameraDevice> cameraDevice);

using PostProcessHandleCallback = std::function<int(bool)>;

class AbstractPostProcessFactory {
public:
    virtual std::unique_ptr<IImageProcess> createColorProcessHandle(int32_t width, int32_t height,
                                                                    APCImageType::Value imageType,
                                                                    PostProcessOptions& postProcessOptions,
                                                                    PostProcessHandleCallback& cb) const = 0;

    virtual std::unique_ptr<IImageProcess> createDepthProcessHandle(int32_t width, int32_t height,
                                                                    APCImageType::Value imageType,
                                                                    PostProcessOptions& postProcessOptions,
                                                                    PostProcessHandleCallback& cb) const = 0;
};

class PostProcessFactory : public AbstractPostProcessFactory {
    std::unique_ptr<IImageProcess> createColorProcessHandle(int32_t width, int32_t height,
                                                            APCImageType::Value imageType,
                                                            PostProcessOptions& postProcessOptions,
                                                            PostProcessHandleCallback& cb) const override {

        std::unique_ptr<IImageProcess> colorProcessHandle(new ColorProcessHandle(width, height, imageType,
                                                                                 postProcessOptions, cb));
        return std::move(colorProcessHandle);
    }

    std::unique_ptr<IImageProcess> createDepthProcessHandle(int32_t width, int32_t height, APCImageType::Value imageType,
                                                            PostProcessOptions& postProcessOptions,
                                                            PostProcessHandleCallback& cb) const override {

        std::unique_ptr<IImageProcess> depthProcessHandle(new PostProcessHandle(width, height, imageType,
                                                                                postProcessOptions, cb));
        return std::move(depthProcessHandle);
    }
};

} // namespace video
} // namespace libeYs3D
