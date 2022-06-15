/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */
 
 #include "devices/CameraDevice.h"
 #include "video/FrameProducer.h"
 #include "video/PCProducer.h"
 
 #include <memory>                                     // for unique_ptr
 
 
namespace libeYs3D    {
namespace video    {

// To prevent circular include, factory methods are defined separately here...
std::unique_ptr<FrameProducer> createColorFrameProducer(libeYs3D::devices::CameraDevice *cameraDevice);
std::unique_ptr<FrameProducer> createDepthFrameProducer(libeYs3D::devices::CameraDevice *cameraDevice);
std::unique_ptr<PCProducer> createPCFrameProducer(std::shared_ptr<libeYs3D::devices::CameraDevice> cameraDevice);

} // namespace video
} // namespace libeYs3D
