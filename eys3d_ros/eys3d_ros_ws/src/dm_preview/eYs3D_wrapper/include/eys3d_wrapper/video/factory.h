/*
 * Copyright (C) 2015-2017 ICL/ITRI
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
 
 #include "devices/CameraDevice.h"
 #include "video/FrameProducer.h"
 #include "video/PCProducer.h"
 
 #include <memory>                                     // for unique_ptr
 
 
namespace libeYs3D    {
namespace video    {

// To prevent circular include, factory methods are defined separately here...
std::unique_ptr<FrameProducer> createColorFrameProducer(libeYs3D::devices::CameraDevice *cameraDevice);
std::unique_ptr<FrameProducer> createDepthFrameProducer(libeYs3D::devices::CameraDevice *cameraDevice);
std::unique_ptr<PCProducer> createPCFrameProducer(libeYs3D::devices::CameraDevice *cameraDevice);

} // namespace video
} // namespace libeYs3D
