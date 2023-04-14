/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "EYS3DSystem.h"
#include "devices/MemoryAllocator.h"
#include "devices/IMUDevice.h"
#include "devices/controller/RegisterReadWriteController.h"
#include "devices/model/DepthFilterOptions.h"
#include "devices/model/PostProcessOptions.h"
#include "devices/model/DepthAccuracyOptions.h"
#include "devices/model/CameraDeviceProperties.h"
#include "devices/model/IRProperty.h"
#include "DMPreview_utility/ModeConfigOptions.h"

#include "video/video.h"
#include "video/FrameProducer.h"
#include "video/PCProducer.h"
#include "video/DepthFrameProducer.h"
#include "sensors/SensorDataProducer.h"
#include "Constants.h"
#include "utils.h"
#include <mutex>
#ifdef WIN32
#  include "eSPDI_Common.h"
#  include "magic.h"
#  include <eSPDI_DM.h>
#  define __attribute__(x)
#else
#  include "eSPDI_def.h"
#endif
#include "DMPreview_utility/ColorPaletteGenerator.h"
#include "DMPreview_utility/PlyWriter.h"

#include <stdint.h>
#include <memory>
#include <map>

#define CD_INITIALIZED            0x0001
#define CD_STREAM_INITIALIZED     0x0002
#define CD_COLOR_STREAM_CB        0x0010
#define CD_DEPTH_STREAM_CB        0x0020
#define CD_PC_STREAM_CB           0x0040
#define CD_IMU_STREAM_CB          0x0080
#define CD_COLOR_STREAM_ACTIVATED 0x0100
#define CD_DEPTH_STREAM_ACTIVATED 0x0200
#define CD_PC_STREAM_ACTIVATED    0x0400
#define CD_IMU_STREAM_ACTIVATED   0x0800


// Each device has ColorFrameProducer, DepthFrameProducer & PCFrameProducer,
//     ColorFrameProducer: 8 * 2
//     DepthFrameProducer: 8 * 2
//     PCFrameProducer:    8 * 1
// Since sizeof(uint32_t) == szieof(float) == 4, we use the same memory allocator
#define MAX_PREALLOCATE_PIXEL_FRAME_COUNT    40

//     PCFrameProducer:    8 * 1
#define MAX_PREALLOCATE_XYZ_FRAME_COUNT    8

#define MAX_STREAM_INFO_COUNT (64)
#define SERIAL_THRESHOLD (30)
#define MAX_DEPTH_DISTANCE (16383)

#define DEFAULT_DEPTH_ROI_PIXELS 20

#define APC_USER_SETTING_OFFSET 5

namespace libeYs3D    {

namespace devices    {
    class CameraDevice;
    class Pipeline;
    class FrameSetPipeline;
}

namespace video    { // forward declaration for friendship assignment
    struct Frame;
    class FrameProducer;
    class ColorFrameProducer;
    class DepthFrameProducer;
    class PCFrameProducer;
    class PostProcessHandle;
    int color_image_produce_bgr_frame(const libeYs3D::devices::CameraDevice *cameraDevice,
                                      libeYs3D::video::Frame *frame);
    int depth_image_produce_rgb_frame(const libeYs3D::devices::CameraDevice *cameraDevice,
                                      libeYs3D::video::Frame *frame);
    int generate_point_cloud(CameraDevice *cameraDevice, const char *plyFilePath,
                             std::vector<uint8_t> &depthData,
                             int nDepthWidth, int nDepthHeight,
                             std::vector<uint8_t> &colorData,
                             int nColorWidth, int nColorHeight,
                             bool usePlyFilter);
}

namespace devices    {

struct CameraDeviceInfo    {
    DEVINFORMATION devInfo;

#ifdef WIN32
    char firmwareVersion[PATH_MAX] = "Unsupported";
    char serialNumber[PATH_MAX] = "Unsupported";
    char busInfo[PATH_MAX] = "Unsupported";
    char modelName[PATH_MAX] = "Unsupported";
#else
    char firmwareVersion[PATH_MAX];
    char serialNumber[PATH_MAX];
    char busInfo[PATH_MAX];
    char modelName[PATH_MAX];
#endif
    USB_PORT_TYPE usbPortType;

    CameraDevice *cameraDevice;

    CameraDeviceInfo(DEVINFORMATION *info)    {
        this->devInfo = *info;
        usbPortType = USB_PORT_TYPE_2_0;
    }

    CameraDeviceInfo()    { usbPortType = USB_PORT_TYPE_2_0; }

    int toString(char *buffer, int bufferLength) const;
    int toString(std::string &string) const;
};

struct ZDTableInfo    {
    ZDTABLEINFO nZDTableInfo;
    int32_t nZDTableSize = APC_ZD_TABLE_FILE_SIZE_11_BITS;
    int32_t nActualZDTableLength;
    uint8_t nZDTable[APC_ZD_TABLE_FILE_SIZE_11_BITS] = {0};
    uint16_t nZDTableMaxFar;
    uint16_t nZDTableMaxNear;
};

class CameraDevice
	: public std::enable_shared_from_this<CameraDevice>
{
public:
    using COLOR_BYTE_ORDER = ::libeYs3D::EYS3DSystem::COLOR_BYTE_ORDER;
    CameraDeviceInfo& getCameraDeviceInfo()    { return mCameraDeviceInfo; }

    struct FocalLength{
        int nLeftFx = 0;
        int nLeftFy = 0;
        int nRightFx = 0;
        int nRightFy = 0;
    };

    /*
     * colorFormat: libeYs3D::video::COLOR_RAW_DATA_TYPE
     * depthFormat: libeYs3D::video::DEPTH_RAW_DATA_TYPE
     * depthDataTransferCtrl: How depth frame data is transcoded to RGB
     *     DEPTH_IMG_COLORFUL_TRANSFER, DEPTH_IMG_GRAY_TRANSFER, DEPTH_IMG_NON_TRANSFER (default)
     * 
     * return
     *     0 (APC_OK): succeed
     *     < 0           : align with with error codes defined in eSPDI_def.h
     *     1:            : enabled, please realease the stream before it can be enabled again.
     */
    virtual int initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE colorFormat,
                           int32_t colorWidth, int32_t colorHeight, int32_t actualFps,
                           libeYs3D::video::DEPTH_RAW_DATA_TYPE depthFormat,
                           int32_t depthWidth, int32_t depthHeight,
                           DEPTH_TRANSFER_CTRL depthDataTransferCtrl,
                           CONTROL_MODE ctrlMode,
                           int rectifyLogIndex,
                           libeYs3D::video::Producer::Callback colorImageCallback,
                           libeYs3D::video::Producer::Callback depthImageCallback,
                           libeYs3D::video::PCProducer::PCCallback pcFrameCallback,
                           libeYs3D::sensors::SensorDataProducer::AppCallback imuDataCallback = nullptr);

    std::shared_ptr<Pipeline>
    initStream(libeYs3D::video::COLOR_RAW_DATA_TYPE colorFormat,
               int32_t colorWidth, int32_t colorHeight, int32_t actualFps,
               libeYs3D::video::DEPTH_RAW_DATA_TYPE depthFormat,
               int32_t depthWidth, int32_t depthHeight,
               DEPTH_TRANSFER_CTRL depthDataTransferCtrl,
               CONTROL_MODE ctrlMode,
               int rectifyLogIndex);

    std::shared_ptr<FrameSetPipeline>
    initStreamFS(libeYs3D::video::COLOR_RAW_DATA_TYPE colorFormat,
                 int32_t colorWidth, int32_t colorHeight, int32_t actualFps,
                 libeYs3D::video::DEPTH_RAW_DATA_TYPE depthFormat,
                 int32_t depthWidth, int32_t depthHeight,
                 DEPTH_TRANSFER_CTRL depthDataTransferCtrl,
                 CONTROL_MODE ctrlMode,
                 int rectifyLogIndex);

    void enableStream();
	void enableColorAndDepthStream_Python();
    void enableColorStream();
    void enableDepthStream();
    void enablePCStream();
    void enableIMUStream();
    void pauseStream();
    void pauseColorStream();
    void pauseDepthStream();
    void pausePCStream();
    void pauseIMUStream();
	void pauseColorAndDepthStream_Python();
                            
    virtual int closeStream();
    
    virtual int readColorFrame(uint8_t *buffer,
                               __attribute__ ((unused)) uint64_t bufferLength,
                               uint64_t *actualSize, uint32_t *serial);
    virtual int readDepthFrame(uint8_t *buffer,
                               __attribute__ ((unused)) uint64_t bufferLength,
                               uint64_t *actualSize, uint32_t *serial);
    virtual int readPCFrame(const uint8_t *colorBuffer, const uint8_t *depthBuffer,
                            uint8_t *rgbDataBuffer, float *xyzDataBuffer);
                            
    virtual bool isInterleaveModeSupported()    { return false; }
    virtual bool isInterleaveModeEnabled();
    virtual int enableInterleaveMode(bool enable);

    // return a copy of current device DepthFilterOptions
    virtual DepthFilterOptions getDepthFilterOptions();
    virtual void setDepthFilterOptions(DepthFilterOptions depthFilterOptions);
    
    // return a copy of current device DepthAccuracyOptions
    virtual DepthAccuracyOptions getDepthAccuracyOptions();
    virtual void setDepthAccuracyOptions(DepthAccuracyOptions depthAccuracyOptions);
    virtual Rect getDepthAccuracyRegion()    { return mDepthAccuracyRegion; }
    
    // ROI (Region of Interest) support
    virtual void setDepthROICenterPoint(int x, int y);
    virtual void setDepthROIPixels(int count); // 1 ~ 40
        
    // return a copy of current device property item
    virtual CameraDeviceProperties::CameraPropertyItem
            getCameraDeviceProperty(CameraDeviceProperties::CAMERA_PROPERTY type);
    virtual int setCameraDevicePropertyValue(CameraDeviceProperties::CAMERA_PROPERTY type,
                                              int32_t value);
    virtual int reloadCameraDeviceProperties();

    // return a copy of current device IR property
    virtual IRProperty getIRProperty();
    virtual int setIRMax(bool ExtendIREnabled);
    virtual int setIRProperty(IRProperty property);
    bool enableExtendIR(bool enabled);

    virtual float getManuelExposureTimeMs();
    virtual void setManuelExposureTimeMs(float fMS);
    virtual float getManuelGlobalGain();
    virtual void setManuelGlobalGain(float fGlobalGain);
    
    // return a copy of current device RegisterReadWriteOptions
    virtual RegisterReadWriteOptions getRegisterReadWriteOptions();
    virtual void setRegisterReadWriteOptionsForRead(RegisterReadWriteOptions registerReadWriteOptions);
    virtual void setRegisterReadWriteOptionsForWrite(RegisterReadWriteOptions registerReadWriteOptions);

    std::unique_ptr<ModeConfigOptions> getModeConfigOptions();
    std::unique_ptr<ModeConfigOptions> getModeConfigOptions(USB_PORT_TYPE usbType, unsigned short nPID);

    // miscellaneous
    void getDepthOfField(uint16_t *nZDTableMaxNear, uint16_t *nZDTableMaxFar);
    void setDepthOfField(uint16_t nZDTableMaxNear, uint16_t nZDTableMaxFar);
    USB_PORT_TYPE getUsbPortType()    { return mUsbPortType; }
    eSPCtrl_RectLogData getRectifyLogData()    { return mRectifyLogData; }

    virtual bool isHWPPSupported() { return true; }
    virtual bool isHWPPEnabled();
    virtual int enableHWPP(bool enable);
    virtual int adjustRegisters();

    // IMU
    IMUDevice::IMUDeviceInfo getIMUDeviceInfo();
    //virtual int ConfigIMU(){ return APC_OK; }
    virtual bool isIMUDeviceSupported()    { return false; }
    virtual bool isIMUDevicePresent()    { return (isIMUDeviceSupported() && (mIMUDevice != nullptr)); }
    //virtual CIMUModel *GetIMUModel(){ return m_pIMUModel; }
    //virtual void SetIMUSyncWithFrame(bool bSync);
    //virtual bool IsIMUSyncWithFrame();
    void dumpIMUData(int recordCount = 256);
    std::vector<IMUDevice::IMU_DATA_FORMAT> getSupportDataFormat() {return mIMUDevice->getSupportDataFormat();}
    int selectDataFormat(IMUDevice::IMU_DATA_FORMAT format);
    
    void dumpFrameInfo(int frameCount = 60);
    void doSnapshot(int StreamType);
    virtual bool isPlyFilterSupported() { return true; }
    void enablePlyFilter(bool enable);
    bool isPlyFilterEnabled() { return mPlyFilterEnabled; }
    
    void dumpCameraDeviceProperties();

	int setupIR(unsigned short IRvalue);
	unsigned short getHWRegister(unsigned short address);
	int setHWRegister(unsigned short address, unsigned short nValue);
	unsigned short getFWRegister(unsigned short address);
	int setFWRegister(unsigned short address, unsigned short nValue);
    unsigned short getSensorRegister(unsigned short address, SENSORMODE_INFO sensorMode, int slaveID);
    int setSensorRegister(unsigned short address, unsigned short nValue, SENSORMODE_INFO sensorMode, int slaveID);
	//++Calibration info
    int  loadRectifyLogData();
    std::shared_ptr<eSPCtrl_RectLogData> getRectifyLogData(int rect_index);
    //--Calibration info
    virtual void release();
    virtual ~CameraDevice();

    int getModuleID();
    int setModuleID(uint8_t nID);
    void CopyFromG1toG2();
    void copy_file_to_g2(int fileIndex);
    int toString(char *buffer, int bufferLength);

    std::vector<APC_STREAM_INFO> getColorStreamInfo() { return mColorStreamInfo; }
    std::vector<APC_STREAM_INFO> getDepthStreamInfo() { return mDepthStreamInfo; }

    FocalLength GetFocalLength() { return m_FocalLength; }
	void UpdateFocalLength();
    uint16_t nZNear_default;
    virtual int initIRProperty();

protected:
    explicit CameraDevice(DEVSELINFO *devSelInfo, DEVINFORMATION *deviceInfo, const COLOR_BYTE_ORDER colorByteOrder = COLOR_BYTE_ORDER::COLOR_RGB24);
    explicit CameraDevice(DEVSELINFO *devSelInfo, DEVINFORMATION *deviceInfo);

    virtual int initStreamInfoList();
    virtual int updateZDTable();
    virtual int32_t getZDTableDataType();
    virtual int32_t getZDTableSize();
    virtual int getZDTableIndex();
    
    virtual void updateColorPalette();
    
    virtual int configurePointCloudInfo(bool isUseCached);
    virtual int enableBlockingRead(bool blocking);
    void initPostProcessOptions();
    virtual int initDepthFilterOptions();
    virtual int initDepthAccuracyOptions();
    virtual int initDepthROIOptions();
    
    virtual int initRegisterReadWriteOptions();
    virtual void adjustDepthInvalidBandPixel();

    // child classes with IMU support must overide this to initialize mIMUDevice
    virtual int initIMUDevice();
    virtual void releaseIMUDevice();

    // Due to, in C++, virtual function can not be called inside constructor/destructor
    // postInitialize is provide for post initialize right after CameraDevice object is created
    virtual int postInitialize();
    
    void __enableColorStream();
    void __enableDepthStream();
    void __enablePCStream();
    void __enableIMUStream();
    void __pauseColorStream();
    void __pauseDepthStream();
    void __pausePCStream();
    void __pauseIMUStream();

protected:
    DEVSELINFO mDevSelInfo;
    CameraDeviceInfo mCameraDeviceInfo;
    std::vector<APC_STREAM_INFO> mColorStreamInfo;
    std::vector<APC_STREAM_INFO> mDepthStreamInfo;
    
    DepthFilterOptions mDepthFilterOptions;
    DepthAccuracyOptions mDepthAccuracyOptions;
    Rect mDepthAccuracyRegion;
    CameraDeviceProperties mCameraDeviceProperties;
    IRProperty mIRProperty;
    const COLOR_BYTE_ORDER mColorByteOrder;
    PostProcessOptions mPostProcessOptions;

public:
    PostProcessOptions &getPostProcessOptions();
    void setPostProcessOptions(PostProcessOptions &postProcessOptions);
    RegisterReadWriteOptions mRegisterReadWriteOptions;
    RegisterReadWriteController mRegisterReadWriteController;
    
    // ROI Support
    int mDepthROICenterPointX = 0;
    int mDepthROITopLeftX = 0;
    int mDepthROITopLeftY = 0;
    int mDepthROICenterPointY = 0;
    int mDepthROIBottomRightX = 0;
    int mDepthROIBottomRightY = 0;
    int mDepthROIPixels = DEFAULT_DEPTH_ROI_PIXELS; // Default is 20

    USB_PORT_TYPE mUsbPortType;

    libeYs3D::video::COLOR_RAW_DATA_TYPE mColorFormat;
    int32_t mColorWidth;
    int32_t mColorHeight;
    int32_t mActualFps;
    libeYs3D::video::DEPTH_RAW_DATA_TYPE mDepthFormat;
    int32_t mDepthWidth;
    int32_t mDepthHeight;
    DEPTH_TRANSFER_CTRL mDepthDataTransferCtrl;
    CONTROL_MODE mCtrlMode;
    
    struct ZDTableInfo mZDTableInfo;
    
    RGBQUAD mColorPaletteZ14[COLOR_PALETTE_MAX_COUNT];
    RGBQUAD mGrayPaletteZ14[COLOR_PALETTE_MAX_COUNT];
    
    int mRectifyLogIndex;
    eSPCtrl_RectLogData mRectifyLogData;
    struct PointCloudInfo mPointCloudInfo;

    //++Calibration info
     std::vector<std::shared_ptr<eSPCtrl_RectLogData>> mCameraRectifyLogData;
    //--Calibration info

    uint32_t mDepthInvalidBandPixel;
    
    bool mBlockingRead;
    uint32_t mCameraDeviceState;
    bool mInterleaveModeEnabled;

    bool mSupportingInterleave;

    // IMU
    IMUDevice *mIMUDevice = nullptr;
    
    std::shared_ptr<Pipeline> mPipeline;
    std::shared_ptr<FrameSetPipeline> mFrameSetPipeline;

    bool mPlyFilterEnabled;

    std::unique_ptr<libeYs3D::video::FrameProducer> mColorFrameProducer;
    std::unique_ptr<libeYs3D::video::FrameProducer> mDepthFrameProducer;
    std::unique_ptr<libeYs3D::video::PCProducer> mPCFrameProducer;

	int m_nPixelUnit;
    FocalLength m_FocalLength;

public:
    friend class CameraDeviceFactory;
    friend class DepthFilterOptions;
    friend class CameraDeviceProperties;
    friend class RegisterReadWriteController;
    friend class IMUDevice;
    friend class libeYs3D::EYS3DSystem;
    friend class libeYs3D::video::FrameProducer;
    friend class libeYs3D::video::ColorFrameProducer;
    friend class libeYs3D::video::DepthFrameProducer;
    friend class libeYs3D::video::PCFrameProducer;

    friend class MemoryAllocator<uint8_t>;
    friend class MemoryAllocator<uint32_t>;
    friend class MemoryAllocator<float>;
    friend void* MemoryAllocator__allocate(CameraDevice *cameraDevice, size_t size);
    friend void MemoryAllocator__deallocate(CameraDevice *cameraDevice, void *p, size_t size);
    friend size_t MemoryAllocator__max_size(CameraDevice *cameraDevice);
    
    friend int libeYs3D::video::color_image_produce_bgr_frame(const CameraDevice *cameraDevice,
                                                              libeYs3D::video::Frame *frame);
    friend int libeYs3D::video::depth_image_produce_rgb_frame(const CameraDevice *cameraDevice,
                                                              libeYs3D::video::Frame *frame);
    friend int libeYs3D::video::generate_point_cloud(CameraDevice *cameraDevice,
                                                     const char *plyFilePath,
                                                     std::vector<uint8_t> &depthData,
                                                     int nDepthWidth, int nDepthHeight,
                                                     std::vector<uint8_t> &colorData,
                                                     int nColorWidth, int nColorHeight,
                                                     bool usePlyFilter);

#ifdef WIN32
    static constexpr int kMaxFrames = 64;
    base::MessageChannel<libeYs3D::video::Frame, kMaxFrames> mColorQueue;
    base::MessageChannel<libeYs3D::video::Frame, kMaxFrames> mDepthQueue;
    base::MessageChannel<libeYs3D::video::Frame, kMaxFrames> mCFreeQueue;
    base::MessageChannel<libeYs3D::video::Frame, kMaxFrames> mDFreeQueue;
#endif

#ifdef DEVICE_MEMORY_ALLOCATOR
protected:
    // memory allocation
    std::map<void *, size_t>mMemories;
    
    void *requestMemory(size_t size);
    void returnMemory(const void *memory, size_t size);
    int preallocateMemory();
    void releasePreallocatedMemory();

	MemoryAllocator<uint16_t> mPixelWordMemoryAllocator;
    MemoryAllocator<uint8_t> mPixelByteMemoryAllocator;
    MemoryAllocator<float> mPixelFloatMemoryAllocator;
	
    
#endif
private:
    std::mutex mPclInfoLck;
    int getRectifyMatLogDataTwice();
};

class CameraDeviceFactory    {
    using COLOR_BYTE_ORDER = ::libeYs3D::EYS3DSystem::COLOR_BYTE_ORDER;
public:
    static std::shared_ptr<CameraDevice> createCameradevice(DEVSELINFO *devSelInfo, DEVINFORMATION *devInfo, COLOR_BYTE_ORDER colorByteOrder);
};

} // end of namespace devices
} // end of namespace libeYs3D
