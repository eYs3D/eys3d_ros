#include "utils.h"

#ifndef WIN32
#include "eSPDI.h"
#else
#include "eSPDI_Common.h"
#endif

#include "EYS3DSystem.h"
#include "devices/CameraDevice.h"
#include "devices/model/CameraDeviceProperties.h"
#include "devices/model/IRProperty.h"
#include "video/Frame.h"
#include "debug.h"
#include "PlyWriter.h"

typedef struct {
    int colorFormat;
    int colorWidth;
    int colorHeight;
    int fps;
    int depthWidth;
    int depthHeight;
    int videoMode;
} CameraOpenConfig;

extern "C" {
#ifdef WIN32
int TransformDepthDataType(int depth_raw_data_type , int bRectifyMode , unsigned short wPID , int depthWidth , int depthHeight);
CameraOpenConfig get_mode_config_by_pif(int pif);
int get_pif();
CameraOpenConfig get_config();
#endif

int get_depth_frame(BYTE* dataInOut, size_t bufInSize, int type);
bool regenerate_palette(unsigned short zMin, unsigned short zFar);
void reset_palette();
unsigned short getDefaultZNear();
unsigned short getDefaultZFar();
//int setupIR(unsigned short value);
int setupIR(uint16_t value);
uint16_t getIRValue();
bool enableExtendIR(bool enabled);
int init_device(void);
int open_device(CameraOpenConfig config, bool is_point_cloud);
int get_color_frame(BYTE* imageInOut);
void close_device(void);
void release_device(void);
WORD get_depth_by_coordinate(int x, int y);
int generate_point_cloud_gpu(unsigned char *colorData, unsigned char *depthData,
                             unsigned char *colorOut, int* colorCapacity, float* depthOut, int* depthCapacity);
}
