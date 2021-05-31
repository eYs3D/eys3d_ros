#pragma once
#include <string>
#include "eSPDI.h"

class RegisterSettings {
	RegisterSettings();
public:
    static int FramesyncD0(void* hEtronDI, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int Framesync(void* hEtronDI, PDEVSELINFO pDevSelInfo, PDEVSELINFO pDevSelInfoEx,
                         int DepthWidth, int DepthHeight,
                         int ColorWidth, int ColorHeight,
                         bool bFormatMJPG, int Fps, const int nPid);
    static int FramesyncFor8054(void* hEtronDI, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int FramesyncFor8040S(void* hEtronDI, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int FrameSync8053_8059(void* hEtronDI, PDEVSELINFO pDevSelInfo);
    static int FrameSync8053_8059_Clock( void* hEtronDI, PDEVSELINFO pDevSelInfo );
    static int FrameSync8053_8059_Reset(void* hEtronDI, PDEVSELINFO pDevSelInfo);
    static int ForEx8053Mode9(void* hEtronDI, PDEVSELINFO pDevSelInfo);
    static int DM_Quality_Register_Setting(void* hEtronDI, PDEVSELINFO pDevSelInfo, unsigned short wPID);
    static int DM_Quality_Register_Setting_For6cm(void* hEtronDI, PDEVSELINFO pDevSelInfo);
    static int DM_Quality_Register_Setting_For15cm(void* hEtronDI, PDEVSELINFO pDevSelInfo);
};
