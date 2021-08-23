#pragma once
#include <string>
#ifdef WIN32
#  include "eSPDI_Common.h"
#else
#  include "eSPDI.h"
#endif
class RegisterSettings {
	RegisterSettings();
public:
    static int FramesyncD0(void* hAPC, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int Framesync(void* hAPC, PDEVSELINFO pDevSelInfo, PDEVSELINFO pDevSelInfoEx,
                         int DepthWidth, int DepthHeight,
                         int ColorWidth, int ColorHeight,
                         bool bFormatMJPG, int Fps, const int nPid);
    static int FramesyncFor8054(void* hAPC, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int FramesyncFor8040S(void* hAPC, PDEVSELINFO pDevSelInfo, int DepthWidth, int DepthHeight, int ColorWidth, int ColorHeight, bool bFormatMJPG, int Fps);
    static int FrameSync8053_8059(void* hAPC, PDEVSELINFO pDevSelInfo);
    static int FrameSync8053_8059_Clock( void* hAPC, PDEVSELINFO pDevSelInfo );
    static int FrameSync8053_8059_Reset(void* hAPC, PDEVSELINFO pDevSelInfo);
    static int ForEx8053Mode9(void* hAPC, PDEVSELINFO pDevSelInfo);
    static int DM_Quality_Register_Setting(void* hAPC, PDEVSELINFO pDevSelInfo, unsigned short wPID);
    static int DM_Quality_Register_Setting_For6cm(void* hAPC, PDEVSELINFO pDevSelInfo);
    static int DM_Quality_Register_Setting_For15cm(void* hAPC, PDEVSELINFO pDevSelInfo);
};
