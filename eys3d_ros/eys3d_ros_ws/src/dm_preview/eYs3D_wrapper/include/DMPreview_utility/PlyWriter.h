#pragma once
#include <string>
#include <vector>
#ifdef WIN32
#  include <eSPDI_DM.h>
#else
#  include <eSPDI.h>
#endif

struct CloudPoint {
	float x;
	float y;
	float z;
	unsigned char r;
	unsigned char g;
	unsigned char b;
};
class PlyWriter {
	PlyWriter();
public:
    static int writePly(std::vector<CloudPoint>& cloud,  std::string filename);
	//static int apcFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output,float zNear,float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
	//static int apcFrameTo3D(int depthWidth, int depthHeight ,std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool removeINF, bool useDepthResolution, float scale_ratio);
    static int apcFrameTo3D_8029(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
	static int apcFrameTo3D(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
	static int apcFrameTo3D_PlyFilterFloat(int depthWidth, int depthHeight, std::vector<float>& dFloatArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool useDepthResolution, float scale_ratio);
	static int apcFrameTo3DMultiSensor(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool isDownSampleK, float scale_ratio, int degreeOfRectifyLogK);
	static int apcFrameTo3DMultiSensorPlyFilterFloat(int depthWidth, int depthHeight, std::vector<float>& dFloatArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, bool isDownSampleK, float scale_ratio, int degreeOfRectifyLogK);
	static int apcFrameTo3DCylinder(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogData, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, float scale_ratio);
	static int apcFrameTo3DCylinder(int depthWidth, int depthHeight, std::vector<unsigned char>& dArray, int colorWidth, int colorHeight, std::vector<unsigned char>& colorArray, eSPCtrl_RectLogData* rectLogDataL, eSPCtrl_RectLogData*rectLogDataK, APCImageType::Value depthType, std::vector<CloudPoint>& output, bool clipping, float zNear, float zFar, bool removeINF, float scale_ratio);
    static std::string generateHeader(bool binary,int size);
//private:
	static void resampleImage(int srcWidth, int srcHeight, unsigned char* srcBuf, int dstWidth, int dstHeight, unsigned char* dstBuf, int bytePerPixel);  
	//static void resampleImage_float(int srcWidth, int srcHeight, float* srcBuf, int dstWidth, int dstHeight, float* dstBuf);
    static void MonoBilinearFineScaler(unsigned char* pIn, unsigned char* pOut, int width, int height, int outwidth, int outheight, int zero_invalid);
	static void MonoBilinearFineScaler(unsigned char *pIn, unsigned char *pOut, int width, int height, int outwidth, int outheight, int zero_invalid, int bytePerPixel);
    static void MonoBilinearFineScaler_short(unsigned short* pIn, unsigned short* pOut, int width, int height, int outwidth, int outheight, int zero_invalid);
	static void MonoBilinearFineScaler_float(float* pIn, float* pOut, int width, int height, int outwidth, int outheight, int zero_invalid);
	
};

