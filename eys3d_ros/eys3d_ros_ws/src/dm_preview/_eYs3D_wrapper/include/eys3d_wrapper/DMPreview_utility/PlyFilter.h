#pragma once

#include "eSPDI_def.h"
#include <vector>

typedef struct {
	int idx;
	float wgt_sum;
}HIST;

//parameters for WMedFilter_1
typedef struct tagWMedFilter_1Param {
	int xran;
	int yran;				//initial value 33. up, cpu loading up.
	int xc;
	int yc;
	int src;				//0
	int mode;				//0
	int filter_hf_en;		//1
	int filter_hf_ratio;	//4
}WMedFilter_1Param;

//parameters for WMedFilter
typedef struct tagWMedFilterParam {
	int xran;
	int yran;				//initial value 33. up, cpu loading up.
	int xc;
	int yc;
	int src;				//0
	int mode;				//0
	int filter_hf_en;		//1
	int filter_hf_ratio;	//4
}WMedFilterParam;

//parameters for WMeanFilter
typedef struct tagWMeanFilterParam {
	int xran;				// initial 9
	int yran;
	int xc;
	int yc;
	int src;				// 1
	int mode;				// 2 0~2
	int filter_hf_en;		// 0
	int filter_hf_ratio;	// 2 0~8
}WMeanFilterParam;

//parameters for WMeanFilterFloat
typedef struct tagWMeanFilterFloatParam {
	int xran;				// initial 9
	int yran;
	int xc;
	int yc;
}WMeanFilterFloatParam;

class PlyFilter {
	PlyFilter();
public:
	static void MakeGaussTab(float * coeff, float s);
	static void MakeMeanTab(float * coeff, float s);
	static void WMedFilter(unsigned char * pLImg, unsigned short * pDImgIn, unsigned short * pDImgOut, float * coeff, int width, int height, WMedFilterParam * pWMedFilterP);
	static void WMedFilter_1(unsigned char * pLImg, unsigned short * pDImgIn, unsigned short * pDImgOut, float * coeff, int width, int height, WMedFilter_1Param * pWMedFilter_1P);
	static void WMeanFilter(unsigned char * pLImg, unsigned short * pDImgIn, unsigned short * pDImgOut, float * coeff, int width, int height, WMeanFilterParam * pWMeanP);
	static void WMeanFilterFloat(float * pDImgIn, float * pDImgOut, float * coeff, int width, int height, WMeanFilterFloatParam * pWMeanFloatP);
	
    static void UnavailableDisparityCancellation(std::vector<unsigned char>&bufDepth, int widthDepth, int heightDepth, int maxDisparity);

    static void CF_FILTER_Z14(
		std::vector<unsigned char>&bufDepth, std::vector<unsigned char> bufColorRGB, int widthDepth, int heightDepth, int widthColor, int heightColor,
		std::vector<float>&imgFloatBufOut, WMedFilter_1Param * pWMedFilter_1P, WMedFilterParam * pWMedFilterP, WMeanFilterFloatParam * pWMeanFloatP,
		eSPCtrl_RectLogData* pRectifyLogData);

	static void CF_FILTER_Z14(
		std::vector<unsigned char>&bufDepth, std::vector<unsigned char> bufColorRGB, int widthDepth, int heightDepth, int widthColor, int heightColor,
		std::vector<float>&imgFloatBufOut, eSPCtrl_RectLogData* pRectifyLogData);

	static void CF_FILTER(
		std::vector<unsigned char>&bufDepth, std::vector<unsigned char> bufColorRGB, int widthDepth, int heightDepth, int widthColor, int heightColor,
		std::vector<float>&imgFloatBufOut, WMedFilter_1Param * pWMedFilter_1P, WMedFilterParam * pWMedFilterP, WMeanFilterFloatParam * pWMeanFloatP, 
		eSPCtrl_RectLogData* pRectifyLogData);
	
	static void CF_FILTER(
		std::vector<unsigned char>&bufDepth, std::vector<unsigned char> bufColorRGB, int widthDepth, int heightDepth, int widthColor, int heightColor,
		std::vector<float>&imgFloatBufOut, eSPCtrl_RectLogData* pRectifyLogData);

	static void GetFilterParams(WMedFilter_1Param* pWMedFilter_1P, WMedFilterParam* pWMedFilterP, WMeanFilterFloatParam* pWMeanFloatP);
};
