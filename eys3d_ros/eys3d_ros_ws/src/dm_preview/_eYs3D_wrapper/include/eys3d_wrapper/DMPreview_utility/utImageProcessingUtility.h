#ifndef UTIMAGEPROCESSINGUTILITY_H
#define UTIMAGEPROCESSINGUTILITY_H
#include "eSPDI_def.h"
#include "ColorPaletteGenerator.h"

class utImageProcessingUtility
{
public:
    static int convert_yuv_to_rgb_pixel(int y, int u, int v);
    static int convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);

    static void UpdateZ14DisplayImage_DIB24(RGBQUAD *pColorPaletteZ14, unsigned char *pDepthZ14,
                                            unsigned char *pDepthDIB24, int width, int height);
    static void UpdateD11DisplayImage_DIB24(RGBQUAD* pColorPalette, unsigned char *pDepth,
                                            unsigned char *pRGB, int width, int height, BYTE *pZDTable);
    static void UpdateD8bitsDisplayImage_DIB24(RGBQUAD *pColorPalette, unsigned char *pDepth,
                                               unsigned char *pRGB, int width, int height, BYTE *pZDTable,
                                               int nZDTableSize);

    static void UpdateD11_Baseline_DisplayImage_DIB24(RGBQUAD* pColorPalette, WORD *pDepth,
                                                      unsigned char *pRGB, int width, int height,
                                                      double dblCamFocus, double dblBaselineDist,
                                                      int nZNear, int nZFar);
    static void UpdateD11_Fusion_DisplayImage_DIB24(RGBQUAD* pColorPalette, WORD *pDepthFs, WORD *pDepth,
                                                    unsigned char *pRGB, int width, int height,
                                                    double dblCamFocus, double dblBaselineDist,
                                                    int nZNear, int nZFar);

    static void Rotate2Dpoint(float cx, float cy,
                              float cxNew, float cyNew,
                              float angle,
                              float &x, float &y);
private:
    utImageProcessingUtility();
};

#endif // UTIMAGEPROCESSINGUTILITY_H
