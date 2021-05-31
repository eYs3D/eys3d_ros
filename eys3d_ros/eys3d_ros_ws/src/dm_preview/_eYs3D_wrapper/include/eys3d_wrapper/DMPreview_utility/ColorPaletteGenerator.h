#ifndef COLORPALETTEGENERATOR_H
#define COLORPALETTEGENERATOR_H

#define COLOR_PALETTE_MAX_COUNT 16384

typedef struct tagRGBQUARD {
    unsigned char rgbBlue;
    unsigned char rgbGreen;
    unsigned char rgbRed;
    unsigned char rgbReserved;
} RGBQUAD;

class ColorPaletteGenerator {
    ColorPaletteGenerator();

    static double maxHueAngle ;
    static int defaultAlpha;
public:
    //Generate a color mapping palette RGBA
    static void generatePalette(unsigned char *palette, int paletteSize, bool reverseRedToBlue = true);
    //controlPoint1Value = 0.1f , controlPoint2Value = 0.9f
    static void generatePalette(unsigned char *palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseRedToBlue = true);
    static void generatePalette(unsigned char *palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseRedToBlue = true);
    static void DmGrayMode14(RGBQUAD *palette,  float zFar,float zNear, bool bReverColor);
    static void DmColorMode14(RGBQUAD *palette, float zFar,float zNear, bool bReverColor);
    static void UpdateD11DisplayImage_DIB24(RGBQUAD *pColorPaletteD11, unsigned char *pDepthD11, unsigned char *pDepthDIB24, int cx, int cy);
    static void UpdateZ14DisplayImage_DIB24(RGBQUAD *pColorPaletteZ14, unsigned char *pDepthZ14, unsigned char *pDepthDIB24, int cx, int cy);
    static void UpdateD8bitsDisplayImage_DIB24(RGBQUAD *pColorPalette8bits, unsigned char *pDepth8bits, unsigned char *pDepthDIB24, int cx, int cy);
    static void generatePaletteColor(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseRedToBlue);
    static void generatePaletteGray(RGBQUAD* palette, int size, int mode, int customROI1, int customROI2, bool reverseGraylevel);
    //Generate a gray mapping palette RGBA
    static void generatePaletteGray(unsigned char * palette, int paletteSize, bool reverseGraylevel = true);

    //controlPoint1Value = 0.1f , controlPoint2Value = 0.9f
    static void generatePaletteGray(unsigned char * palette, int paletteSize, int controlPoint1, int controlPoint2, bool reverseGraylevel = true);

    static void generatePaletteGray(unsigned char * palette, int paletteSize, int controlPoint1, float controlPoint1Value, int controlPoint2, float controlPoint2Value, bool reverseGraylevel = true);

private:
    //value range: 0.0f ~ 1.0f as hue angle
    static void getRGB(double value, double &R, double &G, double &B, bool reverseRedToBlue = true);
    static void getRGB_14bits(double value, double &R, double &G, double &B, bool reverseRedToBlue = true);
    static void HSV_to_RGB(double H, double S, double V, double &R, double &G, double &B);
    static void HSV_to_RGB_14bits(double H, double S, double V, double &R, double &G, double &B);
};

#endif // COLORPALETTEGENERATOR_H
