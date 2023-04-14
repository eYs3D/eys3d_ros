#ifndef MAPPING_H
#define MAPPING_H

#include <QtCore/qglobal.h>

#if defined(MAPPING_LIBRARY)
#  define MAPPINGSHARED_EXPORT Q_DECL_EXPORT
#else
#  define MAPPINGSHARED_EXPORT Q_DECL_IMPORT
#endif

class MAPPINGSHARED_EXPORT Mapping
{
public:
    Mapping();

bool Data16ToData8(short *srcData, unsigned char *destData, int size);

bool Data16ToRGB24(short *srcData, unsigned char *destData, int size, int palette);

bool YUVToRGB24(short *srcData, unsigned char *destData, int width, int height);

};

#endif // MAPPING_H
