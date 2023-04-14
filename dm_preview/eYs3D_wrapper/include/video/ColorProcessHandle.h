#pragma once
#include <vector>
#include <functional>

#ifndef WIN32
	#include "eSPDI.h"
#else
	#include "eSPDI_Common.h"
#endif

#include "coders.h"
#include "IImageProcess.h"
#include "PostProcessOptions.h"
#include "video/Frame.h"
#include "PostProcessHandle.h"

using PostProcessHandleCallback = std::function<int(bool)>;

namespace libeYs3D {
namespace video {

class ColorProcessHandle : public IImageProcess {

private:
    void *pResizeProcessHandle = nullptr;
    APCImageType::Value mCurrentImageType;
    PostProcessOptions &mOptions;
    const size_t mWidth;
    const size_t mHeight;

    size_t mResizedWidth = 0;
    size_t mResizedHeight = 0;
    bool mIsEnable;
    PostProcessHandleCallback& mCallback;
    std::vector<uint8_t> mCachedResizedBuffer;
    float mResizeFactor = 1.0f;

public:
    ColorProcessHandle(int32_t width, int32_t height, APCImageType::Value imageType,
                       PostProcessOptions &postProcessOptions, PostProcessHandleCallback &cb);

    int32_t getFilteredWidth() override;

    int32_t getFilteredHeight() override;

    inline void notifyCameraIfNeeded();

    inline int process(Frame *f);

    ~ColorProcessHandle();

};

} // video
} // libeys3d