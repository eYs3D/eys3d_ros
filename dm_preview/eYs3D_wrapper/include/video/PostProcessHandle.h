/**
 * This helper class allocate a temporary buffer as output of filter.
 */
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

using PostProcessHandleCallback = std::function<int(bool)>;

namespace libeYs3D {
namespace video {

class PostProcessHandle : public IImageProcess {

private:
    std::vector<unsigned char> mCachedDepthBuffer;
    APCImageType::Value mCurrentImageType;
    void* mPostProcessHandle = nullptr;
    PostProcessOptions& mOptions;
    const size_t mWidth;
    const size_t mHeight;

    void* mDecimationHandle = nullptr;
    std::vector<unsigned char> mCachedDecimationBuffer;
    int32_t mDecimatedWidth = 0;
    int32_t mDecimatedHeight = 0;
    PostProcessHandleCallback& mCallback;
    bool mIsPostProcessEnabled;
    bool mIsDecimationEnabled;

public:
    PostProcessHandle(int32_t width, int32_t height, APCImageType::Value imageType,
                      PostProcessOptions& postProcessOptions, PostProcessHandleCallback& cb);

    int32_t getFilteredWidth() override;

    int32_t getFilteredHeight() override;

    inline void notifyCameraDecimationResolution();
    inline void updatePostProcessIfChange();

    inline int process(Frame* f) override;
    ~PostProcessHandle();
};

} // video
} // libeYs3D