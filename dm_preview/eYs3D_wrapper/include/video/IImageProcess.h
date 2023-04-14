#ifndef EYS3DPY_IIMAGEPROCESS_H
#define EYS3DPY_IIMAGEPROCESS_H
#include "Frame.h"

namespace libeYs3D {
namespace video {
/**
 * As a Abstract product interface. Concrete product should implement.
 */
class IImageProcess {
public:
    /**
     * Interface that do post process. Such as decimation filter and gaussian filter should override this function.
     * @param Frame f the frame pointer to be processed.
     * @return Suggest developers return APC_POSTPROCESS_NOT_INIT APC_OK
     */
    virtual inline int process(libeYs3D::video::Frame* f) = 0;
    /**
     * Filtered width: For example decimation filter is sub-sampling the depth data, so it will shrink the width.
     * Override this interface for any filter that gives a new size.
     * @return
     */
    virtual inline int32_t getFilteredWidth() = 0;
    /**
     * Filtered height: For example decimation filter is sub-sampling the depth data, so it will shrink the width.
     * Override this interface for any filter that gives a new size.
     * @return
     */
    virtual inline int32_t getFilteredHeight() = 0;
};

} // video
} // libeYs3D



#endif //EYS3DPY_IIMAGEPROCESS_H
