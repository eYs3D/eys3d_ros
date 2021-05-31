/*
 * Copyright (C) 2015-2017 ICL/ITRI
 * All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of ICL/ITRI and its suppliers, if any.
 * The intellectual and technical concepts contained
 * herein are proprietary to ICL/ITRI and its suppliers and
 * may be covered by Taiwan and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from ICL/ITRI.
 */

#pragma once

#include "debug.h"

#include <cstddef>
#include <memory>

namespace libeYs3D {
namespace devices    {

// forward declaraion
class CameraDevice;

void* MemoryAllocator__allocate(CameraDevice *cameraDevice, size_t size);
void MemoryAllocator__deallocate(CameraDevice *cameraDevice, void *p, size_t size);
size_t MemoryAllocator__max_size(CameraDevice *cameraDevice);

template<typename T>
class MemoryAllocator : public std::allocator<T>    {
public: 
    //    typedefs
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef size_t size_type;
        
public : 
    // convert an allocator<T> to allocator<U>
    template<typename U>
    struct rebind {
        typedef MemoryAllocator<U> other;
    };

public:
    // memory allocation
    pointer allocate(size_type n, const void *hint=0)    {
        return reinterpret_cast<T *>(MemoryAllocator__allocate(mCameraDevice, n * sizeof(T)));
    }
    
    void deallocate(pointer p, size_type size)    {
        MemoryAllocator__deallocate(mCameraDevice, reinterpret_cast<void *>(p), size);
    }
    
    // Returns the largest size for which a call to allocate() might succeed.
    size_type max_size() const    {
        return MemoryAllocator__max_size(mCameraDevice);
    }

    MemoryAllocator(CameraDevice *cameraDevice) : std::allocator<T>(), mCameraDevice(cameraDevice)    {}
    MemoryAllocator(const std::allocator<T> &a) : std::allocator<T>(a)    {
        this->mCameraDevice = nullptr;
    }
    
    // eauivalent to MemoryAllocator(const MemoryAllocator &a) = default;
    MemoryAllocator(const MemoryAllocator &a) : std::allocator<T>(a) {
        this->mCameraDevice = a.mCameraDevice;
    }
    
    ~MemoryAllocator()    {}
    
private:
    CameraDevice *mCameraDevice;
}; // end of class PixelMemoryAllocator

} // namespace devices
} // namespace libeYs3D
