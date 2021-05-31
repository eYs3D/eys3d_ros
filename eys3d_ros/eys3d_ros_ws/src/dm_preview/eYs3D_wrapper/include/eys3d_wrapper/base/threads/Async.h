// Copyright (C) 2015 The Android Open Source Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

#include "base/threads/Types.h"
#include "base/TypeTraits.h"

#include <memory>

namespace libeYs3D    {
namespace base {

// Run a specified functor in a separate thread,
// not waiting for any kind of result
// returns true if thread starts successfully
bool async(const ThreadFunctor& func,
           ThreadFlags flags = ThreadFlags::MaskSignals);

template <class Callable, class = enable_if<is_callable_as<Callable, void()>>>
bool async(Callable&& func, ThreadFlags flags = ThreadFlags::MaskSignals) {
    return async(ThreadFunctor([func]() { func(); return intptr_t(); }), flags);
}

}  // namespace base
}  // namespace libeYs3D
