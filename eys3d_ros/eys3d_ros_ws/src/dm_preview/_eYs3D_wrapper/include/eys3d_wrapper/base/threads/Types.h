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
 * Copyright (C) 2015-2019 ICL/ITRI
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

#include "base/EnumFlags.h"

#include <functional>
#include <stdint.h>

namespace libeYs3D    {
namespace base {

// a functor which can run in a separate thread
using ThreadFunctor = std::function<intptr_t()>;

enum class ThreadFlags : unsigned char {
    NoFlags = 0,
    MaskSignals = 1,
    // A Detach-ed thread is a launch-and-forget thread.
    // wait() and tryWait() on a Detach-ed thread always fails.
    // OTOH, if you don't wait() on a non Detach-ed thread it would do it
    // in dtor anyway.
    Detach = 1 << 1
};

}  // namespace base
}  // namespace libeYs3D
