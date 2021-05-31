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

#include <chrono>
#include <thread>

#include "eSPDI_def.h"

#ifndef RETRY_ETRON_API
#define RETRY_COUNT (5)
#define RETRY_ETRON_API(exp)  ({                            \
    int retryCount = RETRY_COUNT;                           \
    typeof (exp) _rc;                                       \
    do    {                                                 \
        _rc = (exp);                                        \
        if (_rc != ETronDI_OK)   std::this_thread::sleep_for(std::chrono::seconds(1));                                     \
    } while((_rc != ETronDI_OK) && (retryCount-- > 0));     \
    _rc; })
#endif
