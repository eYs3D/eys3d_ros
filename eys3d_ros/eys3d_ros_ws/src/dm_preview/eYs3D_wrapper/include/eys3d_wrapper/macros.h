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

#include "eSPDI_def.h"

#include <stdlib.h> // for abs

#ifndef RETRY_ETRON_API
#define RETRY_COUNT (5)
#define RETRY_ETRON_API(exp)  ({                            \
    int retryCount = RETRY_COUNT;                           \
    typeof (exp) _rc;                                       \
    do    {                                                 \
        _rc = (exp);                                        \
    } while((_rc != ETronDI_OK) && (retryCount-- > 0));     \
    _rc; })
#endif

#ifndef RETRY_API_BOOL
#define RETRY_API_BOOL(exp)  ({                             \
    int retryCount = RETRY_COUNT;                           \
    typeof (exp) _rc;                                       \
    do    {                                                 \
        _rc = (exp);                                        \
    } while((_rc != true) && (retryCount-- > 0));           \
    _rc; })
#endif

#define WAIT_FOR_MESASAGE(MESSAGE_QUEUE, MESSAGE)                        \
    do    {                                                              \
        auto message = MESSAGE_QUEUE.receive();                          \
        if(message)    {                                                 \
            if(*message == MESSAGE)    break;                            \
                                                                         \
            MESSAGE_QUEUE.send(std::move(*message));                     \
            libeYs3D::base::Thread::sleepMs(48); /* 3 frames/60FPS */    \
        } else    {                                                      \
            break;                                                       \
        }                                                                \
    } while(true);
    
#define IS_SN_ROTATE(current, expect)    \
    ((abs(((int)expect) - ((int)current)) > 0X00FF00) ? true : false)

#define IS_BIG_ENDIAN()        \
    ({                         \
        int i = 1;             \
        (! *((char *)&i));     \
    })
