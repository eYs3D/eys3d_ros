/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#ifdef WIN32
#  include "eSPDI_Common.h"
#else
#  include "eSPDI_def.h"
#endif

#include <stdlib.h> // for abs

#ifndef RETRY_APC_API
#define RETRY_COUNT (5)
#ifdef WIN32 // todo windows impl typeof fail
#define RETRY_APC_API(exp) ([&]() {                       \
    int retryCount = RETRY_COUNT;                           \
    decltype (exp) _rc;                                     \
    do {                                                    \
        _rc = (exp);                                        \
    } while((_rc != APC_OK) && (retryCount-- > 0));     \
    return _rc;}())
#else
#define RETRY_APC_API(exp)  ({                            \
    int retryCount = RETRY_COUNT;                           \
    typeof (exp) _rc;                                       \
    do    {                                                 \
        _rc = (exp);                                        \
    } while((_rc != APC_OK) && (retryCount-- > 0));     \
    _rc; })
#endif
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
