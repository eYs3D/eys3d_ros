/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <errno.h>
#include <string.h>
#include <stdio.h>

#define clean_errno() (errno == 0 ? "None" : strerror(errno))
#define LOGER_BUFFER_SIZE 2048

void initialize_debug_framework(const char *config_file);

void deinitialize_debug_framework();

void LOG_ERR_S(const char *TAG, const char *FMT, ...);
#define LOG_ERR(TAG, FMT, ...)                                            \
    do    {                                                                 \
        int i = 0;                                                          \
        char _buffer[2048];                                                  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, "\n        ");        \
        snprintf(&_buffer[i], sizeof(_buffer) - i, "(%s:%d:%s)",              \
                 __FILE__, __LINE__, __func__);                             \
        LOG_ERR_S(TAG, "%s", _buffer);                                       \
    } while(0)
    
#define LOG_ERR_ERRNO(TAG, FMT, ...)                                              \
    do    {                                                                 \
        int i = 0;                                                          \
        char _buffer[2048];                                                  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, "\n        ");        \
        snprintf(&_buffer[i], sizeof(_buffer) - i, "(%s:%d:%s, errno: %s)",   \
                 __FILE__, __LINE__, __func__, clean_errno());              \
        LOG_ERR_S(TAG, "%s", _buffer);                                       \
    } while(0)
    
void LOG_WARN(const char *TAG, const char *FMT, ...);
void LOG_INFO(const char *TAG, const char *FMT, ...);
void LOG_INFO_VITAL(const char *TAG, const char *FMT, ...);
void LOG_DEBUG_S(const char *TAG, const char *FMT, ...);
#define LOG_DEBUG(TAG, FMT, ...)                                            \
    do    {                                                                 \
        int i = 0;                                                          \
        char _buffer[2048];                                                  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, "\n        ");        \
        snprintf(&_buffer[i], sizeof(_buffer) - i, "(%s:%d:%s)",              \
                 __FILE__, __LINE__, __func__);                             \
        LOG_DEBUG_S(TAG, "%s", _buffer);                                     \
    } while(0)

void LOG_VERBOSE_S(const char *TAG, const char *FMT, ...);
#define LOG_VERBOSE(TAG, FMT, ...)                                          \
    do    {                                                                 \
        int i = 0;                                                          \
        char _buffer[2048];                                                  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&_buffer[i], sizeof(_buffer) - i, "\n        ");        \
        snprintf(&_buffer[i], sizeof(_buffer) - i, "(%s:%d:%s, errno: %s)",   \
                 __FILE__, __LINE__, __func__, clean_errno());              \
        LOG_VERBOSE_S(TAG, "%s", _buffer);                                   \
    } while(0)

void LOG_PERFORMANCE(const char *TAG, const char *FMT, ...);
void LOG_DEPTH_ACCURACY(const char *TAG, const char *FMT, ...);

void LOG_COLOR_PRODUCER_TICK(const char *TAG, const char *FMT, ...);
void LOG_DEPTH_PRODUCER_TICK(const char *TAG, const char *FMT, ...);
void LOG_PC_PRODUCER_TICK(const char *TAG, const char *FMT, ...);
void LOG_IMU_PRODUCER_TICK(const char *TAG, const char *FMT, ...);

#if defined(DEBUG_LEVEL) && (DEBUG_LEVEL > 0)
    #define DEBUG(TAG, FMT, ...) LOG_DEBUG_S(TAG,  FMT , ##__VA_ARGS__)
#else
    #define DEBUG(TAG, FMT, ...) (void)0
#endif

#if defined(DEBUG_LEVEL) && (DEBUG_LEVEL > 1)
    #define DDEBUG(...) DEBUG(__VA_ARGS__)
#else
    #define DDEBUG(...) (void)0
#endif // DEBUG_LEVEL > 1
