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

#include <errno.h>
#include <string.h>
#include <stdio.h>

#define clean_errno() (errno == 0 ? "None" : strerror(errno))
#define LOGER_BUFFER_SIZE 2048

void initialize_debug_framework(const char *config_file);

void LOG_ERR_S(const char *TAG, const char *FMT, ...);
#define LOG_ERR(TAG, FMT, ...)                                            \
    do    {                                                                 \
        int i = 0;                                                          \
        char buffer[2048];                                                  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, "\n        ");        \
        snprintf(&buffer[i], sizeof(buffer) - i, "(%s:%d:%s)",              \
                 __FILE__, __LINE__, __func__);                             \
        LOG_ERR_S(TAG, "%s", buffer);                                       \
    } while(0)
    
#define LOG_ERR_ERRNO(TAG, FMT, ...)                                              \
    do    {                                                                 \
        int i = 0;                                                          \
        char buffer[2048];                                                  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, "\n        ");        \
        snprintf(&buffer[i], sizeof(buffer) - i, "(%s:%d:%s, errno: %s)",   \
                 __FILE__, __LINE__, __func__, clean_errno());              \
        LOG_ERR_S(TAG, "%s", buffer);                                       \
    } while(0)
    
void LOG_WARN(const char *TAG, const char *FMT, ...);
void LOG_INFO(const char *TAG, const char *FMT, ...);
void LOG_INFO_VITAL(const char *TAG, const char *FMT, ...);
void LOG_DEBUG_S(const char *TAG, const char *FMT, ...);
#define LOG_DEBUG(TAG, FMT, ...)                                            \
    do    {                                                                 \
        int i = 0;                                                          \
        char buffer[2048];                                                  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, "\n        ");        \
        snprintf(&buffer[i], sizeof(buffer) - i, "(%s:%d:%s)",              \
                 __FILE__, __LINE__, __func__);                             \
        LOG_DEBUG_S(TAG, "%s", buffer);                                     \
    } while(0)

void LOG_VERBOSE_S(const char *TAG, const char *FMT, ...);
#define LOG_VERBOSE(TAG, FMT, ...)                                          \
    do    {                                                                 \
        int i = 0;                                                          \
        char buffer[2048];                                                  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, FMT, ##__VA_ARGS__);  \
        i += snprintf(&buffer[i], sizeof(buffer) - i, "\n        ");        \
        snprintf(&buffer[i], sizeof(buffer) - i, "(%s:%d:%s, errno: %s)",   \
                 __FILE__, __LINE__, __func__, clean_errno());              \
        LOG_VERBOSE_S(TAG, "%s", buffer);                                   \
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
