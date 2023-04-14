/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include <errno.h>
#include <string.h>
#ifdef WIN32
#include "DebugCPP.hpp"
#endif
#if (defined(ANDROID) || defined(__ANDROID__))

#include <android/log.h>

// Usage: Define DEBUG_LEVEL before including this header to
//        select the behaviour of the DEBUG() and DDEBUG() macros.
#if defined(DEBUG_LEVEL) && (DEBUG_LEVEL > 0)
    #define DEBUG(TAG, FMT, ...) \
        __android_log_print(ANDROID_LOG_DEBUG, TAG, FMT, ##__VA_ARGS__)

#else
    #define DEBUG(TAG, FMT, ...)
#endif // DEBUG_LEVEL > 0

#if defined(DEBUG_LEVEL) && (DEBUG_LEVEL > 1)
    #define DDEBUG(...) DEBUG(__VA_ARGS__)
#else
    #define DDEBUG(...) (void)0
#endif // EMUGL_DEBUG_LEVEL > 1

#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define LOG_ERR(TAG, FMT, ...)                                                  \
    __android_log_print(ANDROID_LOG_ERROR, TAG, FMT" (%s:%d: errno: %s)",       \
                        ##__VA_ARGS__, __FILE__, __LINE__, clean_errno())       \

#define LOG_ERR_ARGS(TAG, FMT, ARGS)                                            \
    do    {                                                                     \
        char buffer[2048];                                                      \
        int i = vsprintf(buffer, FMT, ARGS);                                    \
        int j = sprintf(&buffer[i], "(%s:%d:%s, errno: %s) ", __FILE__, __LINE__, __func__, clean_errno());   \
        buffer[i + j] = '\0';                                                   \
        __android_log_print(ANDROID_LOG_ERROR, TAG, "%s", buffer);              \
    } while(0)

#define LOG_WARN(TAG, FMT, ...) \
    __android_log_print(ANDROID_LOG_WARN, TAG, FMT" (%s:%d: errno: %s) ",    \
                        ##__VA_ARGS__, __FILE__, __LINE__, clean_errno())

#define LOG_INFO(TAG, FMT, ...) \
    __android_log_print(ANDROID_LOG_INFO, TAG, FMT, ##__VA_ARGS__)

#define LOG_DEBUG(TAG, FMT, ...) \
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "(%s:%d) " FMT "", __FILE__, __LINE__, \
                        ##__VA_ARGS__)

#define LOG_DEBUG_S(TAG, FMT, ...) \
    __android_log_print(ANDROID_LOG_DEBUG, TAG, FMT, ##__VA_ARGS__)

#define LOG_VERBOSE(TAG, FMT, ...) \
    __android_log_print(ANDROID_LOG_VERBOSE, TAG, FMT, ##__VA_ARGS__)

#else /* !(defined(ANDROID) || defined(__ANDROID__)) */

#include <stdio.h>

#if defined(DEBUG_LEVEL) && (DEBUG_LEVEL > 0)
#define DEBUG(TAG, FMT, ...)                           \
    do    {                                            \
        fprintf(stderr, "%s: [DEBUG] ", TAG);          \
        fprintf(stderr, FMT , ##__VA_ARGS__);          \
        fprintf(stderr, "\n");                         \
    } while(0)
#else
#define DEBUG(TAG, FMT, ...)
#endif

#if defined(DEBUG_LEVEL) && (DEBUG_LEVEL > 1)
    #define DDEBUG(...) DEBUG(__VA_ARGS__)
#else
    #define DDEBUG(...) (void)0
#endif // EMUGL_DEBUG_LEVEL > 1

#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define LOG_ERR(TAG, FMT, ...)                    \
    do    {                                       \
        fprintf(stderr, "%s: [ERROR] ", TAG);     \
        fprintf(stderr, FMT, ##__VA_ARGS__);      \
        fprintf(stderr, "(%s:%d:%s, errno: %s) ", __FILE__, __LINE__, __func__, clean_errno());   \
        fprintf(stderr, "\n");                    \
    } while(0)

#define LOG_ERR_ARGS(TAG, FMT, ARGS)          \
    do    {                                   \
        fprintf(stderr, "%s: [ERROR] ", TAG); \
        vfprintf(stderr, FMT, ARGS);          \
        fprintf(stderr, "(%s:%d:%s, errno: %s) ", __FILE__, __LINE__, __func__, clean_errno());   \
        fprintf(stderr, "\n");                \
    } while(0)

#define LOG_WARN(TAG, FMT, ...)                   \
    do    {                                       \
        fprintf(stderr, "%s: [WARN] ", TAG);      \
        fprintf(stderr, FMT, ##__VA_ARGS__);      \
        fprintf(stderr, "\n");                    \
    } while(0)

#ifdef UNITY_DEBUG_LOG
#define LOG_INFO(TAG, FMT, ...)                   \
    do    {                                       \
        Debug::Log(Debug::string_format("%s: [INFO] ", TAG), Color::White);      \
        Debug::Log(Debug::string_format(FMT, __VA_ARGS__), Color::White);      \
        Debug::Log(Debug::string_format("\n"), Color::White);      \
    } while(0)
#else
#define LOG_INFO(TAG, FMT, ...)                   \
    do    {                                       \
        fprintf(stderr, "%s: [INFO] ", TAG);      \
        fprintf(stderr, FMT, ##__VA_ARGS__);      \
        fprintf(stderr, "\n");                    \
    } while(0)
#endif
#define LOG_DEBUG(TAG, FMT, ...)                         \
    do    {                                              \
        fprintf(stderr, "%s: [DEBUG] ", TAG);            \
        fprintf(stderr, "(%s:%d) ", __FILE__, __LINE__); \
        fprintf(stderr, FMT , ##__VA_ARGS__);            \
        fprintf(stderr, "\n");                           \
    } while(0)

#define LOG_DEBUG_S(TAG, FMT, ...)                \
    do    {                                       \
        fprintf(stderr, "%s: [DEBUG] ", TAG);     \
        fprintf(stderr, FMT, ##__VA_ARGS__);      \
        fprintf(stderr, "\n");                    \
    } while(0)

#define LOG_VERBOSE     LOG_ERR
#if 0
#define LOG_VERBOSE(TAG, FMT, ...)                \
    do    {                                       \
        fprintf(stderr, "%s: [DEBUG] ", TAG);     \
        fprintf(stderr, FMT, ##__VA_ARGS__);     \
        fprintf(stderr, "\n");                    \
    } while(0)
#endif

#endif
