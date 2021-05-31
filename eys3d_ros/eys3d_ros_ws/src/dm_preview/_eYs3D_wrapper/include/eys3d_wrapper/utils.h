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

#include <stdint.h>
#include <cstdio>
#include <ctime>

#include "eSPDI_def.h"

typedef struct Rect    {
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
} Rect;

#define MAX(a,b) \
   ({ __typeof__ (a) _a = (a);  \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
     
#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a);  \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

char *string_trim(char *str);

int read_fully(int fd, void* data, size_t byte_count, const char *log_tag);
int write_fully(int fd, const void* data, size_t byte_count, const char *log_tag);

void get_process_name(uint32_t pid, char *procName, int length);
int get_executable_path(char *path, int length, const char *log_tag);
int get_executable_dir(char *dir, int length, const char *log_tag);

#if 0
int64_t now_in_millisecond();
#endif

int64_t now_in_microsecond_unix_time();
int64_t now_in_microsecond_high_res_time();

int get_cpu_core_count();

void get_time_YYYY_MM_DD_HH_MM_SS(int64_t timeMs, char* date, size_t length);

int get_model_name(const char *devPath, char *out, int length, const char *log_tag);
USB_PORT_TYPE get_usb_type(const char *devPath);
