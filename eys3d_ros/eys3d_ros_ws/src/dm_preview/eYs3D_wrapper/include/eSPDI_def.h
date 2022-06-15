/*! \file eSPDI_def.h
  	\brief error/data type definitions
  	
  	Copyright:

	This file copyright (C) 2021 by eYs3D Microelectronics, Co.

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D Microelectronics, Co.
 */
#ifndef LIB_ESPDI_DEF_H
#define LIB_ESPDI_DEF_H

//define Error Code by Wolf 2015/08/07 +
#define  APC_OK                        0
#define  APC_NoDevice                 -1
#define  APC_NullPtr                  -2
#define  APC_ErrBufLen                -3
#define  APC_Init_Fail                -4
#define  APC_NoZDTable                -5 
#define  APC_READFLASHFAIL            -6
#define  APC_WRITEFLASHFAIL           -7 
#define  APC_VERIFY_DATA_FAIL         -8
#define  APC_KEEP_DATA_FAIL           -9
#define  APC_RECT_DATA_LEN_FAIL      -10
#define  APC_RECT_DATA_PARSING_FAIL  -11
#define  APC_RET_BAD_PARAM           -12
#define  APC_RET_OPEN_FILE_FAIL      -13
#define  APC_NO_CALIBRATION_LOG      -14
#define  APC_POSTPROCESS_INIT_FAIL   -15
#define  APC_POSTPROCESS_NOT_INIT    -16
#define  APC_POSTPROCESS_FRAME_FAIL  -17
#define  APC_NotSupport              -18
#define  APC_GET_RES_LIST_FAIL       -19
#define  APC_READ_REG_FAIL           -20
#define  APC_WRITE_REG_FAIL          -21
#define  APC_SET_FPS_FAIL            -22
#define  APC_VIDEO_RENDER_FAIL       -23
#define  APC_OPEN_DEVICE_FAIL        -24
#define  APC_FIND_DEVICE_FAIL        -25
#define  APC_GET_IMAGE_FAIL          -26
#define  APC_NOT_SUPPORT_RES         -27
#define  APC_CALLBACK_REGISTER_FAIL  -28
#define  APC_CLOSE_DEVICE_FAIL	 	 -29
#define  APC_GET_CALIBRATIONLOG_FAIL -30
#define  APC_SET_CALIBRATIONLOG_FAIL -31
#define  APC_DEVICE_NOT_SUPPORT	     -32
#define  APC_DEVICE_BUSY		     -33
#define  APC_DEVICE_TIMEOUT		     -34
#define  APC_IO_SELECT_EINTR	     -35
#define APC_IO_SELECT_ERROR          -36

// for 3D Scanner +    
#define  APC_ILLEGAL_ANGLE                -40
#define  APC_ILLEGAL_STEP                 -41
#define  APC_ILLEGAL_TIMEPERSTEP          -42
#define  APC_MOTOR_RUNNING                -43 
#define  APC_GETSENSORREG_FAIL            -44
#define  APC_SETSENSORREG_FAIL            -45
#define  APC_READ_X_AXIS_FAIL             -46
#define  APC_READ_Y_AXIS_FAIL             -47
#define  APC_READ_Z_AXIS_FAIL             -48
#define  APC_READ_PRESS_DATA_FAIL         -49
#define  APC_READ_TEMPERATURE_FAIL        -50
#define  APC_RETURNHOME_RUNNING           -51
#define  APC_MOTOTSTOP_BY_HOME_INDEX      -52
#define  APC_MOTOTSTOP_BY_PROTECT_SCHEME  -53
#define  APC_MOTOTSTOP_BY_NORMAL          -54
#define  APC_ILLEGAL_FIRMWARE_VERSION     -55
#define  APC_ILLEGAL_STEPPERTIME          -56
// for 3D Scanner - 

// For AEAWB + 
#define  APC_GET_PU_PROP_VAL_FAIL    	  -60
#define  APC_SET_PU_PROP_VAL_FAIL         -61
#define  APC_GET_CT_PROP_VAL_FAIL         -62
#define  APC_SET_CT_PROP_VAL_FAIL    	  -63
#define  APC_GET_CT_PROP_RANGE_STEP_FAIL  -64
#define  APC_GET_PU_PROP_RANGE_STEP_FAIL  -65
// For AEAWB - 

// for Dewarping + Stitching +
#define  APC_INVALID_USERDATA             -70
#define  APC_MAP_LUT_FAIL                 -71
#define  APC_APPEND_TO_FILE_FRONT_FAIL    -72
// for Dewarping + Stitching -

#define APC_TOO_MANY_DEVICE               -80
#define APC_ACCESS_MP4_EXTRA_DATA_FAIL    -81

//define Error Code by Wolf 2015/08/07 +

// define windows type +
#ifndef BYTE
typedef unsigned char BYTE;
#endif //BYTE

#ifndef BOOL
typedef signed int BOOL;
#endif //BOOL

#ifndef WORD
typedef unsigned short WORD;
#endif //WORD
// define windows type -

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

#define FG_Address_1Byte 0x01
#define FG_Address_2Byte 0x02
#define FG_Value_1Byte   0x10
#define FG_Value_2Byte   0x20

typedef struct packet_s {
    int len;
    int serial;
    bool bisRGB;
    bool bisReady;
    union {
        unsigned char buffer_yuyv[2 * 2560 * 2560];
        unsigned char buffer_RGB[3 * 2560 * 2560];
    };
} srb_packet_s;

// Reference Naming
#define EVENT_BUFFER_SHM_COLOR "/shm_ring_buffer_color"
#define EVENT_BUFFER_SHM_DEPTH "/shm_ring_buffer_depth"
#define EVENT_BUFFER_SHM "/shm_ring_buffer"
#define CMD_FIFO_PATH "/tmp/cmdfifo"
#define ZD_PATH "/tmp/zd_addr"
#define RECTIFY_LOG_PATH "/tmp/rectifylog_addr"
//-----------------------------------------------
#define SRB_LENGTH 10

// for Sensor mode +
typedef enum
{
  SENSOR_A = 0,
  SENSOR_B,
  SENSOR_BOTH,
  SENSOR_C,
  SENSOR_D
} SENSORMODE_INFO;
// for Sensor mode +

// register address define +
#define CHIPID_ADDR         0xf014
#define SERIAL_2BIT_ADDR    0xf0fe
// register address define -

typedef enum {
    YUV22_YUYV_PIXEL_FMT = 0,
    YUV22_UYVY_PIXEL_FMT,
    RAW10_GBRG_PIXEL_FMT,
    RAW10_BGGR_PIXEL_FMT,
    RAW10_RGGB_PIXEL_FMT,
    RAW10_GRBG_PIXEL_FMT,
    MJPEG_PIXEL_FMT,
    UNKOWN_PIXEL_FMT = 0xffff
} PIXEL_FMT;

//Check the doc 'video_mode_setting_rul_0.4b.pdf' in http://redmine.etron.com.tw/redmine/issues/6408
// For Depth Data Type
#define APC_DEPTH_DATA_OFF_RAW			0 /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_DEFAULT			APC_DEPTH_DATA_OFF_RAW /* raw (depth off, only gray raw color) */
#define APC_DEPTH_DATA_8_BITS				1 /* rectify, 1 byte per pixel */
#define APC_DEPTH_DATA_14_BITS				2 /* rectify, 2 byte per pixel */
#define APC_DEPTH_DATA_8_BITS_x80			3 /* rectify, 2 byte per pixel but using 1 byte only */
#define APC_DEPTH_DATA_11_BITS				4 /* rectify, 2 byte per pixel but using 11 bit only */
#define APC_DEPTH_DATA_OFF_RECTIFY		5 /* rectify (depth off, only rectify raw color) */
#define APC_DEPTH_DATA_8_BITS_RAW			6 /* raw */
#define APC_DEPTH_DATA_14_BITS_RAW		7 /* raw */
#define APC_DEPTH_DATA_8_BITS_x80_RAW	8 /* raw */
#define APC_DEPTH_DATA_11_BITS_RAW		9 /* raw */
#define APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY     11// 
#define APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY     13// multi-baseline
#define APC_DEPTH_DATA_OFF_BAYER_RAW     14
// For Interleave mode depth data type
#define APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET 16
#define APC_DEPTH_DATA_ILM_OFF_RAW			APC_DEPTH_DATA_OFF_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_ILM_DEFAULT			APC_DEPTH_DATA_DEFAULT + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_ILM_8_BITS			APC_DEPTH_DATA_8_BITS + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* rectify, 1 byte per pixel */
#define APC_DEPTH_DATA_ILM_14_BITS			APC_DEPTH_DATA_14_BITS + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* rectify, 2 byte per pixel */
#define APC_DEPTH_DATA_ILM_8_BITS_x80		APC_DEPTH_DATA_8_BITS_x80 + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* rectify, 2 byte per pixel but using 1 byte only */
#define APC_DEPTH_DATA_ILM_11_BITS			APC_DEPTH_DATA_11_BITS + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* rectify, 2 byte per pixel but using 11 bit only */
#define APC_DEPTH_DATA_ILM_OFF_RECTIFY		APC_DEPTH_DATA_OFF_RECTIFY + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* rectify (depth off, only rectify color) */
#define APC_DEPTH_DATA_ILM_8_BITS_RAW		APC_DEPTH_DATA_8_BITS_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* raw */
#define APC_DEPTH_DATA_ILM_14_BITS_RAW		APC_DEPTH_DATA_14_BITS_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* raw */
#define APC_DEPTH_DATA_ILM_8_BITS_x80_RAW	APC_DEPTH_DATA_8_BITS_x80_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* raw */
#define APC_DEPTH_DATA_ILM_11_BITS_RAW		APC_DEPTH_DATA_11_BITS_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET /* raw */
#define APC_DEPTH_DATA_ILM_14_BITS_COMBINED_RECTIFY     APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET//
#define APC_DEPTH_DATA_ILM_11_BITS_COMBINED_RECTIFY     APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET// multi-baseline
 
#define APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET 32
#define APC_DEPTH_DATA_SCALE_DOWN_OFF_RAW			    (APC_DEPTH_DATA_OFF_RAW + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)/* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_SCALE_DOWN_DEFAULT			    (APC_DEPTH_DATA_DEFAULT + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)  /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_SCALE_DOWN_8_BITS				(APC_DEPTH_DATA_8_BITS + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)/* rectify, 1 byte per pixel */
#define APC_DEPTH_DATA_SCALE_DOWN_14_BITS				(APC_DEPTH_DATA_14_BITS + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* rectify, 2 byte per pixel */
#define APC_DEPTH_DATA_SCALE_DOWN_8_BITS_x80			(APC_DEPTH_DATA_8_BITS_x80 + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* rectify, 2 byte per pixel but using 1 byte only */
#define APC_DEPTH_DATA_SCALE_DOWN_11_BITS				(APC_DEPTH_DATA_11_BITS + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET)/* rectify, 2 byte per pixel but using 11 bit only */
#define APC_DEPTH_DATA_SCALE_DOWN_OFF_RECTIFY		    (APC_DEPTH_DATA_OFF_RECTIFY + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* Rule 0.4b  Reserved unused in any firmware*/
#define APC_DEPTH_DATA_SCALE_DOWN_8_BITS_RAW			(APC_DEPTH_DATA_8_BITS_RAW + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_14_BITS_RAW		    (APC_DEPTH_DATA_14_BITS_RAW + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_8_BITS_x80_RAW	    (APC_DEPTH_DATA_8_BITS_x80_RAW + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_11_BITS_RAW		    (APC_DEPTH_DATA_11_BITS_RAW + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_14_BITS_COMBINED_RECTIFY     (APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* Rule 0.4b Reserved unused in any firmware*/
#define APC_DEPTH_DATA_SCALE_DOWN_11_BITS_COMBINED_RECTIFY     (APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET) /* Rule 0.4b Reserved unused in any firmware*/

// For Interleave mode depth data type
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_OFF_RAW			(APC_DEPTH_DATA_SCALE_DOWN_OFF_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_DEFAULT			(APC_DEPTH_DATA_SCALE_DOWN_DEFAULT + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* raw (depth off, only raw color) */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS			(APC_DEPTH_DATA_SCALE_DOWN_8_BITS + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* rectify, 1 byte per pixel */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_14_BITS			(APC_DEPTH_DATA_SCALE_DOWN_14_BITS + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* rectify, 2 byte per pixel */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS_x80		(APC_DEPTH_DATA_SCALE_DOWN_8_BITS_x80 + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* rectify, 2 byte per pixel but using 1 byte only */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_11_BITS			(APC_DEPTH_DATA_SCALE_DOWN_11_BITS + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* rectify, 2 byte per pixel but using 11 bit only */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_OFF_RECTIFY		(APC_DEPTH_DATA_SCALE_DOWN_OFF_RECTIFY + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* rectify (depth off, only rectify color) */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS_RAW		(APC_DEPTH_DATA_SCALE_DOWN_8_BITS_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_14_BITS_RAW		(APC_DEPTH_DATA_SCALE_DOWN_14_BITS_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS_x80_RAW	(APC_DEPTH_DATA_SCALE_DOWN_8_BITS_x80_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_11_BITS_RAW		(APC_DEPTH_DATA_SCALE_DOWN_11_BITS_RAW + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) /* raw */
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_14_BITS_COMBINED_RECTIFY     (APC_DEPTH_DATA_SCALE_DOWN_14_BITS_COMBINED_RECTIFY + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) //
#define APC_DEPTH_DATA_SCALE_DOWN_ILM_11_BITS_COMBINED_RECTIFY (APC_DEPTH_DATA_SCALE_DOWN_11_BITS_COMBINED_RECTIFY + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET) // multi-baseline

// for Flash Read/Write +
// Firmware (size in KBytes)
#define APC_READ_FLASH_TOTAL_SIZE			128
#define APC_READ_FLASH_FW_PLUGIN_SIZE		104
#define APC_WRITE_FLASH_TOTAL_SIZE			128

// PlugIn data (size in bytes)
#define APC_Y_OFFSET_FILE_ID_0				30
#define APC_Y_OFFSET_FILE_SIZE			    256
#define APC_RECTIFY_FILE_ID_0				40
#define APC_RECTIFY_FILE_SIZE				1024
#define APC_ZD_TABLE_FILE_ID_0				50
#define APC_ZD_TABLE_FILE_SIZE_8_BITS		512
#define APC_ZD_TABLE_FILE_SIZE_11_BITS		4096
#define APC_CALIB_LOG_FILE_ID_0				240
#define APC_CALIB_LOG_FILE_SIZE				4096
#define APC_USER_DATA_FILE_ID_0				200
#define APC_USER_DATA_FILE_SIZE_0			1024
#define APC_USER_DATA_FILE_SIZE_1			4096
#define APC_BACKUP_USER_DATA_FILE_ID        201
#define APC_BACKUP_USER_DATA_SIZE           1024




// for device information +
typedef struct tagDEVINFORMATION {
  unsigned short wPID;
  unsigned short wVID;
  char *strDevName;
  unsigned short  nChipID;
  unsigned short  nDevType;
} DEVINFORMATION, *PDEVINFORMATION;
// for device information -

#define APC_PID_8029    0x0568
#define APC_PID_8030    APC_PID_8029
#define APC_PID_8039    APC_PID_8029
#define APC_PID_8031    0x0117
#define APC_PID_8032    0x0118
#define APC_PID_8036    0x0120
#define APC_PID_8037    0x0121
#define APC_PID_8038    0x0124
#define APC_PID_8038_M0 APC_PID_8038
#define APC_PID_8038_M1 0x0147
#define APC_PID_8040W   0x0130
#define APC_PID_8040S   0x0131
#define APC_PID_8040S_K 0x0149
#define APC_PID_8041    0x0126
#define APC_PID_8042    0x0127
#define APC_PID_8043    0x0128
#define APC_PID_8044    0x0129
#define APC_PID_8045K   0x0134
#define APC_PID_8046K   0x0135
#define APC_PID_8051    0x0136
#define APC_PID_8052    0x0137
#define APC_PID_8053    0x0138
#define APC_PID_8054    0x0139
#define APC_PID_8054_K  0x0143
#define APC_PID_8059    0x0146
#define APC_PID_8060    0x0152
#define APC_PID_8060_K  0x0150
#define APC_PID_8060_T  0x0151
#define APC_PID_AMBER   0x0112
#define APC_PID_SALLY   0x0158
#define APC_PID_HYPATIA 0x0160
#define APC_PID_HYPATIA2 0x0173
#define APC_PID_8062    0x0162
#define APC_PID_8063     0x0164
#define APC_PID_8063_K   0x0165
#define APC_PID_IVY     0x0177
#define APC_PID_GRAP    0x0179
#define APC_PID_GRAP_K  0x0183
#define APC_PID_GRAP_SLAVE   0x0279
#define APC_PID_GRAP_SLAVE_K 0x0283
#define APC_PID_SANDRA  0x0167
#define APC_PID_NORA  0x0168 //NOTE: http://redmine.etron.com.tw/redmine/issues/6688#change-36410
#define APC_PID_HELEN  0x0171 //NOTE: http://redmine.etron.com.tw/redmine/issues/6649#note-50
//+[Thermal device]
#define APC_PID_GRAP_THERMAL 0xf9f9
#define APC_PID_GRAP_THERMAL2 0xf8f8
#define APC_VID_GRAP_THERMAL 0x04b4
//-[Thermal device]
#define APC_VID_2170 0x0110

// for device selection information +
typedef struct tagDEVSEL
{
  int index;
} DEVSELINFO, *PDEVSELINFO;
// for device selection information -

// for output stream info +
typedef struct tagAPC_STREAM_INFO {
	int		nWidth;
	int		nHeight;
	BOOL	bFormatMJPG;
} APC_STREAM_INFO, *PAPC_STREAM_INFO;
// for output stream info -

// for ZD table info +
typedef struct tagZDTableInfo
{
  int nIndex;
  int nDataType;
} ZDTABLEINFO, *PZDTABLEINFO;
// for ZD table info -

// for device type +
typedef enum {
  OTHERS = 0,
  AXES1,
  PUMA,
  KIWI,
  UNKNOWN_DEVICE_TYPE = 0xffff
}DEVICE_TYPE;
// for device type -

// for total and fw+plugin read/write +
typedef enum {	
    Total = 0,
    FW_PLUGIN,
    BOOTLOADER_ONLY,
    FW_ONLY,
    PLUGIN_ONLY,
    UNP
} FLASH_DATA_TYPE;
// for total and fw+plugin read/write -

// for user data +
typedef enum
{
  USERDATA_SECTION_0 = 0,
  USERDATA_SECTION_1,
  USERDATA_SECTION_2,
  USERDATA_SECTION_3,
  USERDATA_SECTION_4,
  USERDATA_SECTION_5,
  USERDATA_SECTION_6,
  USERDATA_SECTION_7,
  USERDATA_SECTION_8,
  USERDATA_SECTION_9
} USERDATA_SECTION_INDEX;
// for user data -

// for Calibration Log Type +
typedef enum {	
	ALL_LOG = 0,
    SERIAL_NUMBER,
    PRJFILE_LOG,
    STAGE_TIME_RESULT_LOG,
    SENSOR_OFFSET,
    AUTO_ADJUST_LOG,
    RECTIFY_LOG,
    ZD_LOG,
    DEPTHMAP_KOG
} CALIBRATION_LOG_TYPE;
// for Calibration Log Type -

typedef struct tagKEEP_DATA_CTRL {	
	bool  bIsSerialNumberKeep;
	bool  bIsSensorPositionKeep;
	bool  bIsRectificationTableKeep;
	bool  bIsZDTableKeep;
	bool  bIsCalibrationLogKeep;
} KEEP_DATA_CTRL;

typedef enum{
	IMAGE_SN_NONSYNC = 0,
	IMAGE_SN_SYNC
} CONTROL_MODE;

typedef enum{
	DEPTH_IMG_NON_TRANSFER,
	DEPTH_IMG_GRAY_TRANSFER,
	DEPTH_IMG_COLORFUL_TRANSFER
}DEPTH_TRANSFER_CTRL;


// for Sensor type name +
typedef enum {
    APC_SENSOR_TYPE_H22 = 0,
    APC_SENSOR_TYPE_H65 = 1,
    APC_SENSOR_TYPE_OV7740 = 2,
    APC_SENSOR_TYPE_AR0134 = 3,
    APC_SENSOR_TYPE_AR0135 = 4,
    APC_SENSOR_TYPE_AR0144 = 5,
    APC_SENSOR_TYPE_AR0330 = 6,
    APC_SENSOR_TYPE_AR0522 = 7,
    APC_SENSOR_TYPE_AR1335 = 8,
    APC_SENSOR_TYPE_OV9714 = 9,
    APC_SENSOR_TYPE_OV9282 = 10,
    APC_SENSOR_TYPE_H68 = 11,
    APC_SENSOR_TYPE_OV2740 = 12,
    APC_SENSOR_TYPE_OC0SA10 = 13,
    APC_SENSOR_TYPE_UNKOWN = 0xffff
} SENSOR_TYPE_NAME; 
// for Sensor type name -

typedef enum
{
  AE_ENABLE = 0,
  AE_DISABLE
} AE_STATUS, *PAE_STATUS;

typedef enum
{
  AWB_ENABLE = 0,
  AWB_DISABLE
} AWB_STATUS, *PAWB_STATUS;

typedef enum
{
    USB_PORT_TYPE_2_0 = 2,
    USB_PORT_TYPE_3_0,
    USB_PORT_TYPE_UNKNOW
} USB_PORT_TYPE;

//
// CT Property ID
//
#define	CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL       0
#define CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL   1
#define CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL   2
#define CT_PROPERTY_ID_EXPOSURE_TIME_RELATIVE_CTRL   3
#define CT_PROPERTY_ID_FOCUS_ABSOLUTE_CTRL           4
#define CT_PROPERTY_ID_FOCUS_RELATIVE_CTRL           5
#define CT_PROPERTY_ID_FOCUS_AUTO_CTRL          	 6
#define CT_PROPERTY_ID_IRIS_ABSOLUTE_CTRL            7
#define CT_PROPERTY_ID_IRIS_RELATIVE_CTRL            8
#define CT_PROPERTY_ID_ZOOM_ABSOLUTE_CTRL            9
#define CT_PROPERTY_ID_ZOOM_RELATIVE_CTRL           10
#define CT_PROPERTY_ID_PAN_ABSOLUTE_CTRL            11
#define CT_PROPERTY_ID_PAN_RELATIVE_CTRL            12
#define CT_PROPERTY_ID_TILT_ABSOLUTE_CTRL           13
#define CT_PROPERTY_ID_TILT_RELATIVE_CTRL           14
#define CT_PROPERTY_ID_PRIVACY_CTRL                 15

//
// PU Property ID
//
#define PU_PROPERTY_ID_BACKLIGHT_COMPENSATION_CTRL   0
#define PU_PROPERTY_ID_BRIGHTNESS_CTRL      		 1
#define PU_PROPERTY_ID_CONTRAST_CTRL 			     2
#define PU_PROPERTY_ID_GAIN_CTRL 			     	 3
#define PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL 	 4
#define PU_PROPERTY_ID_HUE_CTRL 			         5
#define PU_PROPERTY_ID_HUE_AUTO_CTRL 			     6
#define PU_PROPERTY_ID_SATURATION_CTRL 			     7
#define PU_PROPERTY_ID_SHARPNESS_CTRL 			     8
#define PU_PROPERTY_ID_GAMMA_CTRL 			     	 9
#define PU_PROPERTY_ID_WHITE_BALANCE_CTRL 			10
#define PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL 	    11

//Auto-Exposure Mode Control
#define AE_MOD_MANUAL_MODE							0x01
#define AE_MOD_AUTO_MODE							0x02
#define AE_MOD_SHUTTER_PRIORITY_MODE				0x04
#define AE_MOD_APERTURE_PRIORITY_MODE				0x08

// White Balance Temperature, Auto Control
#define PU_PROPERTY_ID_AWB_DISABLE 	    			0
#define PU_PROPERTY_ID_AWB_ENABLE 	    			1
// for Rectify Log +

typedef struct eSPCtrl_RectLogData {
	union {
		unsigned char uByteArray[1024];/**< union data defined as below struct { }*/
		struct {
			unsigned short	InImgWidth;/**< Input image width(SideBySide image) */
			unsigned short	InImgHeight;/**< Input image height */
			unsigned short	OutImgWidth;/**< Output image width(SideBySide image) */
			unsigned short	OutImgHeight;/**< Output image height */
			int		        RECT_ScaleEnable;/**< Rectified image scale */
			int		        RECT_CropEnable;/**< Rectified image crop */
			unsigned short	RECT_ScaleWidth;/**< Input image width(Single image) *RECT_Scale_Col_N /RECT_Scale_Col_M */
			unsigned short	RECT_ScaleHeight;/**< Input image height(Single image) *RECT_Scale_Row_N /RECT_Scale_Row_M */
			float	        CamMat1[9];/**< Left Camera Matrix
								fx, 0, cx, 0, fy, cy, 0, 0, 1 
								fx,fy : focus  ; cx,cy : principle point */
			float	        CamDist1[8];/**< Left Camera Distortion Matrix
								k1, k2, p1, p2, k3, k4, k5, k6 
								k1~k6 : radial distort ; p1,p2 : tangential distort */
			float			CamMat2[9];/**< Right Camera Matrix
								fx, 0, cx, 0, fy, cy, 0, 0, 1  
								fx,fy : focus  ; cx,cy : principle point */
			float			CamDist2[8];/**< Right Camera Distortion Matrix
								k1, k2, p1, p2, k3, k4, k5, k6 
								k1~k6 : radial distort ; p1,p2 : tangential distort */
			float			RotaMat[9];/**< Rotation matrix between the left and right camera coordinate systems. 
								| [0] [1] [2] |       |Xcr|
								| [3] [4] [5] |   *   |Ycr|            => cr = right camera coordinate
								| [6] [7] [8] |       |Zcr| */
			float			TranMat[3];/**< Translation vector between the coordinate systems of the cameras. 
								|[0]|      |Xcr|
								|[1]|   +  |Ycr|	             => cr = right camera coordinate
								|[2]|      |Zcr| */
			float			LRotaMat[9];/**< 3x3 rectification transform (rotation matrix) for the left camera. 
								| [0] [1] [2] |       |Xcl|
								| [3] [4] [5] |   *   |Ycl|            => cl = left camera coordinate
								| [6] [7] [8] |       |Zcl| */
			float			RRotaMat[9];/**< 3x3 rectification transform (rotation matrix) for the left camera.
								| [0] [1] [2] |       |Xcr|
								| [3] [4] [5] |   *   |Ycr|            => cr = right camera coordinate
								| [6] [7] [8] |       |Zcr| */
			float			NewCamMat1[12];/**< 3x4 projection matrix in the (rectified) coordinate systems for the left camera. 
								fx' 0 cx' 0 0 fy' cy' 0 0 0 1 0 
								fx',fy' : rectified focus ; cx', cy; : rectified principle point */
			float			NewCamMat2[12];/**< 3x4 projection matrix in the (rectified) coordinate systems for the rightt camera. 
								fx' 0 cx' TranMat[0]* 0 fy' cy' 0 0 0 1 0 
								fx',fy' : rectified focus ; cx', cy; : rectified principle point */
			unsigned short	RECT_Crop_Row_BG;/**< Rectidied image crop row begin */
			unsigned short	RECT_Crop_Row_ED;/**< Rectidied image crop row end */
			unsigned short	RECT_Crop_Col_BG_L;/**< Rectidied image crop column begin */
			unsigned short	RECT_Crop_Col_ED_L;/**< Rectidied image crop column end */
			unsigned char	RECT_Scale_Col_M;/**< Rectified image scale column factor M */
			unsigned char	RECT_Scale_Col_N;/**< Rectified image scale column factor N
								Rectified image scale column ratio =  Scale_Col_N/ Scale_Col_M */
			unsigned char	RECT_Scale_Row_M;/**< Rectified image scale row factor M */
			unsigned char	RECT_Scale_Row_N;/**< Rectified image scale row factor N */
			float			RECT_AvgErr;/**< Reprojection error */
			unsigned short	nLineBuffers;/**< Linebuffer for Hardware limitation < 60 */
            float ReProjectMat[16];
            float K6Ratio; //Ratio for distortion K6
		};
	};
} eSPCtrl_RectLogData;

// for Rectify Log -

// for Post Process +

#define POSTPAR_HR_MODE 		5
#define POSTPAR_HR_CURVE_0 		6
#define POSTPAR_HR_CURVE_1 		7
#define POSTPAR_HR_CURVE_2 		8
#define POSTPAR_HR_CURVE_3 		9
#define POSTPAR_HR_CURVE_4 		10
#define POSTPAR_HR_CURVE_5 		11
#define POSTPAR_HR_CURVE_6 		12
#define POSTPAR_HR_CURVE_7 		13
#define POSTPAR_HR_CURVE_8 		14
#define POSTPAR_HF_MODE 		17
#define POSTPAR_DC_MODE 		20
#define POSTPAR_DC_CNT_THD 		21
#define POSTPAR_DC_GRAD_THD 	22
#define POSTPAR_SEG_MODE 		23
#define POSTPAR_SEG_THD_SUB 	24
#define POSTPAR_SEG_THD_SLP 	25
#define POSTPAR_SEG_THD_MAX 	26
#define POSTPAR_SEG_THD_MIN 	27
#define POSTPAR_SEG_FILL_MODE 	28
#define POSTPAR_HF2_MODE 		31
#define POSTPAR_GRAD_MODE 		34
#define POSTPAR_TEMP0_MODE 		37
#define POSTPAR_TEMP0_THD 		38
#define POSTPAR_TEMP1_MODE 		41
#define POSTPAR_TEMP1_LEVEL 	42
#define POSTPAR_TEMP1_THD 		43
#define POSTPAR_FC_MODE 		46
#define POSTPAR_FC_EDGE_THD 	47
#define POSTPAR_FC_AREA_THD 	48
#define POSTPAR_MF_MODE 		51
#define POSTPAR_ZM_MODE 		52
#define POSTPAR_RF_MODE 		53
#define POSTPAR_RF_LEVEL 		54

// for Post Process -

// for 3D Motor Control +

// for Gyro +

typedef enum
{
    DPS_245 = 0,
    DPS_500,
    DPS_2000
} SENSITIVITY_LEVEL_L3G;

typedef struct GyroTag
{
	short x; 		
	short y;
	short z;
} GYRO_ANGULAR_RATE_DATA;

// for Gyro -

// for accelerometer and magnetometer +

typedef struct AccelerationTag
{
	short x; 		
	short y;
	short z;
} ACCELERATION_DATA;

typedef struct CompassTag
{
	short x; 		
	short y;
	short z;
} COMPASS_DATA;

typedef enum
{
    _2G = 0,
    _4G,
    _6G,
    _8G,
    _16G
} SENSITIVITY_LEVEL_LSM;

// for accelerometer and magnetometer -

// for pressure +

typedef enum
{
    One_Shot = 0,
    _1_HZ_1_HZ,
    _7_HZ_1_HZ,
    _12_5_HZ_1HZ,
    _25_HZ_1_HZ,
    _7_HZ_7_HZ,
     _12_5_HZ_12_5_HZ,
    _25_HZ_25_HZ
} OUTPUT_DATA_RATE;

// for pressure -

// for LED and Laser +

typedef enum
{
    POWER_ON = 0,
    POWER_OFF
} POWER_STATE;

typedef enum
{
    LEVEL_0 = 0,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4,
    LEVEL_5,
    LEVEL_6,
    LEVEL_7,
    LEVEL_8,
    LEVEL_9,
    LEVEL_10,
    LEVEL_11,
    LEVEL_12,
    LEVEL_13,
    LEVEL_14,
    LEVEL_15
} BRIGHTNESS_LEVEL;

// for LED and Laser -

// for 3D Motor Control -

// for Point Cloud
struct APCImageType
{
    enum Value
    {
        IMAGE_UNKNOWN = -1,
        COLOR_YUY2 = 0,
        COLOR_RGB24,
        COLOR_MJPG,
        COLOR_UYVY,
        DEPTH_8BITS = 100,
        DEPTH_8BITS_0x80,
        DEPTH_11BITS,
        DEPTH_14BITS
    };

    static bool IsImageColor(APCImageType::Value type)
    {
        return (type == COLOR_YUY2 || type == COLOR_RGB24 || type == COLOR_MJPG);
    }

    static bool IsImageDepth(APCImageType::Value type)
    {
        return (type != IMAGE_UNKNOWN && !IsImageColor(type));
    }

    static APCImageType::Value DepthDataTypeToDepthImageType(WORD dataType)
    {
        switch (dataType)
        {
        case APC_DEPTH_DATA_8_BITS:
        case APC_DEPTH_DATA_8_BITS_RAW:
        case APC_DEPTH_DATA_ILM_8_BITS:
        case APC_DEPTH_DATA_ILM_8_BITS_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_8_BITS:
        case APC_DEPTH_DATA_SCALE_DOWN_8_BITS_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS_RAW:
            return APCImageType::DEPTH_8BITS;
        case APC_DEPTH_DATA_8_BITS_x80:
        case APC_DEPTH_DATA_8_BITS_x80_RAW:
        case APC_DEPTH_DATA_ILM_8_BITS_x80:
        case APC_DEPTH_DATA_ILM_8_BITS_x80_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_8_BITS_x80:
        case APC_DEPTH_DATA_SCALE_DOWN_8_BITS_x80_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS_x80:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_8_BITS_x80_RAW:
            return APCImageType::DEPTH_8BITS_0x80;
        case APC_DEPTH_DATA_11_BITS:
        case APC_DEPTH_DATA_11_BITS_RAW:
        case APC_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_ILM_11_BITS:
        case APC_DEPTH_DATA_ILM_11_BITS_RAW:
        case APC_DEPTH_DATA_ILM_11_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_SCALE_DOWN_11_BITS:
        case APC_DEPTH_DATA_SCALE_DOWN_11_BITS_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_11_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_11_BITS:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_11_BITS_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_11_BITS_COMBINED_RECTIFY:
            return APCImageType::DEPTH_11BITS;
        case APC_DEPTH_DATA_14_BITS:
        case APC_DEPTH_DATA_14_BITS_RAW:
        case APC_DEPTH_DATA_14_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_ILM_14_BITS:
        case APC_DEPTH_DATA_ILM_14_BITS_RAW:
        case APC_DEPTH_DATA_ILM_14_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_SCALE_DOWN_14_BITS:
        case APC_DEPTH_DATA_SCALE_DOWN_14_BITS_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_14_BITS_COMBINED_RECTIFY:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_14_BITS:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_14_BITS_RAW:
        case APC_DEPTH_DATA_SCALE_DOWN_ILM_14_BITS_COMBINED_RECTIFY:
            return APCImageType::DEPTH_14BITS;
        default: return APCImageType::IMAGE_UNKNOWN;
        }
    }
};
/**
 * @param M_dst input camera matrix of RGB-lens, including intrinsic parameters, such as RectifyLog-CamMat2 (M3).
 *              The buffer size is 9.
 * @param R_dst_to_src input rotation matrix of dst-lens to src-lens, dst is the camera at left side, src is the camera at
 *                     right side, such as RectifyLog-RotaMat (R31). The buffer size is 9.
 * @param T_dst_to_src  input translation matrix of dst-lens to src-lens, such as RectifyLog-TranMat (T13).
 *                     The buffer size is 3.
 */

struct PointCloudInfo
{
//normal data
    float centerX;
    float centerY;
    float focalLength;
    float disparityToW[ 2048 ];
    int   disparity_len;
    WORD  wDepthType;
    int depth_image_edian; //0: lillte-edian, 1: big-edia
    //multi-lens data
    float focalLength_K;
    float baseline_K;
    float diff_K;
    float slaveDeviceCamMat2[9];
    float slaveDeviceRotaMat[9];
    float slaveDeviceTranMat[3];
};
#endif // LIB_ESPDI_DEF_H
