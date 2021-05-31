#pragma once
#include <map>
class IMUData
{
private:	
	float MAX_G = 4.0f;
	float MAX_DPS = 500.0f;
	float BASE_uT = 0.15f;

public:
	enum IMUPacketSymbol
	{
		FRAME_COUNT,
		SUB_SECOND,
		SEC,
		MIN,
		HOUR,
		ACCEL_X,
		ACCEL_Y,
		ACCEL_Z,
		TEMPARATURE,
		GYROSCOPE_X,
		GYROSCOPE_Y,
		GYROSCOPE_Z,
		COMPASS_X,
		COMPASS_Y,
		COMPASS_Z,
		COMPASS_X_TBC,
		COMPASS_Y_TBC,
		COMPASS_Z_TBC,
        QUATERNION_0,
        QUATERNION_1,
        QUATERNION_2,
        QUATERNION_3,
		ACCURACY_FLAG
	};

	static std::map<IMUPacketSymbol, size_t> sizeTable; 
	static std::map<IMUPacketSymbol, size_t> sizeTable_DMP;
    static std::map<IMUPacketSymbol, size_t> sizeTable_Quaternion;
	IMUData();
	~IMUData();

	void parsePacket(unsigned char * buf, bool normalization);
	void parsePacket_DMP(unsigned char * buf);
    void parsePacket_Quaternion(unsigned char * buf);

    void setMaxG(float fMaxG){ MAX_G = fMaxG; }
    void setMaxDPS(float fMaxDPS){ MAX_DPS = fMaxDPS; }

	int _frameCount;		// RAW:[0-1] 2byte  // DMP:[0-1] 2byte
	int _subSecond;			// RAW:[2-3] 2byte	// DMP:[2-3] 2byte
	int _sec;				// RAW:[4] 1byte	// DMP:[4] 1byte
	int _min;				// RAW:[5] 1byte	// DMP:[5] 1byte
	int _hour;				// RAW:[6] 1byte	// DMP:[6] 1byte
	float _accelX;			// RAW:[7-8]		// DMP:[7-10]
	float _accelY;			// RAW:[9-10]		// DMP:[11-14]
	float _accelZ;			// RAW:[11-12]		// DMP:[15-18]
	int _temprature;		// RAW:[13-14]		// DMP:[19-20]
	float _gyroScopeX;		// RAW:[15-16]		// DMP:[21-24]
	float _gyroScopeY;		// RAW:[17-18]		// DMP:[25-28]
	float _gyroScopeZ;		// RAW:[19-20]		// DMP:[29-32]
	float _compassX;		// RAW:[21-22]		// DMP:[33-36]
	float _compassY;		// RAW:[23-24]		// DMP:[37-40]
	float _compassZ;		// RAW:[25-26]		// DMP:[41-44]
	float _compassX_TBC;	// RAW:[N/A]		// DMP:[45-48]
	float _compassY_TBC;	// RAW:[N/A]		// DMP:[49-52]
	float _compassZ_TBC;	// RAW:[N/A]		// DMP:[53-56]
	char _accuracy_FLAG;	// RAW:[N/A]		// DMP:[57]

    float _quaternion[4];   // 0:[7-10] 1:[11-14] 2:[15-18] 3:[19-22]

};

