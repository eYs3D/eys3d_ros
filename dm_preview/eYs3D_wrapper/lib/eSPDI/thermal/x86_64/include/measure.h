#ifndef __MEASURE_H__
#define __MEASURE_H__
typedef struct
{
	unsigned short emiss;         
	unsigned short relHum;      
	unsigned short distance;      
	short reflectedTemper;         
	short atmosphericTemper;  
	unsigned short modifyK;     
	short modifyB;                     
}guide_measure_external_param_t;

extern "C"
{
     int guide_measure_setinternalparam(char *pParamLine);
     int guide_measure_loadtempercurve(int devType, int gear, int lenType);
     int guide_measure_deloadtempercurve();
     int guide_measure_convertgray2temper(int devType, short *pGray, int len, guide_measure_external_param_t *pParamExt, float *pTemper);
     int guide_measure_converttemper2gray(int devType, float *pTemper, int len, guide_measure_external_param_t *pParamExt, short *pGray);
};

#endif

