#ifndef _BMP180_H_
#define _BMP180_H_

#include "main.h"

// MPU6050 structure
typedef struct {
	uint16_t Temperature_RAW;
	uint32_t Pressure_RAW;
	float Temperature;
	float Pressure;
	float Altitude;
} BMP180_t;

void BMP180_Init(void);

void BMP180_Get_Temp(BMP180_t *Datastruct);

void BMP180_Get_Press(BMP180_t *Datastruct, int oss);

void BMP180_Get_Alt(BMP180_t *Datastruct, int oss);

void BMP180_Read_All(BMP180_t *Datastruct, int oss);

#endif /* INC_BMP180_H_ */
