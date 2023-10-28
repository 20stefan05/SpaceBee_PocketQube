#ifndef _BMP180_H_
#define _BMP180_H_
#include <stdint.h>


// BMP180 structure
typedef struct {
	uint16_t Temperature_RAW;
	uint32_t Pressure_RAW;
	float Temperature;
	float Pressure;
	float Altitude;
} BMP180_t;

// Starts reading sensor data
void BMP180_Init(void);

// Updates temperature
void BMP180_Get_Temp(BMP180_t *Datastruct);

// Updates pressure
// oss = oversampling something, currently commented out
void BMP180_Get_Press(BMP180_t *Datastruct, int oss);

// Updates altitude (! and pressure because it calls Get_Press to calculate the altitude)
void BMP180_Get_Alt(BMP180_t *Datastruct, int oss);

// Updates all 3 of the above (temperature, altitude and pressure)
void BMP180_Read_All(BMP180_t *Datastruct, int oss);

#endif /* INC_BMP180_H_ */
