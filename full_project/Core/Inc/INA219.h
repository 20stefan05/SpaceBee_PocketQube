/*
 * INA219.h
 *
 *  Created on: Oct 27, 2023
 *      Author: stefan
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_
#include <stdint.h>
// INA219 structure
typedef struct {
	uint16_t Temperature_RAW;
	uint32_t Pressure_RAW;
	float Temperature;
	float Pressure;
	float Altitude;
} INA219_t;


#endif /* INC_INA219_H_ */
