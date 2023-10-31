/*
 * SD_Handler.h
 *
 *  Created on: Oct 27, 2023
 *      Author: stefan
 */

#ifndef INC_SD_HANDLER_H_
#define INC_SD_HANDLER_H_

#include "MPU6050.h"
#include "BMP180.h"
#include "INA219.h"
void process_SD_card(MPU6050_t *MPU6050, BMP180_t *BMP180, INA219_t *INA219);
void init_SD_card(void);

#endif /* INC_SD_HANDLER_H_ */
