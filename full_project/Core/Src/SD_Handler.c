/*
 * SD_Handler.c
 *
 *  Created on: Oct 27, 2023
 *      Author: stefan
 */
#include "SD_Handler.h"
#include "main.h"
#include "BMP180.h"
#include "MPU6050.h"
#include "INA219.h"
#include "fatfs.h"
#include "File_Handling.h"
void init_SD_card(void) {
	Mount_SD("/");
	Create_File("Mission1.csv");
	Update_File("Mission1.csv", "Temperature, Pressure, Ax, Ay, Az\n");
	//Unmount_SD("/");

}
char buf[100];
int indx = 1;
void process_SD_card(MPU6050_t *MPU6050, BMP180_t *BMP180, INA219_t *INA219) {
	int dec[5], frac[5];
	dec[0] = (int)BMP180->Temperature;
	frac[0] = (abs(BMP180->Temperature)-abs(dec[0]))*1000;
	dec[1] = (int)BMP180->Pressure;
	frac[1] = (abs(BMP180->Pressure)-abs(dec[1]))*1000;
	dec[2] = (int)MPU6050->Ax;
	frac[2] = (abs(MPU6050->Ax)-abs(dec[2]))*1000;
	dec[3] = (int)MPU6050->Ay;
	frac[3] = (abs(MPU6050->Ay)-abs(dec[3]))*1000;
	dec[4] = (int)MPU6050->Az;
	frac[4] = (abs(MPU6050->Az)-abs(dec[4]))*1000;
	sprintf(buf, "%d.%d, %d.%d, %d.%d, %d.%d, %d.%d\n", dec[0], frac[0], dec[1], frac[1], dec[2], frac[2], dec[3], frac[3], dec[4], frac[4]);
	Mount_SD("/");
	Update_File("Mission1.csv", buf);
	Unmount_SD("/");
	indx++;
}
