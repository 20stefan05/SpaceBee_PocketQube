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
//void init_SD_card(void) {
//	FATFS FatFs;                //Fatfs handle
//	FIL fil;                  //File handle
//	FRESULT fres;                 //Result after operations
//	char buf[100];
//
//	do {
//		//Mount the SD Card
//		fres = f_mount(&FatFs, "", 1);    //1=mount now
//		if (fres != FR_OK) {
//			break;
//		}
//
//		//Read the SD Card Total size and Free Size
//		FATFS *pfs;
//		DWORD fre_clust;
//		uint32_t totalSpace, freeSpace;
//
//		f_getfree("", &fre_clust, &pfs);
//		totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
//		freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);
//
//		//Open the file
//		fres = f_open(&fil, "Mission1_Data.csv",
//				FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
//		if (fres != FR_OK) {
//			break;
//		}
//
//		//write the data
//		f_puts("Temperature, Pressure, Altitude, Acceleration\n", &fil);
//
//		//close your file
//		f_close(&fil);
//
//
//	} while (0);
//
//	//We're done, so de-mount the drive
//	f_mount(NULL, "", 0);
//}
void process_SD_card(MPU6050_t *MPU6050, BMP180_t *BMP180, INA219_t *INA219) {
	FATFS FatFs;                //Fatfs handle
	FIL fil;                  //File handle
	FRESULT fres;                 //Result after operations
	char buf[100];

	do {
		//Mount the SD Card
		fres = f_mount(&FatFs, "", 1);    //1=mount now
		if (fres != FR_OK) {
			break;
		}

		//Read the SD Card Total size and Free Size
		FATFS *pfs;
		DWORD fre_clust;
		uint32_t totalSpace, freeSpace;

		f_getfree("", &fre_clust, &pfs);
		totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
		freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

		//Open the file
		fres = f_open(&fil, "Mission1_Data.csv",
				FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
		if (fres != FR_OK) {
			break;
		}

		//write the data
		f_puts("Temperature, Pressure, Altitude, Acceleration\n", &fil);

		//close your file
		f_close(&fil);


	} while (0);

	//We're done, so de-mount the drive
	f_mount(NULL, "", 0);
}
