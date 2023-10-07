#ifndef INC_GY521_H_
#define INC_GY521_H_

#include <stdint.h>
#include "main.h"

// MPU6050 structure
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

// Initializes the sensor, returns 1 if something is wrong, returns 0 if everything went successfully
uint8_t MPU6050_Init(void);

// Updates acceleration and raw acceleration on all 3 axes
void MPU6050_Read_Accel(MPU6050_t *DataStruct);

// Updates gyroscope and raw gyroscope information on all 3 axes
void MPU6050_Read_Gyro(MPU6050_t *DataStruct);

// Updates the temperature
void MPU6050_Read_Temp(MPU6050_t *DataStruct);

// Updates acceleration, gyroscope information, temperature and Kalman angle information
void MPU6050_Read_All(MPU6050_t *DataStruct);

// Currently no idea what it does
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif /* INC_GY521_H_ */
