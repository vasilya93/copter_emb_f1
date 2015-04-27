#ifndef SETTINGS_H
#define SETTINGS_H

//#define USE_USART3 //if commented, USART2 (XBEE) will be used

//--------------------------Settings for MPU6050--------------------------------

#define SETTINGS_MPU6050_ACCRANGE_2G
//#define SETTINGS_MPU6050_ACCRANGE_4G
//#define SETTINGS_MPU6050_ACCRANGE_8G
//#define SETTINGS_MPU6050_ACCRANGE_16G

//#define SETTINGS_MPU6050_GYRORANGE_250DEG
#define SETTINGS_MPU6050_GYRORANGE_500DEG
//#define SETTINGS_MPU6050_GYRORANGE_1000DEG
//#define SETTINGS_MPU6050_GYRORANGE_2000DEG

//#define SETTINGS_MPU6050_SEND_RAW_GYRO
//#define SETTINGS_MPU6050_SEND_RAW_ACC

//-----------------------Settings for sensors fusion----------------------------

//#define SETTINGS_SENSFUS_PERFORM_FILTRATION

#define SETTINGS_SENSFUS_SIZE_FILTER_ACCEL 5 //ignored if filtration is not performed
#define SETTINGS_SENSFUS_SIZE_FILTER_GYRO 5 //ignored if filtration is not performed

//#define SETTINGS_SENSFUS_FILTER_MEDIAN //if commented, average will be used
#define SETTINGS_SENSFUS_MEDIAN_POSITION 3 //ignored, if avergage is used

//#define SETTINGS_SENSFUS_SEND_FILTERED_ACC
//#define SETTINGS_SENSFUS_SEND_FILTERED_GYRO

//#define SETTINGS_SENSFUS_SEND_ROLL
//#define SETTINGS_SENSFUS_SEND_PITCH
//#define SETTINGS_SENSFUS_SEND_YAW

#define SETTINGS_SENSFUS_CALIBRATE_GYRO
#define SETTINGS_SENSFUS_CALIBRATE_ACC

#define SETTINGS_SENSFUS_CALIBRATION_CYCLES 200

//-----------------------Settings for controller--------------------------------

//#define SETTINGS_CONTROLLER_USE_PROPORTIONAL
#define SETTINGS_CONTROLLER_USE_INTEGRAL
//#define SETTINGS_CONTROLLER_USE_DIFFERENTIAL

#define SETTINGS_CONTROLLER_SEND_IMPACT

#define SETTINGS_CONTROLLER_INTEGRAL_LENGTH 20 //ignored if integral is not used

//-----------------------Settings for messenger---------------------------------

//#define SETTINGS_MSNR_1BYTE
//#define SETTINGS_MSNR_3BYTE
#define SETTINGS_MSNR_5BYTE

#endif /*SETTINGS_H*/