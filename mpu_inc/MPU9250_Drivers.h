// Author : WYM
#ifndef _MPU9250_DRIVERS
#define _MPU9250_DRIVERS
#include "xparameters.h"
#include "xiicps.h"
#include "xil_printf.h"
#include "math.h"
//#include "IMU_Data_Process.h"
// other include file needed here

#define IIC_DEVICE_ID	XPAR_XIICPS_1_DEVICE_ID
#define IIC_SCLK_RATE		400000
#define MPU9250_ADDR	0x68	// The AD0 pin should be connect to VCC
#define SMPLRT_DIV	0X19 // Gryo sample reg
#define CONFIG	0X1A 
#define BYPASS_CONFIG 0x37
#define GYRO_CONFIG	0X1B 
#define ACCEL_CONFIG	0X1C 
#define ACCEL_CONFIG2	0X1D 
#define PWR_MGMT_1	0X6B
#define PWR_MGMT_2	0X6C 
#define WHO_AM_I	0X75 
#define USER_CTRL	0X6A 
#define ACCEL_XOUT_H	0X3B  // The data register, their address is continuous 
#define ACCEL_XOUT_L	0X3C
#define ACCEL_YOUT_H	0X3D
#define ACCEL_YOUT_L	0X3E
#define ACCEL_ZOUT_H	0X3F
#define ACCEL_ZOUT_L	0X40
#define TEMP_OUT_H	0X41  
#define TEMP_OUT_L	0X42
#define GYRO_XOUT_H	0X43  
#define GYRO_XOUT_L	0X44
#define GYRO_YOUT_H	0X45
#define GYRO_YOUT_L	0X46
#define GYRO_ZOUT_H	0X47
#define GYRO_ZOUT_L	0X48

#define AK8963_ADDR 0x0C
#define WIA 0X00
#define CNTL1 0x0A
#define CNTL2 0x0B
#define STATU1 0x02


typedef struct
{
	double accel1;
	double accel2;
	double accel3;

	double temperature;

	double gryo1;
	double gryo2;
	double gryo3;
}	IMU_DATA;

typedef struct
{
	double rou;
	double fai;
	double gama;
} Attitude;

typedef struct
{
	double hx;
	double hy;
	double hz;
} Magnititude;

extern IMU_DATA IMU_Data;
//extern Attitude Attidude_Instance;
extern Magnititude Magnititude_Instance;

int I2C_MPU9250_Init(void);
int I2C_MPU9250_Write( u8 AimedReg, u8 Data );
int I2C_MPU9250_Read( u8 AimedReg, u8 *DataBuffer, int DataCount );
int MPU9250_Get_Data();
//int I2C_MPU9250_Init_Attitude(void);
void Delay();

int AK8963_Get_Data();

int I2C_AK8963_Read( u8, u8* );
int I2C_AK8963_Write( u8 AimedReg, u8 Data );

#endif
