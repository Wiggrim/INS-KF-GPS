// Author : WYM

/*
 * This file defines the communication with MPU9250 through I2C Bus
 * Problem is that the data pattern send by ZC706 is unknown and needed
 * to be tested
*/

#include "../mpu_inc/MPU9250_Drivers.h"

XIicPs IicPs;		/* The instance of the IIC device. */
IMU_DATA IMU_Data;
Attitude Attitude_Instance;
Magnititude Magnititude_Instance;
u32 count;
/*
 This function initialize the I2C device connected to MPU9250
 by setting its CLKrate and interrupt
*/
int I2C_MPU9250_Init(void)
{
	int Status;
	XIicPs_Config *Config;

	// Look up configure files via IIC device index
	Config = XIicPs_LookupConfig( IIC_DEVICE_ID );
	
	// Initialize IIC device via configure files
	Status = XIicPs_CfgInitialize( &IicPs, Config, Config->BaseAddress );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	Status = XIicPs_SelfTest( &IicPs );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	// Set I2C CLK rate to 400KHz
	XIicPs_SetSClk( &IicPs , IIC_SCLK_RATE );
	
	// MPU default device ID should be 0x71
	u8 MPUDeviceId;
	
	Status = I2C_MPU9250_Read( WHO_AM_I, &MPUDeviceId, 1 );
	if( Status != XST_SUCCESS || MPUDeviceId != 0x73 )
	{
		xil_printf("Get device id is : %u", MPUDeviceId);
		print("unmatched device id\n\r");
		return XST_FAILURE;
	}
	
	// Reset the MPU
	Status = I2C_MPU9250_Write( PWR_MGMT_1, 0x80 );
	if( Status != XST_SUCCESS )
	{
		print("PWR write error\n\r");
		return XST_FAILURE;
	}
	
	Delay();
	
	// Set bypass mode to access AK8963
	Status = I2C_MPU9250_Write( BYPASS_CONFIG, 0x02 );
	if( Status != XST_SUCCESS )
	{
		print("Bypass set error\n\r");
		return XST_FAILURE;
	}
	
	int i;
	for( i = 0; i < 60; i++ )
	Delay();
	
	
	u8 AK8963DeviceId;
	
	Status = I2C_AK8963_Read( WIA, &AK8963DeviceId );
	if( Status != XST_SUCCESS || AK8963DeviceId != 72 )
	{
		xil_printf( "Read AK8963 failed\n\r" );
		return XST_FAILURE;
	}
	//xil_printf( "AK8963 : %d\n\r", AK8963DeviceId );
	
	Status = I2C_AK8963_Write( CNTL2, 0x01 );
	if( Status != XST_SUCCESS )
	{
		print("AK8963 reset write error\n\r");
		return XST_FAILURE;
	}
	
	Status = I2C_AK8963_Write( CNTL1, 0x16 );
	if( Status != XST_SUCCESS )
	{
		print("AK8963 mode set write error\n\r");
		return XST_FAILURE;
	}
	
	// sample rate = 1000/(1+7) = 125Hz
	Status = I2C_MPU9250_Write( SMPLRT_DIV, 0x07 );
	if( Status != XST_SUCCESS )
	{
		print("Gryo sample rate write error\n\r");
		return XST_FAILURE;
	}
	
	// Gryo: Filter band 5Hz, SampleRate 1KHz, delay 33ms
	Status = I2C_MPU9250_Write( CONFIG, 0x06 );
	if( Status != XST_SUCCESS )
	{
		print("Filter band write error\n\r");
		return XST_FAILURE;
	}
	
	// Accel 16g
	Status = I2C_MPU9250_Write( ACCEL_CONFIG, 0x00 );
	if( Status != XST_SUCCESS )
	{
		print("Accel write error\n\r");
		return XST_FAILURE;
	}
	
	// Accel bandwidth 460Hz, SampleRate 1KHz, delay 2ms
	Status = I2C_MPU9250_Write( ACCEL_CONFIG2, 0x00 );
	if( Status != XST_SUCCESS )
	{
		print("Accel sample rate write error\n\r");
		return XST_FAILURE;
	}
	/*
	Status = I2C_MPU9250_Write( GYRO_CONFIG, 0x00 );
	if( Status != XST_SUCCESS )
	{
		print("Gyro range write error\n\r");
		return XST_FAILURE;
	}
	*/

	return XST_SUCCESS;
}

/*
 This function write one byte to MPU 's register, it totally send two bytes
 the first one is the register address, and the second is data to write
*/
int I2C_MPU9250_Write( u8 AimedReg, u8 Data )
{
	u8 Buffer[2];
	Buffer[0] = AimedReg;
	Buffer[1] = Data;
	
	int Status;
	
	Status = XIicPs_MasterSendPolled( &IicPs, Buffer, 2, MPU9250_ADDR );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	while( XIicPs_BusIsBusy( &IicPs ) );
	
	return XST_SUCCESS;
}

int I2C_AK8963_Write( u8 AimedReg, u8 Data )
{
	u8 Buffer[2];
	Buffer[0] = AimedReg;
	Buffer[1] = Data;
	
	int Status;
	Status = XIicPs_MasterSendPolled( &IicPs, Buffer, 2, AK8963_ADDR );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	while( XIicPs_BusIsBusy( &IicPs ) );
	
	return XST_SUCCESS;
}

int I2C_AK8963_Read( u8 AimedReg, u8 *DataBuffer )
{
	int Status;
	
	Status = XIicPs_MasterSendPolled( &IicPs, &AimedReg, 1, AK8963_ADDR );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	while( XIicPs_BusIsBusy( &IicPs ) );
	
	Status = XIicPs_MasterRecvPolled( &IicPs, DataBuffer, 1, AK8963_ADDR );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	while( XIicPs_BusIsBusy( &IicPs ) );
	
	return XST_SUCCESS;
	
}

/*
 This function read several bytes from MPU, the process start with send one address byte,
 then read bytes from MPU
*/
int I2C_MPU9250_Read( u8 AimedReg, u8 *DataBuffer, int DataCount )
{
	int Status;
	
	// Send the aimed register's address to MPU9250
	Status = XIicPs_MasterSendPolled( &IicPs, &AimedReg, 1, MPU9250_ADDR );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	// Wait until the I2C Bus line is idle
	while( XIicPs_BusIsBusy( &IicPs ) );
	
	// Read DataCount byte from MPU9250
	Status = XIicPs_MasterRecvPolled( &IicPs, DataBuffer, DataCount, MPU9250_ADDR );
	if( Status != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	
	while( XIicPs_BusIsBusy( &IicPs ) );
	
	return XST_SUCCESS;
}

/*
 * Fill the structure IMU_Data with data read from MPU9250
*/
int MPU9250_Get_Data()
{
	u8 temp[14];
	
	double temp2;
	int Status;
	// Read 14bytes from MPU9250, which is the accel, gryo, temperature data of 16bit length
	Status = I2C_MPU9250_Read( ACCEL_XOUT_H, temp, 14 );
	if( XST_SUCCESS != Status )
	{
		return XST_FAILURE;
	}
	
	//xil_printf( "u8 : %d\n\r", temp[0] );
	//xil_printf( "s16 : %d\n\r", (s16)temp[0] );
	
	temp2 = (temp[0]>127)?((temp[0]-256)*256+temp[1]):(temp[0]*256+temp[1]);
	temp2 = temp2 / 32768 * 20;
	IMU_Data.accel1 = temp2;
	temp2 = (temp[2]>127)?((temp[2]-256)*256+temp[3]):(temp[2]*256+temp[3]);
	temp2 = temp2 / 32768 * 20;
	IMU_Data.accel2 = temp2;	
	temp2 = (temp[4]>127)?((temp[4]-256)*256+temp[5]):(temp[4]*256+temp[5]);
	temp2 = temp2 / 32768 * 20;
	IMU_Data.accel3 = temp2;
		
	IMU_Data.temperature = 21+(((s16)temp[6]<<8) | temp[7]) / 334;
	
	temp2 = (temp[8]>127)?((temp[8]-256)*256+temp[9]):(temp[8]*256+temp[9]);
	temp2 = temp2 / 32768 * 250 / 180 * 3.14;
	IMU_Data.gryo1 = temp2 - 0.0035;
	temp2 = (temp[10]>127)?((temp[10]-256)*256+temp[11]):(temp[10]*256+temp[11]);
	temp2 = temp2 / 32768 * 250 / 180 * 3.14;
	IMU_Data.gryo2 = temp2 - 0.0134;
	temp2 = (temp[12]>127)?((temp[12]-256)*256+temp[13]):(temp[12]*256+temp[13]);
	temp2 = temp2 / 32768 * 250 / 180 * 3.14;
	IMU_Data.gryo3 = temp2 + 0.0098;
	
	//printf( "%f, %f, %f\n\r", IMU_Data.gryo1, IMU_Data.gryo2, IMU_Data.gryo3 );

	return XST_SUCCESS;
}

int AK8963_Get_Data()
{
	int Status;
/*
	u8 mag_status;
	while( mag_status == 0 )
	{
		Status = I2C_AK8963_Read( STATU1, &mag_status );
		if( XST_SUCCESS != Status )
		{
			printf( "AK8963 read data error\n\r" );
			return XST_FAILURE;
		}
	}
*/
	u8 temp[6];
	double temp2;
	int i;
	
	for( i=0; i<6; i++ )
	{
		Status = I2C_AK8963_Read( 3+i, temp+i );
		if( XST_SUCCESS != Status )
		{
			xil_printf( "AK8963 read data error\n\r" );
			return XST_FAILURE;
		}
	}
	
	temp2 = (temp[0]>127)?((temp[0]-256)*256+temp[1]):(temp[0]*256+temp[1]);
	temp2 = temp2 / 32768 * 4912;
	Magnititude_Instance.hx = temp2;
	temp2 = (temp[2]>127)?((temp[2]-256)*256+temp[3]):(temp[2]*256+temp[3]);
	temp2 = temp2 / 32768 * 4912;
	Magnititude_Instance.hy = temp2;	
	temp2 = (temp[4]>127)?((temp[4]-256)*256+temp[5]):(temp[4]*256+temp[5]);
	temp2 = temp2 / 32768 * 4912;
	Magnititude_Instance.hz = temp2;
	
	return XST_SUCCESS;
	
}


void Delay()
{
	count = 10000;
	while((count--)>=1);
}

/*
int I2C_MPU9250_Init_Attitude(void)
{
	int Status;

	Status = MPU9250_Get_Data();
	Delay();
	Delay();
	Status = MPU9250_Get_Data();
	if(XST_SUCCESS != Status)
	{
		return XST_FAILURE;
	}

	Position_Instance.yaw = atan( IMU_Data.accel2 / sqrt( IMU_Data.accel1*IMU_Data.accel1+IMU_Data.accel3*IMU_Data.accel3 ) );
	Position_Instance.pitch = atan( IMU_Data.accel1 / sqrt( IMU_Data.accel2*IMU_Data.accel2+IMU_Data.accel3*IMU_Data.accel3 ) );
	//Position_Instance.roll = atan( sqrt( IMU_Data.accel1*IMU_Data.accel1+IMU_Data.accel2*IMU_Data.accel2 )/IMU_Data.accel3 );
	Position_Instance.roll = 0.0;

	//xil_printf( "fai : %5f \r\n", 0.123);
	Gravity = sqrt(IMU_Data.accel1*IMU_Data.accel1 + IMU_Data.accel2*IMU_Data.accel2 + IMU_Data.accel3*IMU_Data.accel3);

	return XST_SUCCESS;
}
*/













