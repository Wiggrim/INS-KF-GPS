#include <math.h>
#include <stdio.h>
#include "MPU9250_Drivers.h"

#define STARTUP_VARIANCE 1
#define STATE_NUM 9
#define OB_NUM 6
#define STATE_NUM_ROTATE 3
#define BIAS_VARIANCE 0.02
#define OBSERVE_VARIANCE 10
#define EULAR_STARTUP_VARIANCE 0.01
#define GRYO_MEASURE_VARIANCE 0.2
#define EULAR_OBSERVE_VARIANCE 0.001
#define EARTH_CONSTANT_A 6378137.0
#define EARTH_CONSTANT_B 6356752.0
#define EARTH_CONSTANT_E_SQUARE 0.00669438
#define WE 7.29211E-5
#define GRAVITY 9.83

#define INS_SUCCESS 0
#define INS_FAILURE 1
#define INS_ALARM 2

typedef struct
{
	double position[3];
	double velocity[3];
	double accel[3];
	double accum_accel_bias[3];
	
	double eular_angle[3];
	double angle_vel[3];
	
	double state_vector[STATE_NUM];
	double variance_matrix[STATE_NUM][STATE_NUM];
	
	double variance_eular_angle[3];
	
	double gravity;
}	INS_Data;

typedef struct
{
	double accum_accel_bias[3];
} Useful_Data;

void Init_INS_Data( INS_Data *pINS_Data, double lat, double lon, double hae, double vx, double vy, double vz );
void Get_IMU_Measurement( INS_Data *pINS_Data );
void INS_Position_Update( INS_Data *pINS_Data, double dt );
void Kalman_Filter_Update( INS_Data *pINS_Data, double dt );
void Kalman_Filter_Feedback( INS_Data *pINS_Data, double lat, int index );
int Kalman_Filter_Feedback_Batched( INS_Data *pINS_Data, double ob[] );
Useful_Data Get_Useful_Data( INS_Data *pINS_Data );

void Get_Kalman_Update_Matrix( double M[STATE_NUM][STATE_NUM], INS_Data *pINS_Data );

void Matrix_Dot_Vector( double Vout[STATE_NUM], double M[STATE_NUM][STATE_NUM], double Vin[STATE_NUM], int r1, int c1 );
void Matrix_Dot_Matrix( double Mout[STATE_NUM][STATE_NUM], double M1[STATE_NUM][STATE_NUM], double M2[STATE_NUM][STATE_NUM], int, int, int );
void Matrix_Trans( double Mout[STATE_NUM][STATE_NUM], double Min[STATE_NUM][STATE_NUM] );
void Matrix_Inverse( double Mout[STATE_NUM][STATE_NUM], double Min[STATE_NUM][STATE_NUM], int );

void Change_Axis();
void Remove_Gravity();

void Eular_Angle_Update( INS_Data *pINS_Data, double dt );
void Eular_Angle_Feedback( INS_Data *pINS_Data, double angle, int index );
double getYaw( double, double, double, double, double );
