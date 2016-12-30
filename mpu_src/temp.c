#include "../mpu_inc/temp.h"

/*
 * This function initialize the INS_Data structure using position data come from GPS
 * Also initialize the eular angle using the graviry and magnititude
 * Set the initial variance
*/
void Init_INS_Data( INS_Data *pINS_Data, double lat, double lon, double hae, double vx, double vy, double vz )
{
	int i;
		
	pINS_Data->position[0] = lat;
	pINS_Data->position[1] = lon;
	pINS_Data->position[2] = hae;
	
	pINS_Data->velocity[0] = vx;
	pINS_Data->velocity[1] = vy;
	pINS_Data->velocity[2] = vz;
	
	pINS_Data->eular_angle[1] = asin( pINS_Data->accel[0] / sqrt( pINS_Data->accel[0]*pINS_Data->accel[0]+pINS_Data->accel[1]*pINS_Data->accel[1]+pINS_Data->accel[2]*pINS_Data->accel[2] ) );
	pINS_Data->eular_angle[0] = asin( -pINS_Data->accel[1] / sqrt( pINS_Data->accel[0]*pINS_Data->accel[0]+pINS_Data->accel[1]*pINS_Data->accel[1]+pINS_Data->accel[2]*pINS_Data->accel[2] ) );
	pINS_Data->eular_angle[2] = 0;
	
	pINS_Data->gravity = sqrt( pINS_Data->accel[0]*pINS_Data->accel[0]+pINS_Data->accel[1]*pINS_Data->accel[1]+pINS_Data->accel[2]*pINS_Data->accel[2] );
	
	for( i = 0; i < STATE_NUM; i++ )
	{
		pINS_Data->state_vector[i] = 0;
		pINS_Data->variance_matrix[i][i] = STARTUP_VARIANCE;
	}
	
	return;
}

/*
 * Using the pre-processed accel and gryo data to fill the carrisponding value in INS_Data
*/
void Get_IMU_Measurement( INS_Data *pINS_Data )
{
	// The IMU measurement should be pre-processed - axis change, gravity remove, etc.

	pINS_Data->accel[0] = IMU_Data.accel1;
	pINS_Data->accel[1] = IMU_Data.accel2;
	pINS_Data->accel[2] = IMU_Data.accel3;
	
	pINS_Data->angle_vel[0] = IMU_Data.gryo1;
	pINS_Data->angle_vel[1] = IMU_Data.gryo2;
	pINS_Data->angle_vel[2] = IMU_Data.gryo3;
	
	return;
}

/*
 * The INS positioning module, the position axis is set to be llh
 * The velocity axis is set to be ned, also the others
*/
void INS_Position_Update( INS_Data *pINS_Data, double dt )
{
	double diff_x, diff_y, diff_z;	// differencial position
	double RM, RN;	// Earth
	
	double diff_vx, diff_vy, diff_vz;	// differencial velocity
	double temp;
	
	double diff_roll, diff_pitch, diff_yaw;	// differencial Eular Angle

	// Compute the 1st order differencial of position
	RM = EARTH_CONSTANT_A*(1-EARTH_CONSTANT_E_SQUARE) / sqrt(1-EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]))/sqrt(1-	EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]))/sqrt(1-EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]));
	RN = EARTH_CONSTANT_A / sqrt( 1-EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]) );
	diff_x = pINS_Data->velocity[0] / (RM+pINS_Data->position[2]);
	diff_y = pINS_Data->velocity[1] / (RN+pINS_Data->position[2]) / cos(pINS_Data->position[0]);
	diff_z = -1*pINS_Data->velocity[2];
	
	temp = 0.0;
	temp += cos(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[1])*(pINS_Data->accel[0]-pINS_Data->accum_accel_bias[0]);
	temp += (cos(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0]) - sin(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[0]))*(pINS_Data->accel[1]-pINS_Data->accum_accel_bias[1]);
	temp += (cos(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0]) + sin(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[1]))*(pINS_Data->accel[2]-pINS_Data->accum_accel_bias[2]);
	temp -= -1*(pINS_Data->velocity[0])/(RM+pINS_Data->position[2])*(pINS_Data->velocity[2]) + ((pINS_Data->velocity[1])*tan(pINS_Data->position[0])/(RN+pINS_Data->position[2]) + 2*WE*sin(pINS_Data->position[0]))*(pINS_Data->velocity[1]);
	diff_vx = temp;
	
	temp = 0.0;
	temp += sin(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[1])*(pINS_Data->accel[0]-pINS_Data->accum_accel_bias[0]);
	temp += (sin(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0])+cos(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[0]))*(pINS_Data->accel[1]-pINS_Data->accum_accel_bias[1]);
	temp += (sin(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0])-cos(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[0]))*(pINS_Data->accel[2]-pINS_Data->accum_accel_bias[2]);
	temp += (2*WE*cos(pINS_Data->position[0])+(pINS_Data->velocity[1])/(RN+pINS_Data->position[2]))*(pINS_Data->velocity[2]);
	temp += (2*WE*sin(pINS_Data->position[0])+(pINS_Data->velocity[1])*tan(pINS_Data->position[0])/(RN+(pINS_Data->position[2])))*(pINS_Data->velocity[0]);
	diff_vy = temp;
	
	temp = 0.0;
	temp += -1*sin(pINS_Data->eular_angle[1])*(pINS_Data->accel[0]-pINS_Data->accum_accel_bias[0]) + cos(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0])*(pINS_Data->accel[1]-pINS_Data->accum_accel_bias[1]) + cos(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0])*(pINS_Data->accel[2]-pINS_Data->accum_accel_bias[2]);
	temp -= (2*WE*cos(pINS_Data->position[0]) + (pINS_Data->velocity[1])/(RN+pINS_Data->position[2]))*(pINS_Data->velocity[1]);
	temp -= pINS_Data->velocity[0] / (RM+pINS_Data->position[2]) * (pINS_Data->velocity[0]);
	diff_vz = temp;
	
	/*
	// Code below assums little change in Eular angle
	diff_roll = (pINS_Data->angle_vel[0] - pINS_Data->accum_gryo_bias[0]) + sin(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1])*(pINS_Data->angle_vel[1] - pINS_Data->accum_gryo_bias[1]) + cos(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1])*(pINS_Data->angle_vel[2] - pINS_Data->accum_gryo_bias[2]);
	diff_pitch = cos(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[1] - pINS_Data->accum_gryo_bias[1]) - sin(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[2] - pINS_Data->accum_gryo_bias[2]);
	diff_yaw = cos(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[1] - pINS_Data->accum_gryo_bias[1]) + cos(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[2] - pINS_Data->accum_gryo_bias[2]);
	*/
	
	pINS_Data->position[0] += diff_x * dt;
	pINS_Data->position[1] += diff_y * dt;
	pINS_Data->position[2] += diff_z * dt;
	
	pINS_Data->velocity[0] += diff_vx * dt;
	pINS_Data->velocity[1] += diff_vy * dt;
	pINS_Data->velocity[2] += diff_vz * dt;
	
	Eular_Angle_Update( pINS_Data, dt );
	/*
	pINS_Data->eular_angle[0] += diff_roll * dt;
	pINS_Data->eular_angle[1] += diff_pitch * dt;
	pINS_Data->eular_angle[2] += diff_yaw * dt;
	*/
	
	return;
}

/*
 * This function is used to calculate the prediction matrix used in kalman filter
 * The prediction matrix is 1 order differencial, so we need to update the position at times
 * Here set the accel&gryo bias as constant with Gauss noise
*/
void Get_Kalman_Update_Matrix( double M[STATE_NUM][STATE_NUM], INS_Data *pINS_Data )
{
	int i, j;
	
	for( i = 0; i < STATE_NUM; i++ )
	{
		for( j = 0; j < STATE_NUM; j++ )
		{
			M[i][j] = 0;
		}
	}
	
	double RM, RN, GAMA, R;
	RM = EARTH_CONSTANT_A*(1-EARTH_CONSTANT_E_SQUARE) / sqrt(1-EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]))/sqrt(1-	EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]))/sqrt(1-EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]));
	RN = EARTH_CONSTANT_A / sqrt( 1-EARTH_CONSTANT_E_SQUARE*sin(pINS_Data->position[0])*sin(pINS_Data->position[0]) );
	R = sqrt(RM*RN);
	GAMA = R*R/(R+pINS_Data->position[2])/(R+pINS_Data->position[2]);
	// Frr
	M[0][2] = -1*( pINS_Data->velocity[0] ) / (RM+pINS_Data->position[2]) / (RM+pINS_Data->position[2]);
	M[1][0] = pINS_Data->velocity[1] * sin( pINS_Data->eular_angle[0] ) / (RN+pINS_Data->position[2]) / cos( pINS_Data->eular_angle[0] ) / cos( pINS_Data->eular_angle[0] );
	M[2][2] = -1*( pINS_Data->velocity[1] ) / (RN+pINS_Data->position[2]) / cos( pINS_Data->eular_angle[0] ) / cos( pINS_Data->eular_angle[0] );
	// Frv
	M[0][3] = 1 / ( RM+pINS_Data->position[2] );
	M[1][4] = 1 / ( RN+pINS_Data->position[2] ) / cos( pINS_Data->position[0] );
	M[2][5] = -1;
	// Fvr ----------------- Here need more knowedge of gama
	M[3][0] = -2*(pINS_Data->velocity[1])*WE*cos(pINS_Data->position[0]) - (pINS_Data->velocity[1])*(pINS_Data->velocity[1])/(RN+pINS_Data->position[2])/cos(pINS_Data->position[0])/cos(pINS_Data->position[0]);
	M[3][2] = (pINS_Data->velocity[1])*(pINS_Data->velocity[1])*tan(pINS_Data->position[0])/(RN+pINS_Data->position[0])/(RN+pINS_Data->position[0]) - (pINS_Data->velocity[1])*(pINS_Data->velocity[2])/(RM+pINS_Data->position[0])/(RM+pINS_Data->position[0]);
	M[4][0] = 2*WE*((pINS_Data->velocity[0])*cos(pINS_Data->position[0])-(pINS_Data->velocity[2])*sin(pINS_Data->position[0])) + (pINS_Data->velocity[0])*(pINS_Data->velocity[1])/(RN+pINS_Data->position[2])/cos(pINS_Data->position[0])/cos(pINS_Data->position[0]);
	M[4][2] = -1*(pINS_Data->velocity[1])*(pINS_Data->velocity[2]+(pINS_Data->velocity[0])*tan(pINS_Data->position[0]))/(RN+pINS_Data->position[2])/(RN+pINS_Data->position[2]);
	M[5][0] = 2*WE*(pINS_Data->velocity[1])*sin(pINS_Data->position[0]);
	M[5][2] = (pINS_Data->velocity[1])*(pINS_Data->velocity[1])/(RN+pINS_Data->position[2])/(RN+pINS_Data->position[2]) + (pINS_Data->velocity[0])*(pINS_Data->velocity[0])/(RM+pINS_Data->position[2])/(RM+pINS_Data->position[2]) - 2*GAMA/(R+pINS_Data->position[2]);
	// Fvv
	M[3][3] = (pINS_Data->velocity[2])/(RM+pINS_Data->position[2]);
	M[3][4] = -2*WE*sin(pINS_Data->position[0]) - 2*(pINS_Data->velocity[1])*tan(pINS_Data->position[0])/(RN+pINS_Data->position[2]);
	M[3][5] = (pINS_Data->velocity[0])/(RM+pINS_Data->position[2]);
	M[4][3] = 2*WE*sin(pINS_Data->position[0]) + (pINS_Data->velocity[1])*tan(pINS_Data->position[0])/(RN+pINS_Data->position[2]);
	M[4][4] = ((pINS_Data->velocity[2])+(pINS_Data->velocity[0])*tan(pINS_Data->position[0]))/(RN+pINS_Data->position[2]);
	M[4][5] = 2*WE*cos(pINS_Data->position[0]) + (pINS_Data->velocity[1])/(RN+pINS_Data->position[2]);
	M[5][3] = -2*(pINS_Data->velocity[0])/(RM+pINS_Data->position[2]);
	M[5][4] = -2*WE*cos(pINS_Data->position[0]) - 2*(pINS_Data->velocity[1])/(RN+pINS_Data->position[2]);
	// Fvf
	M[3][6] = cos(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[1]);
	M[3][7] = cos(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0]) - sin(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[0]);
	M[3][8] = cos(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0]) + sin(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[0]);
	M[4][6] = sin(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[1]);
	M[4][7] = sin(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0]) + cos(pINS_Data->eular_angle[2])*cos(pINS_Data->eular_angle[0]);
	M[4][8] = sin(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0]) - cos(pINS_Data->eular_angle[2])*sin(pINS_Data->eular_angle[0]);
	M[5][6] = -1*sin(pINS_Data->eular_angle[1]);
	M[5][7] = cos(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0]);
	M[5][8] = cos(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0]);
	
	for( i = 6; i < 9; i++ )
		M[i][i] = 0.05;
	
	return;
}

/*
 * The custom kalman filter's prediction step, including the state vector update and the variance calculate
*/
void Kalman_Filter_Update( INS_Data *pINS_Data, double dt )
{
	double temp_matrix[STATE_NUM][STATE_NUM];
	double temp_matrix2[STATE_NUM][STATE_NUM];
	double M[STATE_NUM][STATE_NUM];
	double temp_vector[STATE_NUM];
	int i,j;
	
	Get_Kalman_Update_Matrix( M, pINS_Data );

	for( i = 0; i < STATE_NUM; i++ )
	{
		for( j = 0; j < STATE_NUM; j++ )
		{
			if( i == j )
				M[i][j] = 1 + M[i][j]*dt;
			else
				M[i][j] *= dt;
		}

	}

	Matrix_Dot_Vector( temp_vector, M, pINS_Data->state_vector );
	
	for( i = 0; i < STATE_NUM; i++ )
	{
		pINS_Data->state_vector[i] = temp_vector[i];
	}
	
	Matrix_Dot_Matrix( temp_matrix, M, pINS_Data->variance_matrix );
	
	for( i = 0; i < STATE_NUM; i++ )
		for( j = 0; j < STATE_NUM; j++ )
			pINS_Data->variance_matrix[i][j] = temp_matrix[i][j];
	
	Matrix_Trans( temp_matrix, M );
	
	Matrix_Dot_Matrix( temp_matrix2, pINS_Data->variance_matrix, temp_matrix );
	
	for( i = 0; i < STATE_NUM; i++ )
		for( j = 0; j < STATE_NUM; j++ )
			pINS_Data->variance_matrix[i][j] = temp_matrix2[i][j];

	for( i = 6; i < STATE_NUM; i++ )
		pINS_Data->variance_matrix[i][i] += BIAS_VARIANCE;
	
	return;	
}

/*
 * Using the observe value and its index to correct kalman filter, this is a scalar kalman filter
 * At the end update the positioning values in order to keep the 1' order differencial correct
 * But the variance should not be changed
*/
void Kalman_Filter_Feedback( INS_Data *pINS_Data, double ob, int index )
{
	double ob_variance = pINS_Data->variance_matrix[index][index] + OBSERVE_VARIANCE;
	double temp_vector[STATE_NUM];
	double diff = ob - pINS_Data->state_vector[index];
	int i, j;
/*
	for( i = 0; i < STATE_NUM; i++ )
	{
		for( j = 0; j < STATE_NUM; j++ )
		{
			printf( "%f ", pINS_Data->variance_matrix[i][j] );
		}
		print( "\n\r" );
	}
*/
	for( i = 0; i < STATE_NUM; i++ )
	{
		temp_vector[i] = pINS_Data->variance_matrix[i][index];
		pINS_Data->state_vector[i] += temp_vector[i] / ob_variance * diff;
	}
	
	for( i = 0; i < STATE_NUM; i++ )
	{
		for( j = 0; j < STATE_NUM; j++ )
		{
			pINS_Data->variance_matrix[i][j] -= temp_vector[i]*temp_vector[j]/ob_variance;
		}
	}
	
	for( i = 0; i < 3; i++ )
	{
		pINS_Data->position[i] -= pINS_Data->state_vector[i];
		pINS_Data->velocity[i] -= pINS_Data->state_vector[3+i];
		pINS_Data->accum_accel_bias[i] += pINS_Data->state_vector[6+i];
	}
	
	for( i = 0; i < STATE_NUM; i++ )
		pINS_Data->state_vector[i] = 0;
	
	return;
}

void Matrix_Dot_Matrix( double Mout[STATE_NUM][STATE_NUM], double M1[STATE_NUM][STATE_NUM], double M2[STATE_NUM][STATE_NUM] )
{
	int i, j, k;
	double sum;
	
	for( j = 0; j < STATE_NUM; j++ )
	{
		for( i = 0; i < STATE_NUM; i++ )
		{
			sum = 0;
			for( k = 0; k < STATE_NUM; k++ )
				sum += M1[i][k]*M2[k][j];
			Mout[i][j] = sum;
		}
	}
	
	return;
}

void Matrix_Dot_Vector( double Vout[STATE_NUM], double Min[STATE_NUM][STATE_NUM], double Vin[STATE_NUM] )
{
	int i, j;
	double sum;
	
	for( i = 0; i < STATE_NUM; i++ )
	{
		sum = 0;
		for( j = 0; j < STATE_NUM; j++ )
			sum += Min[i][j]*Vin[j];
		Vout[i] = sum;
	}
	
	return;
}

void Matrix_Trans( double Mout[STATE_NUM][STATE_NUM], double Min[STATE_NUM][STATE_NUM] )
{
	int i, j;
	for( i = 0; i < STATE_NUM; i++ )
		for( j = 0; j < STATE_NUM; j++ )
			Mout[j][i] = Min[i][j];
		
	return;
}

/*
 * This function change the IMU axis into aimed axis
 * Maybe changed according to the real axis of MPU9250
 * All the computation later should begin with this function
*/
void Change_Axis()
{
	IMU_Data.accel1 = 1*IMU_Data.accel1;
	IMU_Data.accel2 = -1*IMU_Data.accel2;
	IMU_Data.accel3 = -1*IMU_Data.accel3;
	
	IMU_Data.gryo1 = 3.03*IMU_Data.gryo1;
	IMU_Data.gryo2 = -3.03*IMU_Data.gryo2;
	IMU_Data.gryo3 = -3.03*IMU_Data.gryo3;
	
	Magnititude_Instance.hy = -1*Magnititude_Instance.hy;
	
	return;
}

/*
 * Every time update the position, this function should be called
 * Assume the position information is already initialized
*/
void Remove_Gravity(INS_Data *pINS_Data)
{
	double minus_x, minus_y, minus_z;
	
	minus_x = sin(pINS_Data->eular_angle[1]) * pINS_Data->gravity;
	IMU_Data.accel1 -= minus_x;
	
	minus_y = sin(pINS_Data->eular_angle[0]) * pINS_Data->gravity;
	IMU_Data.accel2 += minus_y;
	

	minus_z = sqrt( (pINS_Data->gravity)*(pINS_Data->gravity) - minus_x*minus_x - minus_y*minus_y );
	//printf( "%f, %f\n\r", IMU_Data.accel3, minus_z );


	IMU_Data.accel3 += minus_z;
	
	return;
}

Useful_Data Get_Useful_Data( INS_Data *pINS_Data )
{
	Useful_Data temp;
	int i;
	for( i = 0; i < 3; i++ )
	{
		temp.accum_accel_bias[i] = pINS_Data->accum_accel_bias[i];
	}
	return temp;
}

void Eular_Angle_Update( INS_Data *pINS_Data, double dt )
{
	double diff_roll, diff_pitch, diff_yaw, temp;
	
	diff_roll = (pINS_Data->angle_vel[0]) + sin(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1])*(pINS_Data->angle_vel[1]) + cos(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1])*(pINS_Data->angle_vel[2]);
	diff_pitch = cos(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[1]) - sin(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[2]);
	diff_yaw = cos(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[1]) + cos(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0])*(pINS_Data->angle_vel[2]);

	pINS_Data->eular_angle[0] += diff_roll * dt;
	pINS_Data->eular_angle[1] += diff_pitch * dt;
	pINS_Data->eular_angle[2] += diff_yaw * dt;
	
	temp = ( 1 + sin(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1]) + cos(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0])*tan(pINS_Data->eular_angle[1]) ) * dt * dt;
	pINS_Data->variance_eular_angle[0] += temp * GRYO_MEASURE_VARIANCE;
	temp = ( cos(pINS_Data->eular_angle[0])*cos(pINS_Data->eular_angle[0]) + sin(pINS_Data->eular_angle[0])*sin(pINS_Data->eular_angle[0]) ) * dt * dt;
	pINS_Data->variance_eular_angle[1] += temp * GRYO_MEASURE_VARIANCE;
	temp = ( cos(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0])*cos(pINS_Data->eular_angle[1])*sin(pINS_Data->eular_angle[0]) + cos(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0])*cos(pINS_Data->eular_angle[1])*cos(pINS_Data->eular_angle[0]) ) * dt * dt;
	pINS_Data->variance_eular_angle[2] += temp * GRYO_MEASURE_VARIANCE;
	
	return;
}

void Eular_Angle_Feedback( INS_Data *pINS_Data, double angle, int index )
{
	double diff, temp;
	
	diff = angle - pINS_Data->eular_angle[index];
	temp = pINS_Data->variance_eular_angle[index] + EULAR_OBSERVE_VARIANCE;
	
	pINS_Data->eular_angle[index] += diff * (pINS_Data->variance_eular_angle[index]) / temp;
	pINS_Data->variance_eular_angle[index] -= (pINS_Data->variance_eular_angle[index])*(pINS_Data->variance_eular_angle[index])/temp;
	
	return;
	
}

























