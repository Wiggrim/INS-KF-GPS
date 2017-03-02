#include "../mpu_inc/temp.h"

double getA( double arcs[STATE_NUM][STATE_NUM], int n )  
{  
    if(n==1)  
    {  
        return arcs[0][0];  
    }  
    double ans = 0;  
    double temp[STATE_NUM][STATE_NUM];  
    int i,j,k;  
    for(i=0;i<n;i++)  
    {  
        for(j=0;j<n-1;j++)  
        {  
            for(k=0;k<n-1;k++)  
            {  
                temp[j][k] = arcs[j+1][(k>=i)?k+1:k];  
                  
            }  
        }  
        double t = getA(temp,n-1);  
        if(i%2==0)  
        {  
            ans += arcs[0][i]*t;  
        }  
        else  
        {  
            ans -=  arcs[0][i]*t;  
        }  
    }  
    return ans;  
}  
void getAStart(double arcs[STATE_NUM][STATE_NUM],int n,double ans[STATE_NUM][STATE_NUM])
{  
    if(n==1)  
    {  
        ans[0][0] = 1;  
        return;  
    }  
    int i,j,k,t;  
    double temp[STATE_NUM][STATE_NUM];
    for(i=0;i<n;i++)  
    {  
        for(j=0;j<n;j++)  
        {  
            for(k=0;k<n-1;k++)  
            {  
                for(t=0;t<n-1;t++)  
                {  
                    temp[k][t] = arcs[k>=i?k+1:k][t>=j?t+1:t];  
                }  
            }  
  
          
            ans[j][i]  =  getA(temp,n-1);  
            if((i+j)%2 == 1)  
            {  
                ans[j][i] = - ans[j][i];  
            }  
        }  
    }  
}  
  
  
void Matrix_Inverse( double Mout[STATE_NUM][STATE_NUM], double Min[STATE_NUM][STATE_NUM], int n )
{
	int i,j;
	double a = getA( Min, n );
	getAStart( Min, n, Mout );
	for( i = 0; i < n; i++ )
	{
		for( j = 0; j < n; j++ )
		{
			Mout[i][j] = Mout[i][j] / a;
		}
	}
	
	return;
}
  
void Matrix_Dot_Matrix( double Mout[STATE_NUM][STATE_NUM], double M1[STATE_NUM][STATE_NUM], double M2[STATE_NUM][STATE_NUM], int r1, int c1, int c2 )
{
	int i, j, k;
	double sum;
	
	for( j = 0; j < c2; j++ )
	{
		for( i = 0; i < r1; i++ )
		{
			sum = 0;
			for( k = 0; k < c1; k++ )
				sum += M1[i][k]*M2[k][j];
			Mout[i][j] = sum;
		}
	}
	
	return;
}

void Matrix_Dot_Vector( double Vout[STATE_NUM], double Min[STATE_NUM][STATE_NUM], double Vin[STATE_NUM], int r, int c )
{
	int i, j;
	double sum;
	
	for( i = 0; i < r; i++ )
	{
		sum = 0;
		for( j = 0; j < c; j++ )
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
