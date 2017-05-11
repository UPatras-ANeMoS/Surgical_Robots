#include "dynamixel_sdk/odefun.h"
#include <math.h>
#include <stdio.h>

void odefun(const double X[2] , double dX[2], double U , double t, double x_lim){
    if(X[0]<x_lim){
        dX[0] = X[1];
        if(X[1]<=40)	dX[1] = U;
        else	dX[1]=0.0;
        //printf("%f \t %f \t %f\n", X[0],X[1],t);
	}
	else{
        //printf("Fucked \t") ;
		//dX[0]=X[0];
		//dX[1]=0;
        dX[0]=0.0;
        dX[1]=0.0;
        //printf("%f \t %f \t %f\n", X[0],X[1],t);
	}
}
