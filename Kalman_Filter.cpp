#include "Kalman_Filter.h"
<<<<<<< HEAD



Kalman_Filter::Kalman_Filter(double x_est, double y)
{
	A = 1;
	B = 0;
	H = 1;
	x_post = 0; // INACZEJ x0 = 0
	Pp = 0; // P0 = 0
	Pf = (1 - K) * Pp;
	K = Pp / (Pp + R);
	// Pp = Pk/k-1
	// Pf = Pk/k
	
	dt = (double(micros()) / 1000000;
}

double Kalman_Filter::update(double x_post, double y)
{

    x_est = x_post;
    P_p = P_f + Q;
    K = P_p / (P_p + R);
    x_est = x_post + K * (y - x_post);
    P_f = P_p * (1 - K);
    
    return x_est;
     
=======
#include <math.h>

#include "Arduino.h"

KalmanFilter::KalmanFilter()
{
    // The state is a scalar quantity
    
	A = 1;
	B = 0;
	H = 1;
	
	// algorithm initialization
	
	x_pos = 0.; // x0 = 0
	P_p = 0; // P0 = 0
	
	K = (P_p != 0) ? K = P_p / (P_p + R) : 0;
	P_f = (1 - K) * P_p;
	
	// std of y
    sigma_y = 1;
    R = pow(sigma_y, 2);
    Q = 0.1;

	// Pf = P(k/k) FIRST
	// Pp = P(k/k-1) NEXT

}

KalmanFilter::~KalmanFilter()
{
}


double KalmanFilter::update(byte y)
{
    
    //prediction
    x_pos = x_est;
    P_p = P_f + Q;
    
    // filtration
    K = P_p / (P_p + R);
    x_est = x_pos + K * (y - x_pos);
    P_f = P_p * (1 - K);
    
    // estimated value
    return x_est;
}
>>>>>>> af49c874aeacf75264dadcd7a72fdfefaff484d8
