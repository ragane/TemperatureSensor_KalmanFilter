<<<<<<< HEAD
#ifndef Kalman_Filter
#define Kalman_Filter


class Kalman_Filter
{
  
  private:
    /* Definition of shortcuts
    * Q_proc - macierz kowariancji szumu procesowego
    * P_pred - macierz kowariancji bledu predykcji
    * P_filtr - macierz kowariancji bledu filtracji
    * K - wzmocnienie filtru
    * R_est - macierz kowariancji szumu pomiarowego
    * x_post - macierz xk/k-1 ocena stanu a priori przed pomiarem
    * x_est - macierz wyjscia Xk/k estymata stanu a priori po pomiarze
    * y - wyjscie
    */
    double Q, Pp, Pf, K, R;
    double xpost, x_est;
    double dt;
    byte y;
    
  public:
    Kalman_Filter();
    double update(byte * y);
   
}; 
#endif
=======
#ifndef KalmanFilter_h
#define KalmanFilter_h

#include "Arduino.h"

// Class for simple implementation Kalman filter for scalar case
class KalmanFilter
{
  private:
    /* Definition of shortcuts
    * Q_p - process noise covariance matrix
    * P_p - prediction error covariance matrix
    * P_f - filter error coviarnace matrix
    * K - filter gain
    * R - measurment noise covariance matrix
    * x_pos - a prior matrix (x(k/k-1))
    * x_est - output matrix (x(k/k))
    * y - input
    * sigma_y - standard deviation of input
    * A - transition matrix
    * B - input matrix
    * H - output matrix
    */
    
    int A,B,H; 
    double Q, P_p, P_f, K, R;
    double x_pos, sigma_y, y;
    
  public:
    KalmanFilter();
    
    double x_est; // output value
    double update(byte y); // // equation of KF model
    
    ~KalmanFilter();

  
}; 
#endif // __KalmanFilter_h
>>>>>>> af49c874aeacf75264dadcd7a72fdfefaff484d8
