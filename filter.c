#include "filter.h"
#include <math.h>

float LPF(float x, float CUTOFF, float SAMPLE_RATE)
{
    float RC, dt, alpha, y;
    static float ylast = 0;
    RC = 1.0/(CUTOFF*2*3.14);
    dt = 1.0/SAMPLE_RATE;
    alpha = dt/(RC+dt);
    y = ylast + alpha * (x - ylast); 
    ylast = y;
    return y;
}

float HPF(float x, float CUTOFF, float SAMPLE_RATE)
{
    float RC = 1.0/(CUTOFF*2*3.14);
    float dt = 1.0/SAMPLE_RATE;
    float alpha = RC/(RC+dt);
    float y;
    static float xlast = 0, ylast = 0;
    y = alpha * (ylast + x - xlast); 
    ylast = y;
    xlast = x;
    return y;
}

void kalman_init(KalmanFilter* kf, float process_noise, float measurement_noise)
{
    kf->x_hat = 0;
    kf->P = 1;
    kf->Q = process_noise * process_noise;
    kf->R = measurement_noise * measurement_noise;
}

float kalman_update(KalmanFilter* kf, float z)
{
    float P_, K;
    
    // Buoc du doan
    P_ = kf->P + kf->Q;
    
    // Buoc cap nhat
    K = P_ / (P_ + kf->R);
    kf->x_hat = kf->x_hat + K * (z - kf->x_hat);
    kf->P = (1 - K) * P_;
    
    return kf->x_hat;
}