#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

// Cau truc cho bo loc Kalman
typedef struct {
    float x_hat;  // Gia tri uoc tinh hien tai
    float P;      // Sai so uoc tinh
    float Q;      // On nhieu qua trinh (binh phuong)
    float R;      // On nhieu do luong (binh phuong)
} KalmanFilter;

// Ham bo loc thong thap
float LPF(float x, float CUTOFF, float SAMPLE_RATE);

// Ham bo loc thong cao
float HPF(float x, float CUTOFF, float SAMPLE_RATE);

// Khoi tao bo loc Kalman
void kalman_init(KalmanFilter* kf, float process_noise, float measurement_noise);

// Cap nhat bo loc Kalman voi gia tri do moi
float kalman_update(KalmanFilter* kf, float z);

#endif /* __FILTER_H */