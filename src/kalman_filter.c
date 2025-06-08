// kalman_filter.c
#include "kalman_filter.h"

void kalman1d_init(Kalman1D *kf, float process_noise, float measurement_noise) {
    kf->estimate = 0.0f;
    kf->error_covariance = 1.0f;
    kf->process_noise = process_noise;
    kf->measurement_noise = measurement_noise;
}

float kalman1d_update(Kalman1D *kf, float measurement) {
    kf->error_covariance += kf->process_noise;
    float K = kf->error_covariance / (kf->error_covariance + kf->measurement_noise);
    kf->estimate += K * (measurement - kf->estimate);
    kf->error_covariance *= (1.0f - K);
    return kf->estimate;
}

void kalman3d_init(KalmanSensor3D *ks, float gyro_q, float gyro_r, float accel_q, float accel_r) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
        kalman1d_init(&ks->gyro[i], gyro_q, gyro_r);
        kalman1d_init(&ks->accel[i], accel_q, accel_r);
    }
}

void kalman3d_update(KalmanSensor3D *ks, float raw_gyro[3], float raw_accel[3], float filtered_gyro[3], float filtered_accel[3]) {
    for (int i = 0; i < AXIS_COUNT; ++i) {
        filtered_gyro[i] = kalman1d_update(&ks->gyro[i], raw_gyro[i]);
        filtered_accel[i] = kalman1d_update(&ks->accel[i], raw_accel[i]);
    }
}
