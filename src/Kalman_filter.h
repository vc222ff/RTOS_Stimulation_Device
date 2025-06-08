// kalman_filter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#define AXIS_COUNT 3

typedef struct {
    float estimate;
    float error_covariance;
    float process_noise;
    float measurement_noise;
} Kalman1D;

typedef struct {
    Kalman1D gyro[AXIS_COUNT];
    Kalman1D accel[AXIS_COUNT];
} KalmanSensor3D;

void kalman1d_init(Kalman1D *kf, float process_noise, float measurement_noise);
float kalman1d_update(Kalman1D *kf, float measurement);
void kalman3d_init(KalmanSensor3D *ks, float gyro_q, float gyro_r, float accel_q, float accel_r);
void kalman3d_update(KalmanSensor3D *ks, float raw_gyro[3], float raw_accel[3], float filtered_gyro[3], float filtered_accel[3]);

#endif
