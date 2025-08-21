#pragma once

class KalmanFilter {
private:
    float Q;   // Process noise covariance (uncertainty in encoder prediction)
    float R;   // Measurement noise covariance (uncertainty in IMU)
    float P;   // Estimate uncertainty
    float x;   // Current state estimate (heading in degrees)

public:
    KalmanFilter(float q = 0.3f, float r = 6.0f, float initialP = 1.0f, float initialX = 0.0f);

    void init(float initialX);

    // Predict step (from encoders)
    void predict(float deltaEncoder);

    // Update step (from IMU)
    void update(float z);

    float getState() const;

    void setR(float r);
    void setQ(float q);
};
