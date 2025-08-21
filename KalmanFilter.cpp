#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(float q, float r, float initialP, float initialX)
    : Q(q), R(r), P(initialP), x(initialX) {}

void KalmanFilter::init(float initialX) {
    x = initialX;
    P = 1.0f;
}

void KalmanFilter::predict(float deltaEncoder) {
    x += deltaEncoder;
    P += Q;
}

void KalmanFilter::update(float z) {
    float K = P / (P + R);   // Kalman gain
    x = x + K * (z - x);     // corrected state
    P = (1 - K) * P;
}

float KalmanFilter::getState() const {
    return x;
}

void KalmanFilter::setR(float r) { R = r; }

void KalmanFilter::setQ(float q) { Q = q; }

