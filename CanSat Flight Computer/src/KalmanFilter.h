#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Arduino.h>

/**
 * 1D Kalman Filter for altitude estimation
 * State vector: [position, velocity]
 * Measurement: position (altitude from barometer)
 */
class KalmanFilter {
private:
  // State estimates
  float position; // Estimated altitude (m)
  float velocity; // Estimated vertical velocity (m/s)

  // Covariance matrix P (2x2)
  float P[2][2];

  // Process noise covariance Q
  float Q_position = 0.01; // Position process noise
  float Q_velocity = 0.1;  // Velocity process noise

  // Measurement noise covariance R
  float R_measure = 2.0; // Barometer measurement noise (m)

  bool initialized = false;

public:
  KalmanFilter() : position(0), velocity(0) {
    // Initialize covariance matrix
    P[0][0] = 100.0; // Initial position uncertainty
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 10.0; // Initial velocity uncertainty
  }

  /**
   * Update filter with new altitude measurement
   * @param measurement New altitude reading (m)
   * @param dt Time step since last update (s)
   * @return Filtered altitude estimate (m)
   */
  float updateEstimate(float measurement, float dt) {
    // First measurement - initialize state
    if (!initialized || dt <= 0 || dt > 1.0) {
      position = measurement;
      velocity = 0;
      initialized = true;
      return position;
    }

    // Prediction step
    // x_pred = F * x
    float position_pred = position + velocity * dt;
    float velocity_pred = velocity; // Constant velocity model

    // P_pred = F * P * F^T + Q
    float dt2 = dt * dt;
    float P00_pred = P[0][0] + 2 * dt * P[0][1] + dt2 * P[1][1] + Q_position;
    float P01_pred = P[0][1] + dt * P[1][1];
    float P10_pred = P[1][0] + dt * P[1][1];
    float P11_pred = P[1][1] + Q_velocity;

    // Update step
    // Innovation: y = z - H * x_pred
    float innovation = measurement - position_pred;

    // Innovation covariance: S = H * P_pred * H^T + R
    float S = P00_pred + R_measure;

    // Kalman gain: K = P_pred * H^T * S^(-1)
    float K0 = P00_pred / S;
    float K1 = P10_pred / S;

    // State update: x = x_pred + K * y
    position = position_pred + K0 * innovation;
    velocity = velocity_pred + K1 * innovation;

    // Covariance update: P = (I - K * H) * P_pred
    float temp = 1.0 - K0;
    P[0][0] = temp * P00_pred;
    P[0][1] = temp * P01_pred;
    P[1][0] = P10_pred - K1 * P00_pred;
    P[1][1] = P11_pred - K1 * P01_pred;

    return position;
  }

  /**
   * Get current velocity estimate
   * @return Estimated vertical velocity (m/s)
   */
  float getVelocity() const { return velocity; }

  /**
   * Reset filter to initial state
   */
  void reset() {
    position = 0;
    velocity = 0;
    P[0][0] = 100.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 10.0;
    initialized = false;
  }

  /**
   * Tune process noise (higher = trust measurements more)
   */
  void setProcessNoise(float pos_noise, float vel_noise) {
    Q_position = pos_noise;
    Q_velocity = vel_noise;
  }

  /**
   * Tune measurement noise (higher = trust model more)
   */
  void setMeasurementNoise(float measure_noise) { R_measure = measure_noise; }
};

#endif
