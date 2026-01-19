// ============================================================================
// PID CONTROLLER IMPLEMENTATION
// ============================================================================

#include "pid.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setLimits(float min_output, float max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::setMaxIntegral(float max_integral) {
    max_integral_ = max_integral;
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_measurement_ = 0.0f;
    ramped_setpoint_ = 0.0f;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // ========== ACCELERATION LIMITING ==========
    float target = setpoint;
    if (max_accel_ > 0.0f) {
        const float max_change = max_accel_ * dt;
        const float delta = setpoint - ramped_setpoint_;
        
        if (delta > max_change) {
            ramped_setpoint_ += max_change;
        } else if (delta < -max_change) {
            ramped_setpoint_ -= max_change;
        } else {
            ramped_setpoint_ = setpoint;
        }
        target = ramped_setpoint_;
    }
    
    // ========== STANDARD PID ==========
    const float error = target - measurement;
    
    // Proportional
    const float p_term = kp_ * error;
    
    // Integral with anti-windup
    integral_ += error * dt;
    integral_ = constrain(integral_, -max_integral_, max_integral_);
    const float i_term = ki_ * integral_;
    
    // Derivative
    const float derivative = (measurement - prev_measurement_) / dt; // Note: measurement-based derivative
    const float d_term = kd_ * derivative;
    prev_measurement_ = measurement;
    
    return constrain(p_term + i_term + d_term, min_output_, max_output_);
}