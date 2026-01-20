#pragma once

// ============================================================================
// PID CONTROLLER - Optimized for real-time control
// ============================================================================

class PIDController {
public:
    static constexpr float DEFAULT_KP = 0.07f;
    static constexpr float DEFAULT_KI = 0.4f;
    static constexpr float DEFAULT_KD = 0.0f;
    static constexpr float DEFAULT_MAX_ACCEL = 20.0f;  // rad/s²
    static constexpr float DEFAULT_MIN_OUTPUT = -1.0f;
    static constexpr float DEFAULT_MAX_OUTPUT = 1.0f;
    static constexpr float DEFAULT_MAX_INTEGRAL = 7.0f;

    PIDController(float kp = DEFAULT_KP,
                  float ki = DEFAULT_KI,
                  float kd = DEFAULT_KD);

    void setMaxAccel(float max_accel);
    void setGains(float kp, float ki, float kd);
    void setLimits(float min_output, float max_output);
    void setMaxIntegral(float max_integral);
    void reset();
    
    float compute(float setpoint, float measurement, float dt);
    
private:
    float kp_, ki_, kd_;
    float min_output_ = DEFAULT_MIN_OUTPUT;
    float max_output_ = DEFAULT_MAX_OUTPUT;
    float max_integral_ = DEFAULT_MAX_INTEGRAL;
    float max_accel_ = 0.0f;  // 0 = disabled

    float integral_ = 0.0f;
    float prev_measurement_ = 0.0f;
    float ramped_setpoint_ = 0.0f;
};
