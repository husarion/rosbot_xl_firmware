/**
 * @file motors.h
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2021-12-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <bsp.h>

#define M1_ENC_TIM      TIM1
#define M1_ENC_A        PE9
#define M1_ENC_B        PE11
#define M1_PWM_TIM      TIM10
#define M1_PWM_PIN      PF6
#define M1_PWM_TIM_CH   1
#define M1A_IN          PE12
#define M1B_IN          PE13
#define M1_DEFAULT_DIR  -1           // 1 (CW) or -1 (CCW)

#define M2_ENC_TIM      TIM2
#define M2_ENC_A        PA15
#define M2_ENC_B        PB3
#define M2_PWM_TIM      TIM11
#define M2_PWM_PIN      PF7
#define M2_PWM_TIM_CH   1
#define M2A_IN          PG11
#define M2B_IN          PG12
#define M2_DEFAULT_DIR  1           // 1 (CW) or -1 (CCW)

#define M3_ENC_TIM      TIM3
#define M3_ENC_A        PC6
#define M3_ENC_B        PC7
#define M3_PWM_TIM      TIM13
#define M3_PWM_PIN      PF8
#define M3_PWM_TIM_CH   1
#define M3A_IN          PG5
#define M3B_IN          PG6
#define M3_DEFAULT_DIR  -1           // 1 (CW) or -1 (CCW)

#define M4_ENC_TIM      TIM4
#define M4_ENC_A        PD12
#define M4_ENC_B        PD13
#define M4_PWM_TIM      TIM14
#define M4_PWM_PIN      PF9
#define M4_PWM_TIM_CH   1
#define M4A_IN          PD10
#define M4B_IN          PD11
#define M4_DEFAULT_DIR  1          // 1 (CW) or -1 (CCW)


#define ILIM1           PE10
#define ILIM2           PG15
#define ILIM3           PG7
#define ILIM4           PD14

//MOTORS TIMEBASE TIMER
#define TIMEBASE_TIMER                      TIM6
#define TIMEBASE_TIMER_FREQ                 10000
#define TIMEBASE_TIMER_CLOCKSOURCE_FREQ     168000000
#define TIMEBASE_TIMER_PSC                  ((TIMEBASE_TIMER_CLOCKSOURCE_FREQ / TIMEBASE_TIMER_FREQ) / 2)
#define TIMEBASE_TIMER_OVERFLOW_VALUE       0xFFFF

//PID PARAMETERS
#define PID_FREQ                            100     //max 1000Hz
#define PID_DEFAULT_KP                      49      //KP * 1000
#define PID_DEFAULT_KI                      8       //KI * 1000
#define PID_DEFAULT_KD                      0       
#define MAX_ERR_SUM                         (1000000 / PID_DEFAULT_KI)

//MOTORS ENCODERS PARAMETERS
#define ENC_RESOLUTION              64
#define ENCODER_COUNTER_MAX_VALUE   0xFFFF
#define ENCODER_COUNTER_OFFSET      (ENCODER_COUNTER_MAX_VALUE / 2)
#define TICK_PER_REAR               (ENC_RESOLUTION * GEARBOX_RATIO)
#define TICK_PER_RADIAN             TICK_PER_REAR / (2*PI)
#define TICK_PER_RADIAN_X_1000      TICK_PER_RADIAN * 1000
#define TICK_TO_RAD_X_1000(arg)     (int64_t((arg) * 1000 * 2 * PI) / TICK_PER_REAR)

//HARDWARE DEFINES
#define MOTORS_SETPOINT_TIMEOUT     3000    //ms
#define MOTORS_PWM_FREQUENCY        15000   //Hz
#define GEARBOX_RATIO               50
#define MAX_ANG_VEL                 20000   // rad/s * 1000
#define MAX_CURRENT                 0x01
#define REDUCED_CURRENT             0x00
#define RAMP_ACCELERATION           2000   // rad/s^2 * 1000
#define RAMP_FLAG                   false   // if true - use ramp, it false - without ramp


void SetMaxMotorsCurrent(uint32_t Ilim1_, uint32_t Ilim2_, uint32_t Ilim3_, uint32_t Ilim4_);

class TimebaseTimerClass {
    public:
        TimebaseTimerClass();
        TimebaseTimerClass(TIM_TypeDef* arg_timer);
        ~TimebaseTimerClass();
        uint64_t GetAbsTimeValue();
        uint64_t GetTimeChange(uint64_t* arg_last_time);
    private:
        HardwareTimer* timebase_timer_ = 0;
        uint64_t time_counter_ = 0;
};

class MotorClass {
    public:
        MotorClass();
        MotorClass( uint32_t arg_pwm_pin,           TIM_TypeDef *arg_pwm_timer,         uint8_t arg_pwm_tim_channel,
                    uint32_t arg_a_channel_motor_pin,   uint32_t arg_b_channel_motor_pin, 
                    TIM_TypeDef *arg_encoder_timer, uint32_t arg_a_channel_encoder_pin, uint32_t arg_b_channel_encoder_pin,
                    int8_t arg_default_direction,   TimebaseTimerClass *arg_timebase_timer);
        ~MotorClass();
        //basic motor controll methods
        void SoftStop(void);
        void EmgStop(void);
        void SetMove(int32_t arg_velocity);
        void SetPwm(uint32_t arg_value);
        void SetCurrentLimit(uint8_t arg_current_mode);
        //motor feedback methods
        int32_t GetVelocity(void);
        int64_t GetWheelAbsPosition(void);
        int16_t GetWheelAngle(void);
        int8_t GetDefaultDirection(void);
        //PID methods
        void SetPidSetpoint(int32_t arg_setpoint);
        void SetPidSetpoint(float arg_setpoint);
        void PidLoopHandler();
        void PidLoopHandler(int32_t arg_setpoint);
        void PidLoopHandler(float arg_setpoint);
        void SetPidParameters(uint16_t arg_kp_gain, uint16_t arg_ki_gain, uint16_t arg_kd_gain);
        void SetPidAcceleration(uint16_t arg_ramp_acceleration);
    private:
        int32_t VelocityUpdate(void);
        uint32_t GetPwmTimerOverflow(void);
        int64_t GetEncoderValue(void);
        HardwareTimer* pwm_timer_;
        HardwareTimer* encoder_timer_;
        TimebaseTimerClass* timebase_tim_;
        int64_t last_encoder_value_;
        int64_t actual_encoder_value_;
        int64_t encoder_value_;
        uint64_t last_time_;
        uint64_t time_change_;
        uint16_t acceleration_;
        int32_t input_;
        int32_t actual_input_;
        int32_t actual_velocity_;
        int32_t last_error_;
        int32_t error_sum_;
        int32_t actual_error_;
        uint16_t kp_gain_ = PID_DEFAULT_KP;
        uint16_t ki_gain_ = PID_DEFAULT_KI;
        uint16_t kd_gain_ = PID_DEFAULT_KD;
        int64_t max_error_sum_ = (1000000 / PID_DEFAULT_KI);
        int32_t output_;
        int8_t default_direction_;
        uint8_t a_channel_motor_pin_;
        uint8_t b_channel_motor_pin_;
        uint8_t a_channel_encoder_pin_;
        uint8_t b_channel_encoder_pin_;
        uint8_t pwm_timer_channel_;
        uint8_t pwm_pin_;
    protected:
    ;
};


#endif /* MOTORS_H */