/**
 * @file motors.c
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2021-12-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "motors.h"

//MOTORS
TimebaseTimerClass timebase_timer(TIMEBASE_TIMER);
MotorClass motor_1( M1_PWM_PIN, M1_PWM_TIM, M1_PWM_TIM_CH, M1A_IN, M1B_IN, 
                    M1_ENC_TIM, M1_ENC_A, M1_ENC_B, M1_DEFAULT_DIR, &timebase_timer);
MotorClass motor_2( M2_PWM_PIN, M2_PWM_TIM, M2_PWM_TIM_CH, M2A_IN, M2B_IN, 
                    M2_ENC_TIM, M2_ENC_A, M2_ENC_B, M2_DEFAULT_DIR, &timebase_timer);
MotorClass motor_3( M3_PWM_PIN, M3_PWM_TIM, M3_PWM_TIM_CH, M3A_IN, M3B_IN,
                    M3_ENC_TIM, M3_ENC_A, M3_ENC_B, M3_DEFAULT_DIR, &timebase_timer);
MotorClass motor_4( M4_PWM_PIN, M4_PWM_TIM, M4_PWM_TIM_CH, M4A_IN, M4B_IN, 
                    M4_ENC_TIM, M4_ENC_A, M4_ENC_B, M4_DEFAULT_DIR, &timebase_timer);
MotorClass wheel_motors[] = {motor_1, motor_2, motor_3, motor_4};

void SetMaxMotorsCurrent(uint32_t Ilim1_, uint32_t Ilim2_, uint32_t Ilim3_, uint32_t Ilim4_){
    if(GetBoardVersion() == "v1.3"){
        pinMode(Ilim1_, INPUT);
        pinMode(Ilim2_, INPUT);
        pinMode(Ilim3_, INPUT);
        pinMode(Ilim4_, INPUT);
    }
    else{
        pinMode(Ilim1_, OUTPUT);
        pinMode(Ilim2_, OUTPUT);
        pinMode(Ilim3_, OUTPUT);
        pinMode(Ilim4_, OUTPUT);
        digitalWrite(Ilim1_, HIGH);
        digitalWrite(Ilim2_, HIGH);
        digitalWrite(Ilim3_, HIGH);
        digitalWrite(Ilim4_, HIGH);
    }
}

MotorClass::MotorClass(){
}

MotorClass::MotorClass( uint32_t arg_pwm_pin,           TIM_TypeDef *arg_pwm_timer,         uint8_t arg_pwm_tim_channel,
                        uint32_t arg_a_channel_mot,         uint32_t arg_b_channel_mot, 
                        TIM_TypeDef *arg_encoder_timer, uint32_t arg_a_channel_encoder_pin,     uint32_t arg_b_channel_encoder_pin,
                        int8_t arg_default_direction,   TimebaseTimerClass *arg_timebase_timer){
    this->pwm_pin_ = arg_pwm_pin;
    this->pwm_timer_ = new HardwareTimer(arg_pwm_timer);
    this->pwm_timer_channel_ = arg_pwm_tim_channel;
    this->pwm_timer_->setMode(this->pwm_timer_channel_, TIMER_OUTPUT_COMPARE_PWM1, this->pwm_pin_, 0);
    this->pwm_timer_->setOverflow(MOTORS_PWM_FREQUENCY, HERTZ_FORMAT);
    this->pwm_timer_->setCaptureCompare(this->pwm_timer_channel_, 0, TICK_COMPARE_FORMAT);
    this->pwm_timer_->resume();
    this->a_channel_motor_pin_ = arg_a_channel_mot;
    this->b_channel_motor_pin_ = arg_b_channel_mot;
    pinMode(this->a_channel_motor_pin_, OUTPUT);
    pinMode(this->b_channel_motor_pin_, OUTPUT);
    this->SoftStop();
    this->a_channel_encoder_pin_ = arg_a_channel_encoder_pin;
    this->b_channel_encoder_pin_ = arg_b_channel_encoder_pin;
    this->encoder_timer_ = new HardwareTimer(arg_encoder_timer);
    this->encoder_timer_->setMode(1, TIMER_INPUT_ENCODER_MODE12, this->a_channel_encoder_pin_, this->b_channel_encoder_pin_);
    this->encoder_timer_->setOverflow(ENCODER_COUNTER_MAX_VALUE);
    this->encoder_timer_->refresh();
    this->encoder_timer_->setCount(ENCODER_COUNTER_OFFSET);
    this->encoder_timer_->resume();
    this->encoder_timer_->getUnderOverFlow(ENCODER_COUNTER_MAX_VALUE);
    this->default_direction_ = arg_default_direction;
    this->timebase_tim_ = arg_timebase_timer;
    this->last_time_ = this->timebase_tim_->GetAbsTimeValue();
    this->last_encoder_value_ = this->actual_encoder_value_ = this->GetEncoderValue();
    }

MotorClass::~MotorClass(){
    ;
}

void MotorClass::SetPidSetpoint(int32_t arg_setpoint){
    this->input_ = arg_setpoint;
}

void MotorClass::SetPidSetpoint(float arg_setpoint){
    this->input_ = (int32_t)(arg_setpoint * 1000);
}

void MotorClass::PidLoopHandler(int32_t arg_setpoint){
    this->SetPidSetpoint(arg_setpoint);
    this->PidLoopHandler();
}

void MotorClass::PidLoopHandler(float arg_setpoint){
    this->SetPidSetpoint(arg_setpoint);
    this->PidLoopHandler();
}

int32_t MotorClass::VelocityUpdate(void){
    this->time_change_ = this->timebase_tim_->GetTimeChange(&this->last_time_);
    this->actual_encoder_value_ = this->GetEncoderValue();
    this->actual_velocity_ =    TICK_TO_RAD_X_1000(this->actual_encoder_value_ - this->last_encoder_value_) * 10000
                                / (int64_t)(this->time_change_ * this->default_direction_);
    this->last_encoder_value_ = this->actual_encoder_value_;
    return actual_velocity_;
}

int32_t MotorClass::GetVelocity(void){
    return this->actual_velocity_;
}

int64_t MotorClass::GetWheelAbsPosition(void){
    return (int64_t)TICK_TO_RAD_X_1000(this->GetEncoderValue() * (int64_t) this->GetDefaultDirection());
}

int8_t MotorClass::GetDefaultDirection(void){
    return this->default_direction_;
}

int64_t MotorClass::GetEncoderValue(void){
    int8_t flag = encoder_timer_->getUnderOverFlow(ENCODER_COUNTER_MAX_VALUE);
    if(flag == 1){
        encoder_value_ += ENCODER_COUNTER_MAX_VALUE;
    }
    if(flag == -1){
        encoder_value_ -= ENCODER_COUNTER_MAX_VALUE;
    }
    return (encoder_value_ + encoder_timer_->getCount()-ENCODER_COUNTER_OFFSET);
}


void MotorClass::SetPwm(uint32_t arg_value){
    uint32_t pwm_tim_max = this->GetPwmTimerOverflow();
    this->pwm_timer_->setCaptureCompare(this->pwm_timer_channel_, pwm_tim_max - constrain(arg_value, 0, pwm_tim_max),
                                        TICK_COMPARE_FORMAT);
}

void MotorClass::SetMove(int32_t arg_velocity){
    this->SetPwm(abs(arg_velocity));
    if((arg_velocity * (int32_t)this->default_direction_) > 0){      ;    //backward move
        pinMode(this->a_channel_motor_pin_, INPUT);
        digitalWrite(this->a_channel_motor_pin_, LOW);
        //
        pinMode(this->b_channel_motor_pin_, OUTPUT);
        digitalWrite(this->b_channel_motor_pin_, HIGH);
    }
    else if((arg_velocity * (int32_t)this->default_direction_) < 0){     ; //forward move
        pinMode(this->a_channel_motor_pin_, OUTPUT);
        digitalWrite(this->a_channel_motor_pin_, HIGH);
        //
        pinMode(this->b_channel_motor_pin_, INPUT);
        digitalWrite(this->b_channel_motor_pin_, LOW);
    }
}

void MotorClass::EmgStop(void){
    pinMode(this->a_channel_motor_pin_, OUTPUT);
    pinMode(this->b_channel_motor_pin_, OUTPUT);
    digitalWrite(this->a_channel_motor_pin_, LOW);
    digitalWrite(this->b_channel_motor_pin_, LOW);
}

void MotorClass::SoftStop(void){
    pinMode(this->a_channel_motor_pin_, OUTPUT);
    pinMode(this->b_channel_motor_pin_, OUTPUT);
    digitalWrite(this->a_channel_motor_pin_, HIGH);
    digitalWrite(this->b_channel_motor_pin_, HIGH);
}

int16_t MotorClass::GetWheelAngle(void){
    return (int16_t)this->actual_encoder_value_ % TICK_PER_RADIAN_X_1000;
}


uint32_t MotorClass::GetPwmTimerOverflow(void){
    return this->pwm_timer_->getOverflow();
}

void MotorClass::PidLoopHandler(void){
    if(RAMP_FLAG){
        if(this->actual_input_ < this->input_){
            if((this->actual_input_ + RAMP_ACCELERATION) > this->input_)
                this->actual_input_ = this->input_;
            else
                this->actual_input_ += (int32_t)(RAMP_ACCELERATION / PID_FREQ);
        }
        if(this->actual_input_ > this->input_){
            if((this->actual_input_ - RAMP_ACCELERATION < this->input_))
                this->actual_input_ = this->actual_input_;
            else
                this->actual_input_ -= (int32_t)(RAMP_ACCELERATION / PID_FREQ);
        }
    }
    else{
        this->actual_input_ = this->input_;
    }
    actual_error_ = (this->actual_input_) - this->VelocityUpdate();
    error_sum_ = constrain(this->error_sum_, (-1 * this->max_error_sum_), this->max_error_sum_);
    this->output_ = this->kp_gain_ * actual_error_;
    this->output_ += this->ki_gain_ * this->error_sum_;
    this->output_ += this->kd_gain_ * (this->actual_error_ - this->last_error_);
    this->last_error_ = this->actual_error_;
    this->error_sum_ += this->actual_error_;
    output_ = constrain(this->output_, -1000000, 1000000);
    this->SetMove(output_ / 1000 * int32_t(this->GetPwmTimerOverflow()) / 1000 );
}


TimebaseTimerClass::TimebaseTimerClass(){
    ;
}

TimebaseTimerClass::TimebaseTimerClass(TIM_TypeDef* arg_timer){
    this->timebase_timer_ = new HardwareTimer(arg_timer);
    this->timebase_timer_->setPrescaleFactor(TIMEBASE_TIMER_PSC);
    this->timebase_timer_->setOverflow(TIMEBASE_TIMER_OVERFLOW_VALUE, TICK_FORMAT);
    this->timebase_timer_->refresh();
    this->timebase_timer_->resume();
}

TimebaseTimerClass::~TimebaseTimerClass(){
    ;
}

uint64_t TimebaseTimerClass::GetAbsTimeValue(){
    int8_t flag = this->timebase_timer_->getUnderOverFlow(TIMEBASE_TIMER_OVERFLOW_VALUE);
    if(flag == 1){
        time_counter_ += TIMEBASE_TIMER_OVERFLOW_VALUE;
    }
    if(flag == -1){
        time_counter_ -= TIMEBASE_TIMER_OVERFLOW_VALUE;
    }
    return (time_counter_ + this->timebase_timer_->getCount());
}

uint64_t TimebaseTimerClass::GetTimeChange(uint64_t* arg_last_time){
    uint64_t ret_val = this->GetAbsTimeValue() - *arg_last_time;
    *arg_last_time = this->GetAbsTimeValue();
    return ret_val;
}

