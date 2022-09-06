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
MotorClass Motor1(M1_PWM_PIN, M1_PWM_TIM, M1_PWM_TIM_CH, M1_ILIM, M1A_IN, M1B_IN, M1_ENC_TIM, M1_ENC_A, M1_ENC_B, M1_DEFAULT_DIR);
MotorClass Motor2(M2_PWM_PIN, M2_PWM_TIM, M2_PWM_TIM_CH, M2_ILIM, M2A_IN, M2B_IN, M2_ENC_TIM, M2_ENC_A, M2_ENC_B, M2_DEFAULT_DIR);
MotorClass Motor3(M3_PWM_PIN, M3_PWM_TIM, M3_PWM_TIM_CH, M3_ILIM, M3A_IN, M3B_IN, M3_ENC_TIM, M3_ENC_A, M3_ENC_B, M3_DEFAULT_DIR);
MotorClass Motor4(M4_PWM_PIN, M4_PWM_TIM, M4_PWM_TIM_CH, M4_ILIM, M4A_IN, M4B_IN, M4_ENC_TIM, M4_ENC_A, M4_ENC_B, M4_DEFAULT_DIR);
MotorPidClass M1_PID(&Motor1);
MotorPidClass M2_PID(&Motor2);
MotorPidClass M3_PID(&Motor3);
MotorPidClass M4_PID(&Motor4);
MotorPidClass wheel_motors[] = {M1_PID, M2_PID, M3_PID, M4_PID};

MotorClass::MotorClass(uint32_t Pwm_pin_, TIM_TypeDef *Pwm_timer_, uint8_t PWM_tim_channel_, uint32_t Ilim_pin_, uint32_t A_channel_mot_,
             uint32_t B_channel_mot_, TIM_TypeDef *Enc_timer_, uint32_t A_channel_enc_, uint32_t B_channel_enc_, int8_t DefaultDir_){
    this->DefaultDir = DefaultDir_;
    this->PWM_pin = Pwm_pin_;
    this->PWM_tim_channel = PWM_tim_channel_;
    this->Ilim_pin = Ilim_pin_;
    this->A_channel_mot = A_channel_mot_;
    this->B_channel_mot = B_channel_mot_;
    this->A_channel_enc = A_channel_enc_;
    this->B_channel_enc = B_channel_enc_;
    this->Enc_tim = new HardwareTimer(Enc_timer_);
    this->Enc_tim->setMode(1, TIMER_INPUT_ENCODER_MODE12, A_channel_enc, B_channel_enc);
    this->Enc_tim->setOverflow(ENC_MAX_CNT);
    this->Enc_tim->refresh();
    this->Enc_tim->setCount(ENC_CNT_OFFSET);
    this->Enc_tim->resume();
    this->Enc_tim->getUnderOverFlow(ENC_MAX_CNT); // clear flag
    this->Pwm_tim = new HardwareTimer(Pwm_timer_);
    this->Pwm_tim->setMode(PWM_tim_channel_, TIMER_OUTPUT_COMPARE_PWM1, Pwm_pin_, 0);
    this->Pwm_tim->setOverflow(MOTORS_PWM_FREQUENCY, HERTZ_FORMAT);
    this->Pwm_tim->setCaptureCompare(PWM_tim_channel, 0, TICK_COMPARE_FORMAT);
    this->Pwm_tim->resume();
    pinMode(this->A_channel_mot, OUTPUT);
    pinMode(this->B_channel_mot, OUTPUT);
    pinMode(this->Ilim_pin, OUTPUT);
    this->SoftStop();
    this->SetCurrentLimit(MAX_CURRENT);
    prev_enc_val_ = actual_enc_val_ = this->EncValUpdate();
    prev_time_ = actual_time_ = xTaskGetTickCount();
}

MotorClass::~MotorClass(){
    ;
}

int64_t MotorClass::EncValUpdate(void){
    int8_t flag = Enc_tim->getUnderOverFlow(ENC_MAX_CNT);
    if(flag == 1){
        Enc_value += ENC_MAX_CNT;
    }
    if(flag == -1){
        Enc_value -= ENC_MAX_CNT;
    }
    return (Enc_value + Enc_tim->getCount()-ENC_CNT_OFFSET);
}


void MotorClass::SetPWM(uint16_t setpoint){
    uint32_t pwm_tim_max = this->GetPwmTimerOverflow();
    uint32_t power = uint32_t(constrain(setpoint, 0, pwm_tim_max));
    // Serial.printf("PWM timer value: %d, PWM timer max value: %d\r\n", uint16_t(power), uint16_t(this->GetPwmTimerOverflow()));
    this->Pwm_tim->setCaptureCompare(this->PWM_tim_channel, pwm_tim_max - power, TICK_COMPARE_FORMAT);
}


void MotorClass::SetMove(int16_t vel){
    this->SetPWM(abs(vel));
    if(vel < 0){      ;    //backward move
        pinMode(this->A_channel_mot, INPUT);
        digitalWrite(this->A_channel_mot, LOW);
        //
        pinMode(this->B_channel_mot, OUTPUT);
        digitalWrite(this->B_channel_mot, HIGH);
    }
    else if(vel > 0){     ; //forward move
        pinMode(this->A_channel_mot, OUTPUT);
        digitalWrite(this->A_channel_mot, HIGH);
        //
        pinMode(this->B_channel_mot, INPUT);
        digitalWrite(this->B_channel_mot, LOW);
    }
    // else
    //     this->SoftStop();
}

void MotorClass::EmgStop(void){
    pinMode(this->A_channel_mot, OUTPUT);
    pinMode(this->B_channel_mot, OUTPUT);
    digitalWrite(this->A_channel_mot, LOW);
    digitalWrite(this->B_channel_mot, LOW);
}

void MotorClass::SoftStop(void){
    pinMode(this->A_channel_mot, OUTPUT);
    pinMode(this->B_channel_mot, OUTPUT);
    digitalWrite(this->A_channel_mot, HIGH);
    digitalWrite(this->B_channel_mot, HIGH);
}

double MotorClass::VelocityUpdate(void){
    actual_time_ = xTaskGetTickCount();
    actual_enc_val_ = this->EncValUpdate();
    time_change_ = double(actual_time_ - prev_time_)/1000; //in seconds
    this->Velocity = double(actual_enc_val_ - prev_enc_val_) / (IMP_PER_RAD) / time_change_;
    prev_enc_val_ = actual_enc_val_;
    prev_time_ = actual_time_;
    return this->Velocity*this->DefaultDir;
}

double MotorClass::GetVelocity(void){
    return this->Velocity*this->DefaultDir;
}

double MotorClass::GetPosition(void){
    return (double(this->actual_enc_val_) / (double(ENC_RESOLUTION) * double(GEARBOX_RATIO)))*2*PI*(double)this->DefaultDir;
}

double MotorClass::GetWheelAngle(void){
    int16_t act_angle = actual_enc_val_ % (ENC_RESOLUTION*GEARBOX_RATIO);
    double pos_rad = double(act_angle)/double(ENC_RESOLUTION)/double(GEARBOX_RATIO)*PI;
    if (actual_enc_val_ >= 0) {
        return pos_rad - PI/2;
    }
    else {
        return PI + pos_rad - PI/2;
    }
}


int8_t MotorClass::GetDefaultDir(void){
    return DefaultDir;
}

uint32_t MotorClass::GetPwmTimerOverflow(void){
    return this->Pwm_tim->getOverflow();
}

void MotorClass::SetCurrentLimit(uint8_t CurrentMode_){
    if(CurrentMode_ = MAX_CURRENT)
        digitalWrite(this->Ilim_pin, HIGH);
    if(CurrentMode_ = REDUCED_CURRENT)
        digitalWrite(this->Ilim_pin, LOW);
}

MotorPidClass::MotorPidClass(MotorClass* Motor_){
    Motor = Motor_;
    OutputMax = double(this->Motor->GetPwmTimerOverflow()-1);
    OutputMin = OutputMax * (-1);
}

MotorPidClass::~MotorPidClass(){
    ;
}

void MotorPidClass::SetSetpoint(double Setpoint_){
    Setpoint = Setpoint_;
}

void MotorPidClass::Handler(void){
    if(RAMP_FLAG){
        if(ActualSetpoint < Setpoint){
            if((ActualSetpoint + RAMP_ACCELERATION) > Setpoint)
                ActualSetpoint = Setpoint;
            else
                ActualSetpoint += (double)((RAMP_ACCELERATION) / (PID_FREQ));
        }
        if(ActualSetpoint > Setpoint){
            if((ActualSetpoint - RAMP_ACCELERATION < Setpoint))
                ActualSetpoint = Setpoint;
            else
                ActualSetpoint -= (double)((RAMP_ACCELERATION)/(PID_FREQ));
        }
    }
    else{
        ActualSetpoint = Setpoint;
    }
    double Error = ActualSetpoint - Motor->VelocityUpdate();
    double U = Kp * Error + Ki * ErrorSum + Kd * (Error - LastError);
    LastError = Error;
    ErrorSum += Error;
    Motor->SetMove(int16_t(U * OutputMax * this->Motor->GetDefaultDir() * (-1)));
}