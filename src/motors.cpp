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


void MotorsResponseMsgInit(sensor_msgs__msg__JointState * msg){
    static double pos[4]; 
    static double vel[4];
    static double eff[4];
    static rosidl_runtime_c__String str_name_tab[4];
    msg->position.capacity = 4;
    msg->position.size = 4; 
    msg->position.data = pos;
    msg->effort.capacity = 4;
    msg->effort.size = 4; 
    msg->effort.data = eff;
    msg->velocity.capacity = 4;
    msg->velocity.size = 4;
    msg->velocity.data = vel;
    msg->header.frame_id.capacity = 20;
    msg->header.frame_id.size = 20;
    msg->header.frame_id.data = (char*) "motors_response";

    str_name_tab->capacity = 4;
    str_name_tab->size = 4;
    
    str_name_tab[0].capacity = 3;
    str_name_tab[0].size = 2;
    str_name_tab[0].data = (char*)"FR";
    
    str_name_tab[1].capacity = 3;
    str_name_tab[1].size = 2;
    str_name_tab[1].data = (char*)"FL";
    
    str_name_tab[2].capacity = 3;
    str_name_tab[2].size = 2;
    str_name_tab[2].data = (char*)"RR";
    
    str_name_tab[3].capacity = 3;
    str_name_tab[3].size = 2;
    str_name_tab[3].data = (char*)"RL";

    msg->name.capacity = 4;
    msg->name.size = 4;
    msg->name.data->size = 4;
    msg->name.data->capacity = 4;
    msg->name.data = str_name_tab;
}

void MotorsCmdMsgInit(sensor_msgs__msg__JointState * msg){
    static char tab0[10];
    static char tab1[10];
    static char tab2[10];
    static char tab3[10];
    static double pos[10]; 
    static double vel[10];
    static double eff[10];
    static rosidl_runtime_c__String str_name_tab[10];

    msg->position.capacity = 10;
    msg->position.data = pos;
    msg->effort.capacity = 10;
    msg->effort.data = eff;
    msg->velocity.capacity = 10;
    msg->velocity.data = vel;
    msg->header.frame_id.capacity = 20;

    str_name_tab->capacity = 10;
    str_name_tab->size = 10;
    
    str_name_tab[0].capacity = 10;
    str_name_tab[0].data = tab0;
    
    str_name_tab[1].capacity = 10;
    str_name_tab[1].data = tab1;
    
    str_name_tab[2].capacity = 10;
    str_name_tab[2].data = tab2;
    
    str_name_tab[3].capacity = 10;
    str_name_tab[3].data = tab3;

    msg->name.capacity = 10;
    msg->name.data->capacity = 10;
    msg->name.data = str_name_tab;
}

void SetpointMsgInit(sensor_msgs__msg__JointState* msg){
    static char tab0[3];
    static char tab1[3];
    static char tab2[3];
    static char tab3[3];
    static double pos[4]; 
    static double vel[4];
    static double eff[4];
    static rosidl_runtime_c__String__Sequence msg_name;
    static rosidl_runtime_c__String str_name_tab[4];
    msg->position.capacity = 4;
    msg->position.size = 4; 
    msg->position.data = pos;
    msg->effort.capacity = 4;
    msg->effort.size = 4; 
    msg->effort.data = eff;
    msg->velocity.capacity = 4;
    msg->velocity.size = 4;
    msg->velocity.data = vel;
    msg->header.frame_id.capacity = 20;
    msg->header.frame_id.size = 20;

    str_name_tab->capacity = 4;
    str_name_tab->size = 4;
    
    str_name_tab[0].capacity = 3;
    str_name_tab[0].size = 2;
    
    str_name_tab[1].capacity = 3;
    str_name_tab[1].size = 2;
    
    str_name_tab[2].capacity = 3;
    str_name_tab[2].size = 2;
    
    str_name_tab[3].capacity = 3;
    str_name_tab[3].size = 2;

    msg_name.capacity = 4;
    msg_name.size = 4;
    msg_name.data->size = 4;
    msg_name.data->capacity = 4;
    msg_name.data = str_name_tab;
    msg->name = msg_name;
}

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
    PrevEncVal = ActualEncVal = this->EncValUpdate();
    PrevTime = ActualTime = xTaskGetTickCount();
}

MotorClass::~MotorClass(){
    ;
}

int64_t MotorClass::EncValUpdate(void){
    int8_t flag = Enc_tim->getUnderOverFlow(ENC_MAX_CNT);
    if(flag == 1)
        Enc_value += ENC_MAX_CNT;
    if(flag == -1)
        Enc_value -= ENC_MAX_CNT;
    return (Enc_value + Enc_tim->getCount()-ENC_CNT_OFFSET);
}


void MotorClass::SetPWM(uint16_t setpoint){
    uint32_t PwmTimMax = this->GetPwmTimerOverflow();
    uint32_t power = uint32_t(constrain(setpoint, 0, PwmTimMax));
    // Serial.printf("PWM timer value: %d, PWM timer max value: %d\r\n", uint16_t(power), uint16_t(this->GetPwmTimerOverflow()));
    this->Pwm_tim->setCaptureCompare(this->PWM_tim_channel, PwmTimMax - power, TICK_COMPARE_FORMAT);
}


void MotorClass::SetMove(int16_t vel){
    vel = vel;
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
    ActualTime = xTaskGetTickCount();
    ActualEncVal = this->EncValUpdate();
    TimeChange = double(ActualTime-PrevTime)/1000; //in secondes
    // this->Velocity = ((double(ActualEncVal-PrevEncVal))*1000/(IMP_PER_RAD)/TimeChange);
    this->Velocity = double(ActualEncVal-PrevEncVal)/(IMP_PER_RAD)/TimeChange;
    // this->Velocity = double(ActualEncVal-PrevEncVal)/(GEARBOX_RATIO*ENC_RESOLUTION)/TimeChange;
    // Serial.printf("Actual Value: %d, Previous Value: %d, Velocity: %d, LastTime: %d, NowTime: %d, Time Change: %d \r\n", 
    //             int32_t(ActEncVal), int32_t(PrevEncVal), int32_t(this->Velocity), LastTime, NowTime, TimeChange);
    PrevEncVal = ActualEncVal;
    PrevTime = ActualTime;
    return this->Velocity*this->DefaultDir;
}

double MotorClass::GetVelocity(void){
    return this->Velocity*this->DefaultDir;
}


double MotorClass::GetPosition(void){
    int16_t act_angle = ActualEncVal%(ENC_RESOLUTION*GEARBOX_RATIO);
    double pos_rad = double(act_angle)/double(ENC_RESOLUTION)/double(GEARBOX_RATIO)*PI;
    if (ActualEncVal >= 0) {
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
    uint8_t PidDirection;
    if(this->Motor->GetDefaultDir() == 1)
        PidDirection = REVERSE;
    if(this->Motor->GetDefaultDir() == -1)
        PidDirection = DIRECT;
    PID myPID(&Input, &Output, &PidSetpoint, Kp, Ki, Kd, P_ON_E, PidDirection);
    MotorPID = new PID(myPID);
    OutputMax = double(this->Motor->GetPwmTimerOverflow()-1);
    OutputMin = OutputMax * (-1);
    MotorPID->SetOutputLimits(OutputMin, OutputMax);
    MotorPID->SetSampleTime(1000/PID_FREQ);         
    MotorPID->SetMode(AUTOMATIC);
}

MotorPidClass::~MotorPidClass(){
    ;
}

void MotorPidClass::SetSetpoint(double Setpoint_){
    Setpoint = Setpoint_;
}

// void MotorPidClass::Handler(void){

//     PidSetpoint = (ActualSetpoint*OutputMax)/(MAX_ANG_VEL)*Motor->GetDefaultDir();
//     Input = double((Motor->VelocityUpdate()*OutputMax)/(MAX_ANG_VEL));
//     //Serial.printf("Input: %d, Velocity: %d \r\n", int16_t(Input), int16_t(temp));
//     MotorPID->Compute();
//     Motor->SetMove(int16_t(this->Output));
//     //Serial.printf("Actual setpoint: %d, Setpoint: %d, Output: %d, Input: %d \r\n", int16_t(this->ActualSetpoint), int16_t(this->Setpoint), int16_t(this->Output), int16_t(this->Input));
// }

void MotorPidClass::Handler(void){
    if(RAMP_FLAG){
        if(ActualSetpoint < Setpoint){
            if((ActualSetpoint + RAMP_ACCELERATION) > Setpoint)
                ActualSetpoint = Setpoint;
            else
                ActualSetpoint += (RAMP_ACCELERATION)/(PID_FREQ);
        }
        if(ActualSetpoint > Setpoint){
            if((ActualSetpoint - RAMP_ACCELERATION < Setpoint))
                ActualSetpoint = Setpoint;
            else
                ActualSetpoint -= (RAMP_ACCELERATION)/(PID_FREQ);
        }
    }
    else{
        ActualSetpoint = Setpoint;
    }
    PidSetpoint = (ActualSetpoint*OutputMax)/(MAX_ANG_VEL);
    Input = double((Motor->VelocityUpdate()*OutputMax)/(MAX_ANG_VEL));
    MotorPID->Compute();
    Motor->SetMove(int16_t(this->Output));
}