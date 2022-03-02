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
#include <PID_v1.h>
#include <STM32FreeRTOS.h>
#include <sensor_msgs/msg/joint_state.h>

#define M1_ENC_TIM      TIM1
#define M1_ENC_A        PE9
#define M1_ENC_B        PE11
#define M1_PWM_TIM      TIM10
#define M1_PWM_PIN      PF6
#define M1_PWM_TIM_CH   1
#define M1A_IN          PE12
#define M1B_IN          PE13
#define M1_ILIM         PE10
#define M1_DEFAULT_DIR  -1           // 1 (CW) or -1 (CCW)

#define M2_ENC_TIM      TIM2
#define M2_ENC_A        PA15
#define M2_ENC_B        PB3
#define M2_PWM_TIM      TIM11
#define M2_PWM_PIN      PF7
#define M2_PWM_TIM_CH   1
#define M2A_IN          PG11
#define M2B_IN          PG12
#define M2_ILIM         PG15
#define M2_DEFAULT_DIR  1          // 1 (CW) or -1 (CCW)

#define M3_ENC_TIM      TIM3
#define M3_ENC_A        PC6
#define M3_ENC_B        PC7
#define M3_PWM_TIM      TIM13
#define M3_PWM_PIN      PF8
#define M3_PWM_TIM_CH   1
#define M3A_IN          PG5
#define M3B_IN          PG6
#define M3_ILIM         PG7
#define M3_DEFAULT_DIR  -1           // 1 (CW) or -1 (CCW)

#define M4_ENC_TIM      TIM4
#define M4_ENC_A        PD12
#define M4_ENC_B        PD13
#define M4_PWM_TIM      TIM14
#define M4_PWM_PIN      PF9
#define M4_PWM_TIM_CH   1
#define M4A_IN          PD10
#define M4B_IN          PD11
#define M4_ILIM         PD14
#define M4_DEFAULT_DIR  1          // 1 (CW) or -1 (CCW)

#define ENC_MAX_CNT                 0xFFFF
#define ENC_CNT_OFFSET              ENC_MAX_CNT/2
#define PID_FREQ                    300  //max 1000Hz
#define PID_DEFAULT_KP              9
#define PID_DEFAULT_KI              0
#define PID_DEFAULT_KD              0
#define MOTORS_PWM_FREQUENCY        15000   //Hz


//HARDWARE DEFINES
#define ENC_RESOLUTION      64      //four edges 
#define GEARBOX_RATIO       50
#define WHEEL_DIAM          0.105   //meters
#define IMP_PER_RAD         ENC_RESOLUTION*GEARBOX_RATIO/(2*PI)
#define MAX_ANG_VEL         20.0    // rad/s
#define RAMP_ACCELERATION   25.0      // rad/s^2
#define RAMP_FLAG           true   // if true - use ramp, it false - without ramp
#define MAX_CURRENT         0x01
#define REDUCED_CURRENT     0x00

//MOTORS ROS MSGS DEFINES
#define MOT_CMD_MSG_LEN         4
#define MOT_CMD_MSG_NAMES_LEN  10
#define MOT_RESP_MSG_LEN        4
// #define MOT_RESP_MSG_NAMES_LEN 10


void MotorsResponseMsgInit(sensor_msgs__msg__JointState* msg);
void MotorsCmdMsgInit(sensor_msgs__msg__JointState* msg);

class MotorClass {
    public:
        MotorClass();
        MotorClass(uint32_t Pwm_pin_, TIM_TypeDef *Pwm_timer_, uint8_t PWM_tim_channel_, uint32_t Ilim_pin_, uint32_t A_channel_mot_,
              uint32_t B_channel_mot_, TIM_TypeDef *Enc_timer_, uint32_t A_channel_enc_, uint32_t B_channel_enc_, int8_t DefaultDir_);
        ~MotorClass();
        int64_t EncValUpdate(void);
        void SetPWM(uint16_t setpoint);
        void SetMove(int16_t vel);
        void SoftStop(void);
        void EmgStop(void);
        double VelocityUpdate(void);
        double GetVelocity(void);
        double GetPosition(void);
        int8_t GetDefaultDir(void);
        uint32_t GetPwmTimerOverflow(void);
        void SetCurrentLimit(uint8_t CurrentMode_);
    private:
        HardwareTimer* Pwm_tim;
        HardwareTimer* Enc_tim;
        int64_t Enc_value;
        uint32_t A_channel_mot;
        uint32_t B_channel_mot;
        uint32_t A_channel_enc;
        uint32_t B_channel_enc;
        uint32_t Ilim_pin;
        uint32_t PWM_pin;
        uint8_t PWM_tim_channel;
        double Velocity = 0; //r ad/s
        int8_t DefaultDir;
        int64_t PrevEncVal = ENC_CNT_OFFSET;
        int64_t ActualEncVal = ENC_CNT_OFFSET;
        uint32_t PrevTime;
        uint32_t ActualTime;
        double TimeChange;
    protected:
    ;
};

class MotorPidClass {
    public:
        MotorPidClass(MotorClass* Motor_);
        ~MotorPidClass();
        void Handler(void);
        void SetSetpoint(double);
        MotorClass *Motor;
        PID *MotorPID;
        double Kp = PID_DEFAULT_KP;
        double Ki = PID_DEFAULT_KI;
        double Kd = PID_DEFAULT_KD;
        double Input = 0;
        double Output = 0;
        double Setpoint = 0;           //velocity rad/s
        double ActualSetpoint = 0;     //velocity rad/s
        double PidSetpoint = 0;       
        double OutputMin;
        double OutputMax;
        private:
};


#endif /* MOTORS_H */