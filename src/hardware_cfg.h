/**
 * @file hardware_cfg.h
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2022-02-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef HARDWARE_CFG
#define HARDWARE_CFG

#include <stdint.h>

// Define all inputs/outputs etc.

/* ROBOT MECHANICAL DIMENSIONS */
#define ROBOT_WIDTH     0.200   //meters
#define ROBOT_LENGTH    0.170   //meters

/* OTHERS */
#define BOARD_MODE_DEBUG    true    //if true - debug mode is active

/* REAR PANEL */
#define GRN_LED             PE3
#define RD_LED              PE4
#define PUSH_BUTTON1        PF11
#define PUSH_BUTTON2        PF12

/* PERIPHERALS */
#define EN_LOC_5V           PF13
#define FAN                 PC13
#define DIP_SW              PD2     //or PD3 -> to check

/* AUDIO */
#define AUDIO_SHDN          PB2
#define AUDIO_DAC_OUT       PA4
#define AUDIO_DAC_CH        OUT1

/* POWER BOARD */
#define PWR_BRD_GPIO_INPUT  PD4 //PB5 on power board -> output push pull
#define PWR_BRD_GPIO_OUTPUT PD7 //PB8 on power board -> input
#define PWR_BRD_RX          PD6
#define PWR_BRD_TX          PD5
#define PWR_BRD_SERIAL      UART2

/* IMU */
#define IMU_I2C             I2C2
#define IMU_SDA             PF0
#define IMU_SCL             PF1
#define IMU_GPIO_IT         PF2
#define IMU_SAMPLE_FREQ     50  //Hz
#define IMU_ID              0x37
#define IMU_ADDR_A          0x28
#define IMU_ADDR_B          0x29

/* PIXEL LED */
#define PIXEL_SPI           SPI2  
#define PIXEL_MOSI          PB15
#define PIXEL_SCK           PB10
#define PIXEL_LENGTH        18
#define VIRTUAL_LED_LENGTH  10
#define PIXEL_SPI_SPEED     4000000

/* SBC */
#define SBC_SERIAL_RX       PA10
#define SBC_SERIAL_TX       PA9
#define SBC_SERIAL_BAUDRATE 460800

/* KINEMATICS */
#define KINEMATIC_TASK_FREQ     25 //Hz


/* EXTERNAL PERIPHERALS */

//EXT SPI
#define EXT_SPI             SPI1
#define EXT_SPI_SCK         PA5
#define EXT_SPI_MISO        PA6
#define EXT_SPI_MOSI        PB5
//EXT I2C1
#define EXT_I2C1            I2C1
#define EXT_I2C1_SDA        PB7
#define EXT_I2C1_SCL        PB6
//EXT I2C2
#define EXT_I2C2            I2C3
#define EXT_I2C2_SDA        PC9
#define EXT_I2C2_SCL        PA8
//EXT Serial
#define EXT_SERIAL          UART6
#define EXT_SERIAL_RX       PG9
#define EXT_SERIAL_TX       PG14
//EXT PWM1
#define EXT_PWM1_TIM        TIM9
#define EXT_PWM1_CH         CH1
#define EXT_PWM1_PIN        PE5
//EXT PWM2
#define EXT_PWM2_TIM        TIM9
#define EXT_PWM2_CH         CH2
#define EXT_PWM2_PIN        PE6
//EXT PWM3
#define EXT_PWM3_TIM        TIM12
#define EXT_PWM3_CH         CH1
#define EXT_PWM3_PIN        PB14
//EXT ANALOG
#define EXT_ANALOG_IN1      PF10
#define EXT_ANALOG_IN2      PF3
//EXT GPIO
#define EXT_GPIO1           PG2
#define EXT_GPIO2           PG3
#define EXT_GPIO3           PG4


#endif /* HARDWARE_CFG */