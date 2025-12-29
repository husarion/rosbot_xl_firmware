/**
 * @file hardware_cfg.h
 * @author Jakub Klein
 * @brief
 * @version 0.1
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ROSBOT__HARDWARE_CFG
#define ROSBOT__HARDWARE_CFG


/* OTHERS */
#define DEFAULT_FIRMWARE_MODE 2  // 0 - normal; 1 - error; 2 - debug

// POWER OFF
#define SBC_ETH_CONNECT_TIMEOUT 10  // ms
#define POWEROFF_DELAY 5000         // ms

/* REAR PANEL */
#define RD_LED PE2
#define GRN_LED PE3
#define GRN_LED2 PE4
#define PUSH_BUTTON1 PG12
#define PUSH_BUTTON2 PG13

/* THERMISTOR NTC*/
#define NTC_SENS_PIN PB1  // ADC2 IN9
#define NTC_SENS_C1 0.001112613927
#define NTC_SENS_C2 0.000237277392
#define NTC_SENS_C3 0.000000071670
#define NTC_PULLUP_RES 5230          // NTC pull up resisior
#define NTC_OFFSET_VAL (273.15 + 3)  // Kelvin to Celsius offset + calibration offset

/* SBC */
#define SBC_SERIAL Serial1
#define SBC_SERIAL_BAUDRATE 576000
#define SBC_SERIAL_TX PA9
#define SBC_SERIAL_RX PA10
#define SBC_SERIAL_TIMEOUT 1 // ms
#define SBC_STATUS PG6 // According to "Rosbot v1.3 schematics", this should be connected to GPIO_03 in RPI which is an I2C with pullup (intended for detection)
#define RPI_CONSOLE PG5
#define RPI_BTN PG7

/* FTDI SERIAL */
#define FTDI_SERIAL Serial3
#define FTDI_SERIAL_BAUDRATE 115200
#define FTDI_SERIAL_TX PB10
#define FTDI_SERIAL_RX PB11
#define FTDI_SERIAL_TIMEOUT 1 // ms

/* IMU */
#define IMU_POWER_ON PG4
#define IMU_SDA PC9 // I2C3
#define IMU_SCL PA8 // I2C3
#define IMU_SAMPLE_FREQ 25  // Hz
#define IMU_ID 0xA0 // used internally by the Adafruit Unified Sensor API ?
#define IMU_ADDR_A 0x28
#define IMU_ADDR_B 0x29

/* MOTORS */
#define M14_SLEEP PC14
#define M14_FAULT PE1
#define M23_SLEEP PC13
#define M23_FAULT PE0

#define M1_ENC_TIM TIM8
#define M1_ENC_A PC6 // TIM8_CH1
#define M1_ENC_B PC7 // TIM8_CH2
#define M1_PWM_TIM TIM11
#define M1_PWM_TIM_CH 1
#define M1_PWM_PIN PF7
#define M1A_IN PD3
#define M1B_IN PD4
#define M1_DEFAULT_DIR 1  // 1 (CW) or -1 (CCW)

#define M2_ENC_TIM TIM3
#define M2_ENC_A PB4 // TIM3_CH1
#define M2_ENC_B PA7 // TIM3_CH2
#define M2_PWM_TIM TIM13
#define M2_PWM_TIM_CH 1
#define M2_PWM_PIN PF8
#define M2A_IN PC15
#define M2B_IN PF2
#define M2_DEFAULT_DIR -1  // 1 (CW) or -1 (CCW)

#define M3_ENC_TIM TIM2
#define M3_ENC_A PA0 // TIM2_CH1_ETR
#define M3_ENC_B PA1 // TIM2_CH2
#define M3_PWM_TIM TIM10
#define M3_PWM_TIM_CH 1
#define M3_PWM_PIN PF6
#define M3A_IN PG10
#define M3B_IN PG11
#define M3_DEFAULT_DIR 1  // 1 (CW) or -1 (CCW)

#define M4_ENC_TIM TIM4
#define M4_ENC_A PB6 // TIM4_CH1
#define M4_ENC_B PB7 // TIM4_CH2
#define M4_PWM_TIM TIM14
#define M4_PWM_TIM_CH 1
#define M4_PWM_PIN PF9
#define M4A_IN PE5
#define M4B_IN PE6
#define M4_DEFAULT_DIR -1  // 1 (CW) or -1 (CCW)

/* ETHERNET */
// #define CLIENT_IP "192.168.77.3"
// #define SBC_AGENT_IP "192.168.77.2"  // SBC
// // #define SBC_AGENT_IP 	"192.168.77.5"	//External device
// #define AGENT_PORT 8888
// #define SHUTDOWN_PORT 3000

/* ETH LINK STATUS DEFINES */
// #define ETH_LINK_STATUS_CONNECTED_BIT (1 << 0)  // if set - connected
// #define ETH_LINK_STATUS_ERROR_BIT (1 << 1)

/* EXTERNAL PERIPHERALS */

// // EXT SPI
// #define EXT_SPI SPI1
// #define EXT_SPI_SCK PA5
// #define EXT_SPI_MISO PA6
// #define EXT_SPI_MOSI PB5
// // EXT I2C1
// #define EXT_I2C1 I2C1
// #define EXT_I2C1_SDA PB7
// #define EXT_I2C1_SCL PB6
// // EXT I2C2
// #define EXT_I2C2 I2C3
// #define EXT_I2C2_SDA PC9
// #define EXT_I2C2_SCL PA8
// // EXT Serial
// #define EXT_SERIAL_EN_FLAG 1
// #define EXT_SERIAL Serial6
// #define EXT_SERIAL_BAUDRATE 115200
// #define EXT_SERIAL_RX PG9
// #define EXT_SERIAL_TX PG14
// // EXT PWM1
// #define EXT_PWM1_TIM TIM9
// #define EXT_PWM1_CH CH1
// #define EXT_PWM1_PIN PE5
// // EXT PWM2
// #define EXT_PWM2_TIM TIM9
// #define EXT_PWM2_CH CH2
// #define EXT_PWM2_PIN PE6
// // EXT PWM3
// #define EXT_PWM3_TIM TIM12
// #define EXT_PWM3_CH CH1
// #define EXT_PWM3_PIN PB14
// // EXT ANALOG
// #define EXT_ANALOG_IN1 PF10
// #define EXT_ANALOG_IN2 PF3
// // EXT GPIO
// #define EXT_GPIO1 PG2
// #define EXT_GPIO2 PG3
// #define EXT_GPIO3 PG4

/* WATCHDOG */
#define WATCHDOG_TIMEOUT 20000000  // microseconds

/* BATTERY */

#define BATTERY_CELLS_SERIES 3
#define BATTERY_CELLS_PARALLEL 3
#define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE 1  // in unmeasured
#define BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE 1      // in unmeasured
// #define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE
// (BATTERY_CELLS_PARALLEL * BATTERY_CELLS_SERIES) #define
// BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE (BATTERY_CELLS_PARALLEL *
// BATTERY_CELLS_SERIES)

#endif /* ROSBOT__HARDWARE_CFG */
