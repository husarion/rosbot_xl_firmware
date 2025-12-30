/**
 * @file HARDWARE_CFG_ROSBOT_XL.h
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef HARDWARE_CFG_ROSBOT_XL
#define HARDWARE_CFG_ROSBOT_XL

/* OTHERS */
#define DEFAULT_FIRMWARE_MODE 2  // 0 - normal; 1 - error; 2 - debug

// POWER OFF
#define SBC_ETH_CONNECT_TIMEOUT 10  // ms
#define POWEROFF_DELAY 5000         // ms

/* REAR PANEL */
#define GRN_LED PE3
#define RD_LED PE4
#define PUSH_BUTTON1 PF11
#define PUSH_BUTTON2 PF12

/* FAN	*/
#define FAN_PP_PIN PC13
#define FAN_PWM_PIN PB_0_ALT1
#define FAN_PWM_TIMER TIM3
#define FAN_PWM_CHANNEL 3
#define FAN_PWM_FREQUENCY 1000
#define FAN_TEMP_THRSH_UP 35
#define FAN_TEMP_THRSH_DOWN 30

/* THERMISTOR NTC*/
#define NTC_SENS_PIN PB1  // ADC2 IN9
#define NTC_SENS_C1 0.001112613927
#define NTC_SENS_C2 0.000237277392
#define NTC_SENS_C3 0.000000071670
#define NTC_PULLUP_RES 5230  // NTC pull up resisior
#define NTC_OFFSET_VAL \
  (273.15 + 3)  // Kelvin to Celsius offset + calibration offset

/* PERIPHERALS */
#define EN_LOC_5V PF13
#define DIP_SW PD2  // or PD3 -> to check

/* AUDIO */
#define AUDIO_SHDN PB2
#define AUDIO_DAC_OUT PA4
#define AUDIO_DAC_CH OUT1

/* POWER BOARD */
#define PWR_BRD_GPIO_INPUT PD4   // PB5 on power board -> output push pull
#define PWR_BRD_GPIO_OUTPUT PD7  // PB8 on power board -> input
#define PWR_BRD_SERIAL Serial2
#define PWR_BRD_SERIAL_BAUDRATE 38400
#define PWR_BRD_SERIAL_RX PD6
#define PWR_BRD_SERIAL_TX PD5
#define PWR_BRD_SERIAL_CONFIG 0x06
#define PWR_BRD_SERIAL_TIMEOUT 1  // ms

/* SBC */
#define SBC_SERIAL Serial1
#define SBC_SERIAL_BAUDRATE 460800
#define SBC_SERIAL_RX PA10
#define SBC_SERIAL_TX PA9

/* IMU */
#define IMU_I2C I2C2
#define IMU_SDA PF0
#define IMU_SCL PF1
#define IMU_GPIO_IT PF2
#define IMU_SAMPLE_FREQ 25  // Hz
#define IMU_ID 0x37
#define IMU_ADDR_A 0x28
#define IMU_ADDR_B 0x29

/* MOTORS */
#define M1_ENC_TIM TIM1
#define M1_ENC_A PE9
#define M1_ENC_B PE11
#define M1_PWM_TIM TIM10
#define M1_PWM_PIN PF6
#define M1_PWM_TIM_CH 1
#define M1A_IN PE12
#define M1B_IN PE13
#define M1_DEFAULT_DIR -1  // 1 (CW) or -1 (CCW)

#define M2_ENC_TIM TIM2
#define M2_ENC_A PA15
#define M2_ENC_B PB3
#define M2_PWM_TIM TIM11
#define M2_PWM_PIN PF7
#define M2_PWM_TIM_CH 1
#define M2A_IN PG11
#define M2B_IN PG12
#define M2_DEFAULT_DIR 1  // 1 (CW) or -1 (CCW)

#define M3_ENC_TIM TIM3
#define M3_ENC_A PC6
#define M3_ENC_B PC7
#define M3_PWM_TIM TIM13
#define M3_PWM_PIN PF8
#define M3_PWM_TIM_CH 1
#define M3A_IN PG5
#define M3B_IN PG6
#define M3_DEFAULT_DIR -1  // 1 (CW) or -1 (CCW)

#define M4_ENC_TIM TIM4
#define M4_ENC_A PD12
#define M4_ENC_B PD13
#define M4_PWM_TIM TIM14
#define M4_PWM_PIN PF9
#define M4_PWM_TIM_CH 1
#define M4A_IN PD10
#define M4B_IN PD11
#define M4_DEFAULT_DIR 1  // 1 (CW) or -1 (CCW)

#define ILIM1 PE10
#define ILIM2 PG15
#define ILIM3 PG7
#define ILIM4 PD14

/* PIXEL LED */
#define PIXEL_SPI SPI2
#define PIXEL_MOSI PB15
#define PIXEL_SCK PB10
#define PIXEL_LENGTH 18
#define VIRTUAL_LED_LENGTH 10
#define PIXEL_SPI_SPEED 4000000
#define PIXEL_ANIMATION_FREQ 0.5

/* ETHERNET */
#define CLIENT_MAC_ADDR 0x02, 0x47, 0x00, 0x00, 0x00, 0x01
#define CLIENT_IP "192.168.77.3"
#define SBC_AGENT_IP "192.168.77.2"  // SBC
// #define SBC_AGENT_IP 	"192.168.77.5"	//External device
#define AGENT_PORT 8888
#define SHUTDOWN_PORT 3000

/* ETH LINK STATUS DEFINES */
#define ETH_LINK_STATUS_CONNECTED_BIT (1 << 0)  // if set - connected
#define ETH_LINK_STATUS_ERROR_BIT (1 << 1)

/* EEPROM */
#define EEPROM_BLOCK_ADDR_0 0x00
#define EEPROM_BLOCK_ADDR_1 0x01
#define EEPROM_BLOCK_ADDR_2 0x02
#define EEPROM_BLOCK_ADDR_3 0x03
#define EEPROM_BLOCK_ADDR_4 0x04
#define EEPROM_BLOCK_ADDR_5 0x05
#define EEPROM_BLOCK_ADDR_6 0x06
#define EEPROM_BLOCK_ADDR_7 0x07
#define EEPROM_DEV_ID 0x50
#define EEPROM_CONTROL_BYTE(DevId, BlockAddr) (DevId | BlockAddr)

// Board version eeprom defines
#define BOARD_VER_MEM_BLOCK 0x00
#define BOARD_VER_MEM_ADDR 0x00
#define BOARD_VER_MEM_SIZE 0x04
#define BOARD_VER_READ_ATTEMPTS 5

/* EXTERNAL PERIPHERALS */

// EXT SPI
#define EXT_SPI SPI1
#define EXT_SPI_SCK PA5
#define EXT_SPI_MISO PA6
#define EXT_SPI_MOSI PB5
// EXT I2C1
#define EXT_I2C1 I2C1
#define EXT_I2C1_SDA PB7
#define EXT_I2C1_SCL PB6
// EXT I2C2
#define EXT_I2C2 I2C3
#define EXT_I2C2_SDA PC9
#define EXT_I2C2_SCL PA8
// EXT Serial
#define EXT_SERIAL_EN_FLAG 1
#define EXT_SERIAL Serial6
#define EXT_SERIAL_BAUDRATE 115200
#define EXT_SERIAL_RX PG9
#define EXT_SERIAL_TX PG14
// EXT PWM1
#define EXT_PWM1_TIM TIM9
#define EXT_PWM1_CH CH1
#define EXT_PWM1_PIN PE5
// EXT PWM2
#define EXT_PWM2_TIM TIM9
#define EXT_PWM2_CH CH2
#define EXT_PWM2_PIN PE6
// EXT PWM3
#define EXT_PWM3_TIM TIM12
#define EXT_PWM3_CH CH1
#define EXT_PWM3_PIN PB14
// EXT ANALOG
#define EXT_ANALOG_IN1 PF10
#define EXT_ANALOG_IN2 PF3
// EXT GPIO
#define EXT_GPIO1 PG2
#define EXT_GPIO2 PG3
#define EXT_GPIO3 PG4

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

#endif /* HARDWARE_CFG_ROSBOT_XL */
