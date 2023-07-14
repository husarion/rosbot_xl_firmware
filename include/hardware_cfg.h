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

/* OTHERS */
#define DEFAULT_FIRMWARE_MODE  		2    	//0 - normal; 1 - error; 2 - debug
#define RTOS_FREQUENCY				1000	//hz
#define FREQ_TO_DELAY_TIME(freq)	(TickType_t)(RTOS_FREQUENCY/freq*portTICK_PERIOD_MS)

//POWER OFF
#define SBC_ETH_CONNECT_TIMEOUT	10			//ms
#define POWEROFF_DELAY			5000		//ms

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
#define PWR_BRD_GPIO_INPUT          PD4 //PB5 on power board -> output push pull
#define PWR_BRD_GPIO_OUTPUT         PD7 //PB8 on power board -> input
#define PWR_BRD_SERIAL              Serial2
#define PWR_BRD_SERIAL_BAUDRATE     38400
#define PWR_BRD_SERIAL_RX           PD6
#define PWR_BRD_SERIAL_TX           PD5
#define PWR_BRD_SERIAL_CONFIG       0x06
#define PWR_BRD_SERIAL_TIMEOUT      1   //ms

/* SBC */
#define SBC_SERIAL          	Serial1
#define SBC_SERIAL_BAUDRATE 	460800
#define SBC_SERIAL_RX       	PA10
#define SBC_SERIAL_TX       	PA9

/* IMU */
#define IMU_I2C             I2C2
#define IMU_SDA             PF0
#define IMU_SCL             PF1
#define IMU_GPIO_IT         PF2
#define IMU_SAMPLE_FREQ     25  //Hz
#define IMU_ID              0x37
#define IMU_ADDR_A          0x28
#define IMU_ADDR_B          0x29

/* PIXEL LED */
#define PIXEL_SPI           	SPI2  
#define PIXEL_MOSI          	PB15
#define PIXEL_SCK           	PB10
#define PIXEL_LENGTH        	18
#define VIRTUAL_LED_LENGTH  	10
#define PIXEL_SPI_SPEED     	4000000
#define PIXEL_ANIMATION_FREQ	0.5

/* ETHERNET */
#define CLIENT_IP 		"192.168.77.3"
#define SBC_AGENT_IP 	"192.168.77.2"	//SBC
// #define SBC_AGENT_IP 	"192.168.77.5"	//External device
#define AGENT_PORT 		8888
#define SHUTDOWN_PORT	3000

/* ETH LINK STATUS DEFINES */
#define ETH_LINK_STATUS_CONNECTED_BIT		(1 << 0)	//if set - connected
#define ETH_LINK_STATUS_ERROR_BIT			(1 << 1)

/* EEPROM */
#define EEPROM_BLOCK_ADDR_0     				0x00
#define EEPROM_BLOCK_ADDR_1     				0x01
#define EEPROM_BLOCK_ADDR_2     				0x02
#define EEPROM_BLOCK_ADDR_3     				0x03
#define EEPROM_BLOCK_ADDR_4     				0x04
#define EEPROM_BLOCK_ADDR_5     				0x05
#define EEPROM_BLOCK_ADDR_6     				0x06
#define EEPROM_BLOCK_ADDR_7     				0x07
#define EEPROM_DEV_ID							0x50
#define EEPROM_CONTROL_BYTE(DevId, BlockAddr)	(DevId | BlockAddr)

// Board version eeprom defines
#define BOARD_VER_MEM_BLOCK			0x00
#define BOARD_VER_MEM_ADDR			0x00
#define BOARD_VER_MEM_SIZE			0x04
#define BOARD_VER_READ_ATTEMPTS		5

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
#define EXT_SERIAL_EN_FLAG      1
#define EXT_SERIAL              Serial6
#define EXT_SERIAL_BAUDRATE     115200
#define EXT_SERIAL_RX           PG9
#define EXT_SERIAL_TX           PG14
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


/* WATCHDOG */
#define WATCHDOG_TIMEOUT	15000000	//microseconds

/* BATTERY */

#define BATTERY_CELLS_SERIES							3
#define BATTERY_CELLS_PARALLEL							3
#define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE 	1	// in unmeasured
#define BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE 		1	// in unmeasured
// #define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE 	(BATTERY_CELLS_PARALLEL * BATTERY_CELLS_SERIES)	
// #define BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE 		(BATTERY_CELLS_PARALLEL * BATTERY_CELLS_SERIES)

typedef enum{
	unknown_status 	= 0,
	charging 		= 1,
	discharging 	= 2,
	not_charging 	= 3,
	full 			= 4
}BatteryStatusTypeDef;

typedef enum{
	unknown_health 			= 0,
	good 					= 1,
	overhaet 				= 2,
	dead 					= 3,
	overvoltage 			= 4,
	unspec_failure 			= 5,
	cold 					= 6,
	watchdog_timer_expire 	= 7,
	safety_timer_expire 	= 8
}BatteryHealthTypeDef;

typedef enum{
	unknown_type = 0,
	NIMH = 1,
	LION = 2,
	LIPO = 3,
	LIFE = 4,
	NICD = 5,
	LIMN = 6
}BatteryTechnologyTypeDef;

typedef struct{
	//ROS battery msgs variables
	float						voltage;
	float						temperature;
	float						current;
	float						charge_current;
	float						capacity;
	float						design_capacity;
	float						percentage;
	float						cell_temperature[BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE];
	float						cell_voltage[BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE];
	BatteryStatusTypeDef 		status;
	BatteryHealthTypeDef		health;
	BatteryTechnologyTypeDef 	technology;
	bool						present;
}battery_state_queue_t;

/* FIRMWARE MODE */

typedef enum{
	fw_normal 	= 0,
	fw_error 	= 1,
	fw_debug 	= 2
}FirmwareModeTypeDef;


#endif /* HARDWARE_CFG */