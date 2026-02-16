// Copyright 2022 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>

#include "battery_adc.hpp"
#include "battery_interface.hpp"
#include "config.hpp"
#include "control/motors_manager.hpp"
#include "encoder_array.hpp"
#include "hardware_encoder.hpp"
#include "imu_bno055.hpp"
#include "led_indicator.hpp"
#include "range_array.hpp"
#include "range_vl53l0.hpp"
#include "rtos.hpp"
#include "serial_manager.hpp"
#include "uros.hpp"

// Battery
ADCConfig battery_adc_config = {.adc_pin = BATTERY_ADC_PIN,
                                .v_ref = BATTERY_VREF,
                                .v_min = BATTERY_VMIN,
                                .v_max = BATTERY_VMAX,
                                .divider = BATTERY_DIVIDER,
                                .correction = BATTERY_CORRECTION};
BatteryAdc battery_impl(battery_adc_config);

// Encoders
static HardwareEncoder enc_fl({
    .pin_a = ENC_FL_PIN_A,
    .pin_b = ENC_FL_PIN_B,
    .timer = ENC_FL_TIMER,
    .dir_cw = ENC_FL_DIR_CW,
    .rad_per_tick = RAD_PER_TICK,
    .label = "fl",
});

static HardwareEncoder enc_fr({
    .pin_a = ENC_FR_PIN_A,
    .pin_b = ENC_FR_PIN_B,
    .timer = ENC_FR_TIMER,
    .dir_cw = ENC_FR_DIR_CW,
    .rad_per_tick = RAD_PER_TICK,
    .label = "fr",
});

static HardwareEncoder enc_rl({
    .pin_a = ENC_RL_PIN_A,
    .pin_b = ENC_RL_PIN_B,
    .timer = ENC_RL_TIMER,
    .dir_cw = ENC_RL_DIR_CW,
    .rad_per_tick = RAD_PER_TICK,
    .label = "rl",
});

static HardwareEncoder enc_rr({
    .pin_a = ENC_RR_PIN_A,
    .pin_b = ENC_RR_PIN_B,
    .timer = ENC_RR_TIMER,
    .dir_cw = ENC_RR_DIR_CW,
    .rad_per_tick = RAD_PER_TICK,
    .label = "rr",
});
static EncoderInterface* encoders[] = {&enc_fl, &enc_fr, &enc_rl, &enc_rr};
static constexpr uint8_t ENCODER_COUNT = sizeof(encoders) / sizeof(encoders[0]);

// IMU
TwoWire imu_i2c(IMU_I2C_SDA, IMU_I2C_SCL);
Bno055Config imu_bno055_config = {
    .bus = &imu_i2c,
    .i2c_addr = IMU_ADDR_B,
    .sensor_id = IMU_ID,
    .int_pin = IMU_INT,
    .axis_config = Adafruit_BNO055::REMAP_CONFIG_P0,
};
ImuBno055 imu_impl(imu_bno055_config);

// Range sensors
TwoWire range_i2c(RANGE_I2C_SDA, RANGE_I2C_SCL);
RangeVl53l0x range_fl(&range_i2c, RANGE_XSHUT_FL, 0x30);
RangeVl53l0x range_fr(&range_i2c, RANGE_XSHUT_FR, 0x31);
RangeVl53l0x range_rl(&range_i2c, RANGE_XSHUT_RL, 0x32);
RangeVl53l0x range_rr(&range_i2c, RANGE_XSHUT_RR, 0x33);
static RangeInterface* range_sensors[] = {
    &range_fl,
    &range_fr,
    &range_rl,
    &range_rr,
};
static constexpr uint8_t RANGE_COUNT =
    sizeof(range_sensors) / sizeof(range_sensors[0]);

/* EXTERN VARIABLES */
log_level_t g_firmware_log_level = LOG_LEVEL_DEBUG;

BatteryInterface* g_battery = &battery_impl;
EncoderArray g_encoders(encoders, ENCODER_COUNT);
ImuInterface* g_imu = &imu_impl;
RangeArray g_ranges(range_sensors, RANGE_COUNT);

SerialManager serialManager;

void BoardPheripheralsInit() {
  // Initialize Buttons
  pinMode(PUSH_BUTTON1, INPUT_PULLUP);
  pinMode(PUSH_BUTTON2, INPUT_PULLUP);

  // Initialize LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(GRN_LED2, OUTPUT);
  digitalWrite(RED_LED, HIGH);

  // Enable power for IMU sensor
  pinMode(IMU_POWER_ON, OUTPUT);
  digitalWrite(IMU_POWER_ON, HIGH);

  // Initialize I2C
  imu_i2c.begin();
  imu_i2c.setClock(400000);
  range_i2c.begin();
  range_i2c.setClock(400000);

  delay(20);
}

/*==================== SETUP ========================*/
void setup() {
  // Peripherals initialization
  BoardPheripheralsInit();

  // Pre-communication
  serialManager.init();
  const auto& selected_serial = serialManager.selectActive();
  serialManager.configureNamespace();

  // Sensors initialization
  battery_impl.init();
  g_encoders.init();
  imu_impl.init();
  ledIndicator.init(RED_LED, HIGH);
  motors.init();
  g_ranges.init();
  u_ros::transportInit(selected_serial);

  // RTOS
  rtos::createQueues();
  rtos::createTasks();
  vTaskStartScheduler();
}

/*============== LOOP ===============*/
void loop() {}

/*=========== Runtime stats ====================*/
HardwareTimer RunTimeStatsTimer(TIM5);

void vConfigureTimerForRunTimeStats(void) {
  RunTimeStatsTimer.setPrescaleFactor(
      1680);  // every 10 µs (168MHz / 1680 = 100kHz)
  RunTimeStatsTimer.setOverflow(0xFFFFFFFF);
  RunTimeStatsTimer.refresh();
  RunTimeStatsTimer.resume();
}

uint32_t vGetTimerValueForRunTimeStats(void) {
  return RunTimeStatsTimer.getCount();
}
