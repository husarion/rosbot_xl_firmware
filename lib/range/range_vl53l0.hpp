#pragma once

#include <VL53L0X.h>
#include <Wire.h>

#include "range_interface.hpp"

struct Vl53l0xConfig {
  uint16_t timeout_ms = 500;
  float signal_rate_limit = 0.1f;
  uint8_t vcsel_pre_range = 16;
  uint8_t vcsel_final_range = 12;
  uint32_t timing_budget_us = 50000;
  uint32_t continuous_period_ms = 100;
};

class RangeVl53l0x : public RangeInterface {
 public:
  RangeVl53l0x(TwoWire* bus, uint8_t xshut_pin, uint8_t i2c_address,
               const Vl53l0xConfig& config = {});

  void init() override;
  void update() override;
  void powerOff() override;
  void powerOn() override;
  const char* name() const override { return "VL53L0X"; }

 private:
  TwoWire* bus_;
  VL53L0X driver_;
  uint8_t xshut_pin_;
  uint8_t address_;
  const Vl53l0xConfig config_;
  bool initialized_ = false;
};
