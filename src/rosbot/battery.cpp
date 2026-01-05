#include <rosbot/battery.hpp>

#include <Arduino.h>

#include "bsp.hpp"

namespace battery
{
constexpr float BATTERY_LOW_VOLTAGE = 10.8;
constexpr float BATTERY_LOW_HYST = 0.2;
constexpr float V_REF = 3.3;
constexpr float VIN_MEAS_CORRECTION = 0.986;
constexpr float UPPER_RESISTOR = 5.6e4;
constexpr float LOWER_RESISTOR = 1.0e4;
constexpr float V_MIN = 9.6;
constexpr float V_MAX = 12.6;
constexpr int LED_INTERVAL_MS = 500;


battery_data_t loop()
{
  static unsigned long last_toggle_time = 0;
  static bool low_battery = false;

  battery_data_t battery_data = readBattery();

  if (battery_data.voltage < BATTERY_LOW_VOLTAGE)
  {
    low_battery = true;
  }
  else if (battery_data.voltage > BATTERY_LOW_VOLTAGE + BATTERY_LOW_HYST)
  {
    low_battery = false;
    SetRedLed(On);
  }

  if (low_battery)
  {
    if (millis() - last_toggle_time > LED_INTERVAL_MS)
    {
      SetRedLed(Toggle);
      last_toggle_time = millis();
    }
  }

  return battery_data;
}


battery_data_t readBattery()
{
  battery_data_t battery_data;
  float raw = analogRead(BATTERY_ADC_PIN) / 1023.0f;
  battery_data.voltage = V_REF * VIN_MEAS_CORRECTION * (UPPER_RESISTOR + LOWER_RESISTOR) / LOWER_RESISTOR  * raw;
  battery_data.current = NAN;
  battery_data.temperature = NAN;
  return battery_data;
}

float percentage(float voltage) {
  if (voltage < V_MIN) voltage = V_MIN;
  if (voltage > V_MAX) voltage = V_MAX;
  
  float perc = (voltage - V_MIN) / (V_MAX - V_MIN); // linear scaling

  return perc;
}

}  // namespace battery