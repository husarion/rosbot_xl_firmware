#include "range_array.hpp"

#include <Arduino.h>

RangeArray::RangeArray(RangeInterface** sensors, uint8_t count)
    : sensors_(sensors), count_(count) {
  if (count_ > MAX_NUM_RANGE_SENSORS) {
    count_ = MAX_NUM_RANGE_SENSORS;
  }
  data_.count = count_;
}

void RangeArray::init() {
  for (uint8_t i = 0; i < count_; i++) {
    sensors_[i]->powerOff();
  }
  for (uint8_t i = 0; i < count_; i++) {
    sensors_[i]->init();
  }
}

void RangeArray::update() {
  for (uint8_t i = 0; i < count_; i++) {
    sensors_[i]->update();
    data_.range[i] = sensors_[i]->getData().range;
  }
}
