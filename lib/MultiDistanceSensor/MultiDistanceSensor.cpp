#include "MultiDistanceSensor.hpp"

static const uint8_t DEFAULT_HW_ADDRESS = 0x29;
static const uint8_t SENSOR_HW_ADDRESS[NUM_DISTANCE_SENSORS] = {
    DEFAULT_HW_ADDRESS + 1, DEFAULT_HW_ADDRESS + 2, DEFAULT_HW_ADDRESS + 3,
    DEFAULT_HW_ADDRESS + 4};

MultiDistanceSensor::MultiDistanceSensor()
    : _initialized(false),
      _sensorsEnabled(false),
      _lastSensorIndex(-1),
      _updateInterval(10),
      _lastUpdateTime(0) {
  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) _isActive[i] = false;
}

MultiDistanceSensor::~MultiDistanceSensor() { stop(); }

MultiDistanceSensor& MultiDistanceSensor::getInstance() {
  static MultiDistanceSensor instance;
  return instance;
}

int MultiDistanceSensor::init(uint8_t xshutPins[NUM_DISTANCE_SENSORS]) {
  if (_initialized) return 0;

  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    _xshutPins[i] = xshutPins[i];
    pinMode(_xshutPins[i], OUTPUT);
    digitalWrite(_xshutPins[i], LOW);
    _isActive[i] = false;
  }
  delay(10);

  Wire.begin();
  _initialized = true;

  return restartSensors();
}

int MultiDistanceSensor::restartSensors() {
  int result = 0;

  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    digitalWrite(_xshutPins[i], LOW);
    _isActive[i] = false;
  }
  delay(10);

  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    digitalWrite(_xshutPins[i], HIGH);
    delay(10);
    _sensors[i].init();
    _sensors[i].setAddress(SENSOR_HW_ADDRESS[i]);
    _sensors[i].setTimeout(500);
    _sensors[i].setMeasurementTimingBudget(80000);  // us
    _sensors[i].startContinuous(100);               // ms
    _isActive[i] = true;
    _lastSensorIndex = i;
    result++;
  }

  _sensorsEnabled = true;
  return result;
}

void MultiDistanceSensor::start() {
  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    if (_isActive[i]) _sensors[i].startContinuous(100);
  }
  _sensorsEnabled = true;
}

void MultiDistanceSensor::stop() {
  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    if (_isActive[i]) _sensors[i].stopContinuous();
  }
  _sensorsEnabled = false;
}

void MultiDistanceSensor::runMeasurement() {
  if (_lastSensorIndex == -1) {
    _m.status = ERR_NOT_INIT;
    return;
  }

  _m.timestamp = millis();
  _m.status = ERR_NONE;

  for (int i = 0; i < NUM_DISTANCE_SENSORS; i++) {
    if (_isActive[i]) {
      uint16_t range = _sensors[i].readRangeContinuousMillimeters();
      if (_sensors[i].timeoutOccurred()) {
        _m.range[i] = -1;
        _isActive[i] = false;
        _m.status = ERR_I2C_FAILURE;
      } else {
        _m.range[i] = range / 1000.0f;  // meters
      }
    } else {
      _m.range[i] = -1;
    }
  }
}

void MultiDistanceSensor::update() {
  if (millis() - _lastUpdateTime >= _updateInterval) {
    _lastUpdateTime = millis();
    if (_sensorsEnabled) runMeasurement();
  }
}

void MultiDistanceSensor::readMeasurements(SensorsMeasurement& measurement) {
  measurement = _m;
}
