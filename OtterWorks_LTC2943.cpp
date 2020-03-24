#include "OtterWorks_LTC2943.h" // includes Wire.h
#include <limits.h>

OtterWorks_LTC2943::OtterWorks_LTC2943(){};

OtterWorks_LTC2943::~OtterWorks_LTC2943( void ) {
  if(temperature_sensor) {
    delete temperature_sensor;
  }
  if(charge_sensor) {
    delete charge_sensor;
  }
  if(potential_sensor) {
    delete potential_sensor;
  }
  if(current_sensor) {
    delete current_sensor;
  }
};

bool OtterWorks_LTC2943::begin( float resistance, uint16_t prescaler, TwoWire *theWire ) {
  _resistance = resistance;
  _prescaler = prescaler;
  _wire = theWire;
  return init();
}

bool OtterWorks_LTC2943::init() {
  _wire->begin();
  _configuration.f.mode = MODE_SCAN;
  switch (_prescaler) {
    case 4096:
      _configuration.f.prescaler = PRESCALER_4096;
      break;
    case 1024:
      _configuration.f.prescaler = PRESCALER_1024;
      break;
    case 256:
      _configuration.f.prescaler = PRESCALER_256;
      break;
    case 64:
      _configuration.f.prescaler = PRESCALER_64;
      break;
    case 16:
      _configuration.f.prescaler = PRESCALER_16;
      break;
    case 4:
      _configuration.f.prescaler = PRESCALER_4;
      break;
    case 1:
    default:
      _configuration.f.prescaler = PRESCALER_1;
      break;
  }
  _configuration.f.charge_complete = 0;
  _configuration.f.alert = 0;
  _configuration.f.shutdown = 0;
  Serial.print( "Writing configuration: 0b" );
  Serial.print( _configuration.b, BIN );
  Serial.print( " to control register: 0x" );
  Serial.print( _control_register, HEX );
  Serial.println();
  write8( _control_register, _configuration.b );
  // TODO: consider reading it back
  return true;
}

void OtterWorks_LTC2943::write8(byte reg, byte value) {
  _wire->beginTransmission((uint8_t)_i2caddr);
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)value);
  _wire->endTransmission();
}

uint8_t OtterWorks_LTC2943::read8(byte reg) {
  uint8_t value;
  _wire->beginTransmission((uint8_t)_i2caddr);
  _wire->write((uint8_t)reg);
  _wire->endTransmission();
  _wire->requestFrom((uint8_t)_i2caddr, (byte)1);
  value = _wire->read();
  return value;
}

uint16_t OtterWorks_LTC2943::read16(byte reg) {
  uint16_t value;
  _wire->beginTransmission((uint8_t)_i2caddr);
  _wire->write((uint8_t)reg);
  _wire->endTransmission();
  _wire->requestFrom((uint8_t)_i2caddr, (byte)2);
  value = (_wire->read() << 8) | _wire->read();
  // TODO: swap reads ^ if I got the endianness backward
  //       LTC2943 communicates BE, Arduino operates LE
  return value;
}

float OtterWorks_LTC2943::readTemperature(void) {
  float x = (float)read16(_temperature_register);
  return (_conversion_constant.temperature/USHRT_MAX)*x - 273.15;
}

float OtterWorks_LTC2943::readCharge(void) {
  uint16_t u = read16(_charge_register);
  float x = (3600.0 / 4096.0) * 50e-3 * _prescaler * u;
  //TODO: name magic numbers ^
  return (_conversion_constant.charge/_resistance)*x;
}

float OtterWorks_LTC2943::readPotential(void) {
  float x = (float)read16(_potential_register);
  return (_conversion_constant.potential/USHRT_MAX)*x;
}

float OtterWorks_LTC2943::readCurrent(void) {
  uint16_t u = read16(_current_register);
  float x = (float)(u - SHRT_MAX)/(SHRT_MAX);
  return (_conversion_constant.current/_resistance)*x;
}

Adafruit_Sensor *OtterWorks_LTC2943::getTemperatureSensor(void) {
  if (!temperature_sensor) {
    temperature_sensor = new OtterWorks_LTC2943_Temperature(this);
  }
  return temperature_sensor;
}

void OtterWorks_LTC2943_Temperature::getSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LTC2943", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_OBJECT_TEMPERATURE;
  sensor->min_delay = 8e-3;
  sensor->min_value = -40.0; // minimum ambient operating temperature
  sensor->max_value = +85.0; // maximum ambient operating temperature
  sensor->resolution = 0.061; // 11 bits
}

bool OtterWorks_LTC2943_Temperature::getEvent(sensors_event_t *event) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_OBJECT_TEMPERATURE;
  event->timestamp = millis();
  event->temperature = _theLTC2943->readTemperature();
  return true;
}

Adafruit_Sensor *OtterWorks_LTC2943::getChargeSensor(void) {
  if (!charge_sensor) {
    charge_sensor = new OtterWorks_LTC2943_Charge(this);
  }
  return charge_sensor;
}

void OtterWorks_LTC2943_Charge::getSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LTC2943", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = 0; // Adafruit_Sensor does not define a type for CHARGE
  sensor->min_delay = 0;
  sensor->min_value = 0.0;
  sensor->max_value = 3.6e6; // 1000 Ah
  sensor->resolution = 0.01;
}

bool OtterWorks_LTC2943_Charge::getEvent(sensors_event_t *event) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = 0; // Adafruit_Sensor does not define a type for CHARGE
  event->timestamp = millis();
  // event->charge = _theLTC2943->readCharge();
  return true;
}

Adafruit_Sensor *OtterWorks_LTC2943::getPotentialSensor(void) {
  if (!potential_sensor) {
    potential_sensor = new OtterWorks_LTC2943_Potential(this);
  }
  return potential_sensor;
}

void OtterWorks_LTC2943_Potential::getSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LTC2943", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_VOLTAGE;
  sensor->min_delay = 48e-3;
  sensor->min_value = +3.6; // minimum operating voltage
  sensor->max_value = +20.0; // maximum operating voltage
  sensor->resolution = 1e-3; // depends on sense resistor & prescaler
}

bool OtterWorks_LTC2943_Potential::getEvent(sensors_event_t *event) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_VOLTAGE;
  event->timestamp = millis();
  event->voltage = _theLTC2943->readPotential();
  return true;
}

Adafruit_Sensor *OtterWorks_LTC2943::getCurrentSensor(void) {
  if (!current_sensor) {
    current_sensor = new OtterWorks_LTC2943_Current(this);
  }
  return current_sensor;
}

void OtterWorks_LTC2943_Current::getSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LTC2943", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_CURRENT;
  sensor->min_delay = 8e-3;
  sensor->min_value = -4.0; // depends on sense resistor & prescaler
  sensor->max_value = +4.0; // depends on sense resistor & prescaler
  sensor->resolution = 1e-3; // depends on sense resistor & prescaler
}

bool OtterWorks_LTC2943_Current::getEvent(sensors_event_t *event) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_CURRENT;
  event->timestamp = millis();
  event->current = _theLTC2943->readCurrent();
  return true;
}
