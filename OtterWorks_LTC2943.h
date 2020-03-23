// https://www.analog.com/media/en/technical-documentation/data-sheets/2943fa.pdf

#ifndef __OW_LTC2943_H__
#define __OW_LTC2943_H__

#include "Arduino.h"

#include <Adafruit_Sensor.h> // use Adafruit's nice unified sensor framework
#include <Wire.h>

#define _I2C_ALERT_RESPONSE (0x0c) // TODO: do we need/want this?

class OtterWorks_LTC2943; // forward declaration for unified sensor framework

class OtterWorks_LTC2943_Temperature : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Temperature( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *_theLTC2943 = NULL;
};

class OtterWorks_LTC2943_Charge : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Charge( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *_theLTC2943 = NULL;
};

class OtterWorks_LTC2943_Potential : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Potential( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *_theLTC2943 = NULL;
};

class OtterWorks_LTC2943_Current : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Current( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *_theLTC2943 = NULL;
};

class OtterWorks_LTC2943 {
  public:
    enum { // PRESCALAR
        PRESCALAR_1 = 0b000,
        PRESCALAR_4 = 0b001,
        PRESCALAR_16 = 0b010,
        PRESCALAR_64 = 0b011,
        PRESCALAR_256 = 0b100,
        PRESCALAR_1024 = 0b101,
        PRESCALAR_4096 = 0b110,
        PRESCALAR_4096_2 = 0b111
    };
    enum { // MODE
        SLEEP = 0b00,
        MANUAL = 0b01,
        SCAN = 0b10,
        AUTO = 0b11
    };

    struct config {
        uint8_t none : 1; // unused, don't set
        uint8_t charge_complete : 1;
        uint8_t alert : 1;
        uint8_t prescalar : 3;
        uint8_t mode : 2;
        uint8_t get() { // TODO: Do you really need a get method?
          return ( mode << 6 | prescalar << 3 | alert << 2 | charge_complete << 1 | none );
        }
    };
    config _configReg;

    OtterWorks_LTC2943();
    ~OtterWorks_LTC2943(void);

    bool begin( float resistance, uint16_t prescalar, TwoWire *theWire = &Wire );
    bool init();

    float readTemperature(void);
    float readCharge(void);
    float readPotential(void);
    float readCurrent(void);

    Adafruit_Sensor *getTemperatureSensor(void);
    Adafruit_Sensor *getChargeSensor(void);
    Adafruit_Sensor *getPotentialSensor(void);
    Adafruit_Sensor *getCurrentSensor(void);

  protected:
    TwoWire *_wire;

    OtterWorks_LTC2943_Temperature *temperature_sensor = NULL;
    OtterWorks_LTC2943_Charge *charge_sensor = NULL;
    OtterWorks_LTC2943_Potential *potential_sensor = NULL;
    OtterWorks_LTC2943_Current *current_sensor = NULL;

    const uint8_t _i2caddr = 0b1100100;
    // ^ hard-coded I2C address 0b1100100 = 0x64 = 100
    // (see p. 15 of datasheet)
    const byte _charge_register = 0x02;
    const byte _potential_register = 0x08;
    const byte _current_register = 0x0e;
    const byte _temperature_register = 0x14;

    int32_t _sensorID;
    float _resistance;
    uint16_t _prescalar;
    // _prescalar value should match PRESCALAR_MODE; example has both 4096
    // so we should probably get it by reading the register and using the enum
    const struct {
        float temperature;
        float charge;
        float potential;
        float current;
    } _conversion_constant = {
        .temperature = 510,
        .charge = 0.34e-3,
        .potential = 23.6,
        .current = 60e-3
    };

    void write8( byte reg, byte value );
    uint8_t read8( byte reg );
    uint16_t read16( byte reg );
};

// extern OtterWorks_LTC2943 LTC2943;
// ^ a named instance for use like Arduino's Serial
// TODO: Why doesn't Adafruit do this for their BME280 library?

#endif // __OW_LTC2943_H__
