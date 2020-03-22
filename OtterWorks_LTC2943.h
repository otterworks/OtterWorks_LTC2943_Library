// https://www.analog.com/media/en/technical-documentation/data-sheets/2943fa.pdf

#ifndef __OW_LTC2943_H__
#define __OW_LTC2943_H__

#include "Arduino.h"

#include <Adafruit_Sensor.h> // use Adafruit's nice unified sensor framework
#include <Wire.h>

#define LTC2943_ADDRESS (0x64)

#define _I2C_ALERT_RESPONSE (0x0c) // TODO: do we need/want this?
#define _DISABLE_ALCC_PIN (0x00) // TODO: do we need/want this?

// TODO: copy register addresses from bayarveli Linduino library

class OtterWorks_LTC2943 {
  public:
    enum {
        SLEEP = 0x00,
        SHUTDOWN = 0x01,
        CHARGE_COMPLETE = 0x02,
        ALERT = 0x04,
        MANUAL = 0x40,
        SCAN = 0x80,
        AUTO = 0xc0
    };
    enum prescalar_magnitude {
        PRESCALAR_1 = 0x00,
        PRESCALAR_4 = 0x80,
        PRESCALAR_16 = 0x10,
        PRESCALAR_64 = 0x18,
        PRESCALAR_256 = 0x20,
        PRESCALAR_1024 = 0x28,
        PRESCALAR_4096 = 0x30,
        PRESCALAR_4096_2 = 0x31
    };

    OtterWorks_LTC2943();
    ~OtterWorks_LTC2943(void);

    bool begin( uint8_t = LTC2943_ADDRESS, TwoWire *theWire = &Wire );
    bool init();

    float readTemperature(void);
    float readCharge(void);
    float readPotential(void);
    float readCurrent(void);

    uint32_t sensorID(void);

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

    void write8( byte reg, byte value );
    void write16( byte reg, byte value );
    uint16_t read16_LE( byte reg );
    int16_t readS16_LE( byte reg );

    const uint8_t _i2caddr = 0x64;
    // ^ hard-coded I2C address 0b1100100 = 0x64 = 100
    // (see p. 15 of datasheet)

    int32_t _sensorID;
    float _resistance; // TODO: where do we want to set this?
    uint16_t _prescalar; // TODO: where to we set/get this?

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

    // TODO: struct for each register, as in BME280 example
    //A status_register;
    //B control_register;
}

class OtterWorks_LTC2943_Temperature : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Temperature( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *theLTC2943 = NULL;
}

class OtterWorks_LTC2943_Charge : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Charge( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *theLTC2943 = NULL;
}

class OtterWorks_LTC2943_Potential : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Potential( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *theLTC2943 = NULL;
}

class OtterWorks_LTC2943_Current : public Adafruit_Sensor {
  public:
    OtterWorks_LTC2943_Current( OtterWorks_LTC2943 *parent ) {
        _theLTC2943 = parent;
    }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);
  private:
    int _sensorID = 2943;
    OtterWorks_LTC2943 *theLTC2943 = NULL;
}

extern OtterWorks_LTC2943 LTC2943;
// ^ a named instance for use like Arduino's Serial
// TODO: Why doesn't Adafruit do this for their BME280 library?

#endif // __OW_LTC2943_H__
