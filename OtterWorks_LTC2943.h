// https://www.analog.com/media/en/technical-documentation/data-sheets/2943fa.pdf

#ifndef __OW_LTC2943_H__
#define __OW_LTC2943_H__

#include "Arduino.h"

#include <Adafruit_Sensor.h> // use Adafruit's nice unified sensor framework
#include <Wire.h>

#define LTC2943_ADDRESS (0x64) // hard-coded I2C address b1100100 = 0x64 = 100 (see p. 15 of datasheet)

// TODO: copy register addresses from bayarveli Linduino library

class OtterWorks_LTC2943 {
  public:
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

    uint8_t _i2caddr;
    int32_t _sensorID;
    float _sense_resistance; // TODO: where do we want to set this?

    // TODO: struct for each register, as in BME280 example
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
