#include <OtterWorks_LTC2943.h> // includes <Wire.h> for I2C

void setup() {
    Serial.begin(9600);
    while(!Serial); // wait
    Serial.println("OtterWorks LTC2943 Library Test");

    if(!LTC2943.begin()) {
        Serial.println("No LTC2943 foind on I2C bus.");
        Serial.print("Expected sensor to answer at ID 0x");
        Serial.println(LTC2943.sensorID(), HEX);
        while(true) {
            delay(10);
        }
    } else {
        Serial.println("Found LTC2943");
        Serial.println("~~~~~~~~~~~~~");
    }
}

void loop() {
    Serial.println("~~~~~~~~~~~~~");
    printBatteryStatus();
    Serial.println("~~~~~~~~~~~~~");
    delay(1000);
}

void printBatteryStatus() {
    Serial.print("Temperature = ");
    Serial.print(LTC2943.temperature());
    Serial.println(" degC");

    Serial.print("Electric charge = ");
    Serial.print(LTC2943.charge());
    Serial.println(" Coulombs");

    Serial.print("Electric potential = ");
    Serial.print(LTC2943.potential());
    Serial.println(" Volts");

    Serial.print("Electric current = ");
    Serial.print(LTC2943.current());
    Serial.println(" Amps");
}
