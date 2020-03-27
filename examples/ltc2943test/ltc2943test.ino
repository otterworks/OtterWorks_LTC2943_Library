#include <OtterWorks_LTC2943.h>

OtterWorks_LTC2943 ltc2943;

void setup() {
    Serial.begin(9600);
    while(!Serial); // wait
    Serial.println("OtterWorks LTC2943 Library Test");

    if(!ltc2943.begin(4e-3, 4096)) {
        Serial.println("No LTC2943 found on I2C bus.");
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
    Serial.print(ltc2943.readTemperature());
    Serial.println(" degC");

    Serial.print("Electric charge = ");
    Serial.print(ltc2943.readCharge());
    Serial.println(" Coulombs");

    Serial.print("Electric potential = ");
    Serial.print(ltc2943.readPotential());
    Serial.println(" Volts");

    Serial.print("Electric current = ");
    Serial.print(ltc2943.readCurrent());
    Serial.println(" Amps");
}
