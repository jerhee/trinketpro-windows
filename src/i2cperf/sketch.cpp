//
// Measure bus utilization of WriteRead transfers
//
#include <Arduino.h>
#include <Wire.h>

const uint8_t SlaveAddress = 0x57;
const unsigned long NumBytes = 1024;
const unsigned long DataLengths[] = {1, 8, 32};

void setup ()
{
    Serial.begin(115200);
    Wire.begin();
}

void measureUtilization (unsigned long ReadLength)
{
    unsigned int numTransfers = NumBytes / ReadLength;
    unsigned long start = micros();
    for (unsigned int i = 0; i < numTransfers; ++i) {
        Wire.beginTransmission(SlaveAddress);
        Wire.write(0);
        Wire.endTransmission(false);
        delayMicroseconds(1);   // required for EEPROM to ACK
        Wire.requestFrom(SlaveAddress, ReadLength);
        if (!Wire.available()) {
            Serial.println("Failed to read from slave device");
            return;
        }
        delayMicroseconds(1);   // required for EEPROM to ACK
    }
    unsigned long end = micros();
    
    unsigned long clocksRequired = ((numTransfers - 1) * 2) +
    (numTransfers * (((ReadLength + 3) * 9) + 4));

    double timeInSecs = (end - start) / 1000000.0;
    double clocksUsed = 100000.0 * timeInSecs;
    double utilization = 100.0 * clocksRequired / clocksUsed;
    Serial.print("Bus utilization for ReadLength=");
    Serial.print(ReadLength);
    Serial.print(" is ");
    Serial.print(utilization);
    Serial.println("%");
}

void loop ()
{
    delay(2000);
    
    for (unsigned int i = 0; i <  sizeof(DataLengths)/sizeof(*DataLengths); ++i) {
        measureUtilization(DataLengths[i]);
        delay(1);
    }
}
