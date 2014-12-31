//
// WARNING: repeated starts in slave mode are BROKEN in current release of Arduino.
// You must comment out the call to twi_stop() in
// C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\Wire\utility\twi.c
//
//     case TW_SR_STOP: // stop or repeated start condition received
//      // put a null char after data if there's room
//      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
//        twi_rxBuffer[twi_rxBufferIndex] = '\0';
//      }
//      // sends ack and stops interface for clock stretching
// ---> //twi_stop();                                          <--- COMMENT OUT THIS LINE TO FIX REPEATED STARTS IN SLAVE MODE!
//      // callback to user defined callback
//      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
//      // since we submit rx buffer to "wire" library, we can reset it
//      twi_rxBufferIndex = 0;
//      // ack future responses and leave slave receiver state
//      twi_releaseBus();
//      break;
//    case TW_SR_DATA_NACK:       // data received, returned nack
// 

#include <Arduino.h>
#include <Wire.h>

#define SLAVE_ADDR 0x42

// provide 128 bytes of storage in our virtual eeprom
byte storage[128];

// current EEPROM address
unsigned int address = 0;

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
    Serial.println("request");
    // fill buffer
    Wire.write(&storage[address], min(sizeof(storage) - address, BUFFER_LENGTH));
    address = 0;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int count)
{
    Serial.println("receive");
    // first two bytes are slave address {HI LO}
    // ignore all other bytes
    int adrHi = Wire.read();
    int adrLo = Wire.read();
    if ((adrHi != -1) && (adrLo != -1))
    {
        // set current address pointer
        unsigned int newAddress = ((adrHi & 0xff) << 8) | (adrLo & 0xff);
        if (newAddress < sizeof(storage))
        {
            address = newAddress;
        }
    }

    // write remaining bytes to storage at address
    int c;
    while ((c = Wire.read()) != -1)
    {
        if (address < sizeof(storage))
        {
            storage[address] = uint8_t(c);
            ++address;
        }
    }
}


void setup()
{
    Serial.begin(115200);

    for (unsigned int i = 0; i < sizeof(storage); ++i)
    {
        storage[i] = i;
    }

    //Wire.begin();

    // join bus as slave
    Wire.begin(SLAVE_ADDR);
    //  Wire.setClock(100000);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    pinMode(13, OUTPUT);
    interrupts();
}

void loop()
{
    Serial.println("Hello");

    //  Wire.beginTransmission(0x10);
    //  Wire.write(0xaa);
    //  Wire.write(0xbb);
    //  Wire.endTransmission();

    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
}
