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
#include <Spi.h>

#define SLAVE_ADDR 0x42

// provide 128 bytes of storage in our virtual eeprom
byte storage[128];

// current EEPROM address
unsigned int address = 0;

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
    digitalWrite(LED_BUILTIN, HIGH);
    // fill buffer
    Wire.write(&storage[address], min(sizeof(storage) - address, BUFFER_LENGTH));
    address = 0;
    digitalWrite(LED_BUILTIN, LOW);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int count)
{
    digitalWrite(LED_BUILTIN, HIGH);
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
    unsigned int tempAddr = address;
    while ((c = Wire.read()) != -1)
    {
        if (tempAddr < sizeof(storage))
        {
            storage[tempAddr] = uint8_t(c);
            ++tempAddr;
        }
    }
    digitalWrite(LED_BUILTIN, LOW);
}

//
// Receive up to the specified number of bytes from the master.
// Returns when the buffer is full or when the master deasserts SS (PB2).
// The number of bytes received is returned. This function should be called
// in a loop as long as the return value is nonzero.
//
size_t spi_slave_receive (uint8_t *buf, size_t count)
{
    if (count == 0) return 0;
    uint8_t *p = buf;
    do {
        SPDR = 0;
        // wait for byte to be available
        while (!(SPSR & _BV(SPIF))) {
            // if CS deasserted, bail out now
            if (PINB & _BV(2)) {
                return p - buf;
            }
        }
        // read byte into buffer
        *p++ = SPDR;
    } while (--count > 0);
    return p - buf;
}

//
// Send up to the specified number of bytes to the master. Returns when
// all bytes have been sent or when master deasserts SS (PB2). The number of
// bytes sent is returned.
//
size_t spi_slave_send (const uint8_t *buf, size_t count)
{
    if (count == 0) return 0;
    const uint8_t *p = buf;
    do {
        SPDR = *p++;
        // wait for frame to clear
        while (!(SPSR & _BV(SPIF))) {
            // if CS deasserted, bail out now
            if (PINB & _BV(2)) {
                return p - buf;
            }
        }
    } while (--count > 0);
    return p - buf;
}

static const uint8_t SPI_SLAVE_SENTINEL = 0x85;

//
// Send the sentinel character until the master deasserts SS (PB2).
//
void spi_slave_send_sentinel ()
{
    SPDR = SPI_SLAVE_SENTINEL;
    while (!(PINB & _BV(2))) {
        if (SPSR & _BV(SPIF)) {
            SPDR = SPI_SLAVE_SENTINEL;
        }
    }
}

//
// Initialize SPI in slave mode.
//
void spi_slave_init (uint8_t dataMode)
{
    // Make MISO an output, initially high
    digitalWrite(MISO, HIGH);
    pinMode(MISO, OUTPUT);
    
    // Enable pull-ups on SPI input lines
    digitalWrite(SS, HIGH);
    digitalWrite(MOSI, HIGH);
    digitalWrite(SCK, HIGH);
    
    // Enable SPI, Disable SPI Interrupts, MSB first, Enable Slave mode
    SPCR = _BV(SPE) | (dataMode & SPI_MODE_MASK);
}

enum SpiEepromCommand {
    SpiEepromCommandWrite = 0x2,
    SpiEepromCommandRead = 0x3 
};

//
// This function implements an SPI EEPROM using the same protocol as
// the Atmel AT25010B.
//
// Write to Eeprom:
//    MOSI: {0x2 address data0 data1 ... dataN}
//    MISO: {XX  XX ...                  XX   }
//
// Read from Eeprom:
//    MOSI: {0x3 address XX    XX    ... XX   }
//    MISO: {XX  XX      data0 data1 ... dataN}
//
//   where XX = don't care
//
// This has been tested and works reliably with an SPI clock of 100kHz.
// Attempting to use a higher clock may yield unpredicable results.
// At 500kHz, incorrect data is returned. The implementation was initially
// attempted using interrupts but it was too slow. Instead, this function 
// will poll synchronously for the duration of the frame. A frame is as
// long as SS (aka PB2, aka IO10) is held LOW.
//
static void handleSpiFrame ()
{
    // read command and address bytes
    union {
        uint8_t buf[2];
        struct {
            uint8_t command;
            uint8_t addr;
        } u;
    } header;
    
    if (spi_slave_receive(header.buf, sizeof(header.buf)) != 2) {
        return;
    }
    
    if (header.u.addr < sizeof(storage)) {
        // Preload SPDR with data at address. This optimization is necessary
        // to prevent us from missing the frame in the read case.
        SPDR = storage[header.u.addr];
        
        switch(header.u.command) {
        case SpiEepromCommandRead:
            // Master is reading (data from slave to master)
            spi_slave_send(&storage[header.u.addr], sizeof(storage) - header.u.addr);
            break;
        case SpiEepromCommandWrite:
            // Master is writing (data from master to slave)
            spi_slave_receive(&storage[header.u.addr], sizeof(storage) - header.u.addr);
            break;
        default:
            // unrecognized command
            Serial.println("Unrecognized command");
        }
    } else {
        Serial.println("Invalid address");
    }
        
    // continue to send sentinel until transfer is complete
    spi_slave_send_sentinel();
}

void setup()
{
    // set LED pin as output
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

    for (unsigned int i = 0; i < sizeof(storage); ++i) {
        storage[i] = i;
    }

    // join bus as slave
    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    
    // Initialize SPI in slave mode
    spi_slave_init(SPI_MODE0);
}

unsigned long nextPrintMillis;

void loop()
{
    // print out the first 16 bytes of the eeprom every 3 seconds
    if (millis() > nextPrintMillis) {
        nextPrintMillis = millis() + 3000;
        Serial.print("EEprom content: ");
        for (int i = 0; i < 16; ++i) {
            Serial.print(storage[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // Look for beginning of SPI frame (PB2 going LOW)
    if (!(PINB & _BV(2))) {
        handleSpiFrame();
    }
}
