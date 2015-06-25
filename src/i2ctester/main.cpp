#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

class AutoLed
{
public:
    AutoLed ()
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }

    ~AutoLed ()
    {
        digitalWrite(LED_BUILTIN, LOW);
    }
};

class NoInterrupts
{
public:
    NoInterrupts ()
    {
        noInterrupts();
    }

    ~NoInterrupts ()
    {
        interrupts();
    }
};

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define SLAVE_ADDR 0x55
#define TWI_FREQ 100000L

bool twi_byte_received (uint8_t data, bool isStart);
bool twi_byte_requested (bool isStart, uint8_t* data);
void twi_stop_received ( );

static volatile uint8_t twi_error;

/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void)
{
  // initialize state
  // twi_state = TWI_READY;
  
  // activate internal pullups for twi.
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // enable twi module, acks, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

/* 
 * Function twi_slaveInit
 * Desc     sets slave address and enables interrupt
 * Input    none
 * Output   none
 */
void twi_setAddress(uint8_t address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}

/* 
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
void twi_reply(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop(void)
{
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }

  // update twi state
  // twi_state = TWI_READY;
}

/* 
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twi_releaseBus(void)
{
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

  // update twi state
  // twi_state = TWI_READY;
}

volatile bool twi_start = false;

ISR(TWI_vect)
{
  switch(TW_STATUS){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
    case TW_MT_SLA_NACK:  // address sent, nack received
    case TW_MT_DATA_NACK: // data sent, nack received
    case TW_MT_ARB_LOST: // lost bus arbitration
    case TW_MR_DATA_ACK: // data received, ack sent
    case TW_MR_SLA_ACK:  // address sent, ack received
    case TW_MR_DATA_NACK: // data received, nack sent
    case TW_MR_SLA_NACK: // address sent, nack received
      for (;;) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
      }
      break;

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      // enter slave receiver mode
      // twi_state = TWI_SRX;
      // save the flag indicating that we were just addressed
      twi_reply(1);
      twi_start = true;
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
      // if there is still room in the rx buffer
      twi_reply(twi_byte_received(TWDR, twi_start));
      twi_start = false;
      // if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        // // put byte in buffer and ack
        // twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        // twi_reply(1);
      // }else{
        // // otherwise nack
        // twi_reply(0);
      // }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      // put a null char after data if there's room
      // if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        // twi_rxBuffer[twi_rxBufferIndex] = '\0';
      // }
      // sends ack and stops interface for clock stretching
      //twi_stop();  <-- THIS LINE IS COMMENTED OUT TO FIX REPEATED STARTS
      // callback to user defined callback
      twi_stop_received();
      // since we submit rx buffer to "wire" library, we can reset it
      // twi_rxBufferIndex = 0;
      // ack future responses and leave slave receiver state
      twi_releaseBus();
      twi_start = false;
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      // nack back at master
      twi_reply(0);
      // ack future responses and leave slave receiver state
      twi_releaseBus();
      twi_start = false;
      break;
    
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      twi_start = true;
      // enter slave transmitter mode
      // twi_state = TWI_STX;
      // ready the tx buffer index for iteration
      // twi_txBufferIndex = 0;
      // // set tx buffer length to be zero, to verify if user changes it
      // twi_txBufferLength = 0;
      // // request for txBuffer to be filled and length to be set
      // // note: user must call twi_transmit(bytes, length) to do this
      // twi_onSlaveTransmit();
      // // if they didn't change buffer & length, initialize it
      // if(0 == twi_txBufferLength){
        // twi_txBufferLength = 1;
        // twi_txBuffer[0] = 0x00;
      // }
      // transmit first byte from buffer, fall
    case TW_ST_DATA_ACK: // byte sent, ack returned
      // if there is more to send, ack, otherwise nack
      uint8_t data;
      if (twi_byte_requested(twi_start, &data)) {
        // copy data to output register
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        TWDR = data;
#pragma GCC diagnostic pop        
        twi_reply(1);
      } else {
        twi_reply(0);
      }
      twi_start = false;
      break;
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      twi_reply(1);
      // leave slave receiver state
      // twi_state = TWI_READY;
      twi_start = false;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      twi_start = false;
      break;
  }
}

enum REGION : uint8_t {
    REGION_WRAP,
    REGION_STRETCH_100US,
    REGION_STRETCH_100MS,
    REGION_STRETCH_1S,
    REGION_NOWRAP,
    REGION_NULL,
    REGION_CHECKSUM,
};

enum : uint8_t {
    REGION_WRAP_START = 0,
    REGION_WRAP_END = 0x80,
    REGION_STRETCH_100US_START = REGION_WRAP_END,
    REGION_STRETCH_100US_END = 0xA0,
    REGION_STRETCH_100MS_START = REGION_STRETCH_100US_END,
    REGION_STRETCH_100MS_END = 0xC0,
    REGION_STRETCH_1S_START = REGION_STRETCH_100MS_END,
    REGION_STRETCH_1S_END = 0xE0,
    REGION_NOWRAP_START = REGION_STRETCH_1S_END,
    REGION_NOWRAP_END = 0xF0,
    REGION_NULL_START = REGION_NOWRAP_END,
    REGION_NULL_END = 0xFF,
    CHECKSUM_ADDRESS = 0xFF,
};

// provide 256 bytes of storage in our virtual eeprom
uint8_t storage[256];

// current EEPROM address
uint8_t address = 0;

REGION region = REGION_WRAP;

static uint32_t crc;
void update_checksum ( uint8_t data )
{
    crc ^= (data << 8);
    for (uint8_t i = 8; i; i--) {
        if (crc & 0x8000)
            crc ^= (0x1070 << 3);
        crc <<= 1;
    }
}

REGION region_from_address ( uint8_t address )
{
    // determine region based on first byte of transfer (the address byte)
    if (address < REGION_WRAP_END) {
        return REGION_WRAP;
    } else if (address < REGION_STRETCH_100US_END) {
        return REGION_STRETCH_100US;
    } else if (address < REGION_STRETCH_100MS_END) {
        return REGION_STRETCH_100MS;
    } else if (address < REGION_STRETCH_1S_END) {
        return REGION_STRETCH_1S;
    } else if (address < REGION_NOWRAP_END) {
        return REGION_NOWRAP;
    } else if (address == CHECKSUM_ADDRESS) {
        return REGION_CHECKSUM;
    }
    
    return REGION_NULL;
}

bool twi_byte_received (uint8_t data, bool isStart)
{
    if (isStart) {
        // determine region based on first byte of transfer (the address byte)
        region = region_from_address(data);
        address = data;
        return true;
    }
    
    if (region < REGION_NULL) {
        update_checksum(data);
    }
    
    switch (region) {
    case REGION_WRAP:
        storage[address] = data;
        ++address;
        if (address >= REGION_WRAP_END) {
            address = REGION_WRAP_START;
        }
        return true;
    case REGION_STRETCH_100US:
        storage[address] = data;
        ++address;
        if (address >= REGION_STRETCH_100US_END) {
            address = REGION_STRETCH_100US_START;
        }
        delayMicroseconds(100);
        return true;
    case REGION_STRETCH_100MS:
        storage[address] = data;
        ++address;
        if (address >= REGION_STRETCH_100MS_END) {
            address = REGION_STRETCH_100MS_START;
        }
        delay(100);
        return true;
    case REGION_STRETCH_1S:
        storage[address] = data;
        ++address;
        if (address >= REGION_STRETCH_1S_END) {
            address = REGION_STRETCH_1S_START;
        }
        delay(1000);
        return true;
    case REGION_NOWRAP:
        if (address >= REGION_NOWRAP_END) {
            return false;
        }
        storage[address] = data;
        ++address;
        return true;
    case REGION_CHECKSUM:
        crc = data << 8;
        return true;
    case REGION_NULL:
    default:
        return true;
    }
}

bool twi_byte_requested (bool isStart, uint8_t* data)
{
    // region was determined by previous write
    switch (region) {
    case REGION_WRAP:
        *data = storage[address];
        ++address;
        if (address >= REGION_WRAP_END) {
            address = REGION_WRAP_START;
        }
        return true;
    case REGION_STRETCH_100US:
        *data = storage[address];
        ++address;
        if (address >= REGION_STRETCH_100US_END) {
            address = REGION_STRETCH_100US_START;
        }
        delayMicroseconds(100);
        return true;
    case REGION_STRETCH_100MS:
        *data = storage[address];
        ++address;
        if (address >= REGION_STRETCH_100MS_END) {
            address = REGION_STRETCH_100MS_START;
        }
        delay(100);
        return true;
    case REGION_STRETCH_1S:
        *data = storage[address];
        ++address;
        if (address >= REGION_STRETCH_1S_END) {
            address = REGION_STRETCH_1S_START;
        }
        delay(1000);
        return true;
    case REGION_NOWRAP:
        // a repeated start within the NOWRAP region resets 
        if (address >= REGION_NOWRAP_END) {
            return false;
        }
        *data = storage[address];
        ++address;
        return true;
    case REGION_CHECKSUM:
        *data = static_cast<uint8_t>(crc >> 8);
        return true;
    case REGION_NULL:
    default:
        *data = 0x55;
        return true;
    }
}

void twi_stop_received ( )
{
    // do nothing
}

void setup()
{
    // set LED pin as output
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    
    twi_init();
    twi_setAddress(SLAVE_ADDR);
}

unsigned long nextPrintMillis;

void loop()
{
    // print out the first 16 bytes of the eeprom every 3 seconds
    //if (millis() > nextPrintMillis) {
    //    nextPrintMillis = millis() + 3000;
    //    Serial.print("EEprom content: ");
    //    for (int i = 0; i < 16; ++i) {
    //        Serial.print(storage[i]);
    //        Serial.print(" ");
    //    }
    //    Serial.println();
    //}
    
    // Look for beginning of SPI frame (PB2 going LOW)
    // if (!(PINB & _BV(2))) {
        // handleSpiFrame();
    // }
}

