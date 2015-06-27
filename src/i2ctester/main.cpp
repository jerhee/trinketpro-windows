#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#define SLAVE_ADDR 0x55

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

void fatalError ( )
{
    for (;;) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
}

void pulseLed ( )
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}

void blinkDelay (uint8_t seconds)
{
    while (seconds--) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
}

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define TWI_FREQ 100000L

static volatile uint8_t twi_error;

static void twi_receive_started ( );
static void twi_byte_received ( uint8_t data );
static void twi_byte_requested ( );
static void twi_stop_received ( );

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
  TWCR = _BV(TWEN) /*| _BV(TWIE)*/ | _BV(TWEA);
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
// void twi_reply(uint8_t ack)
// {
  // // transmit master read ready signal, with or without ack
  // if(ack){
    // TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  // }else{
	  // TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  // }
// }

static inline void twi_ack ( )
{
    TWCR = _BV(TWEN) /*| _BV(TWIE)*/ | _BV(TWINT) | _BV(TWEA);
}

static inline void twi_nack ( )
{
    TWCR = _BV(TWEN) /*| _BV(TWIE)*/ | _BV(TWINT);
}

/* 
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop ( )
{
  // send stop condition
  TWCR = _BV(TWEN) /*| _BV(TWIE)*/ | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

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
  TWCR = _BV(TWEN) /*| _BV(TWIE)*/ | _BV(TWEA) | _BV(TWINT);

  // update twi state
  // twi_state = TWI_READY;
}

static void twi_state_machine ( )
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
        twi_receive_started();
        break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
        // if there is still room in the rx buffer
        twi_byte_received(TWDR);
        break;
    case TW_SR_STOP: // stop or repeated start condition received
        twi_stop_received();
        twi_releaseBus();
        break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
        // nack back at master
        twi_nack();
        // ack future responses and leave slave receiver state
        twi_releaseBus();
        break;
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
    // {
        // TWDR = 0x55;
        // twi_ack();
        // uint8_t status;
        // uint8_t i = 0;
        // do {
            // status = TW_STATUS;
            // if (status == TW_ST_DATA_ACK) {
                // TWDR = i;
                // twi_ack();
                // ++i;
            // }
        // } while (status != TW_ST_DATA_NACK);
        // break;
    // }
    case TW_ST_DATA_ACK: // byte sent, ack returned        
        twi_byte_requested();
        break;
    case TW_ST_DATA_NACK: // received nack, we are done 
        twi_ack();
        break;
    case TW_ST_LAST_DATA: // received ack, but we are done already!
        // ack future responses
        twi_ack();
        break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
        twi_error = TW_BUS_ERROR;
        twi_stop();
        break;
    
    // All
    case TW_NO_INFO:   // no state information
    default:
        break;
  }
}

ISR(TWI_vect)
{
    for (;;) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
}

enum REGION : uint8_t {
    REGION_WRAP,
    REGION_STRETCH_250MS,
    REGION_STRETCH_LONG,
    REGION_NACK,
    REGION_CHECKSUM,
};

enum : uint8_t {
    REGION_WRAP_START = 0,
    REGION_WRAP_END = 0x80,
    REGION_STRETCH_250MS_START = REGION_WRAP_END,
    REGION_STRETCH_250MS_END = 0xA0,
    REGION_STRETCH_LONG_START = REGION_STRETCH_250MS_END,
    REGION_STRETCH_LONG_END = 0xC0,
    REGION_NACK_START = REGION_STRETCH_LONG_END,
    REGION_NACK_END = 0xF0,
    REGION_CHECKSUM_START = REGION_NACK_END,
    REGION_CHECKSUM_END = 0xFF,
    CHECKSUM_ADDRESS = 0xFF,
};

enum : uint8_t {
    LONG_STRETCH_DURATION_SECONDS = 15,
};

// provide 256 bytes of storage in our virtual eeprom
uint8_t storage[256];

// current EEPROM address
static volatile uint8_t address = 0;

static volatile REGION region = REGION_WRAP;

static uint32_t crc;
void update_checksum ( uint8_t data )
{
    uint32_t tempCrc = crc ^ (data << 8);
    for (uint8_t i = 8; i; i--) {
        if (tempCrc & 0x8000)
            tempCrc ^= (0x1070 << 3);
        tempCrc <<= 1;
    }
    crc = tempCrc;
}

REGION region_from_address ( uint8_t data )
{
    // determine region based on first byte of transfer (the address byte)
    if (data < REGION_WRAP_END) {
        return REGION_WRAP;
    } else if (data < REGION_STRETCH_250MS_END) {
        return REGION_STRETCH_250MS;
    } else if (data < REGION_STRETCH_LONG_END) {
        return REGION_STRETCH_LONG;
    } else if (data < REGION_NACK_END) {
        return REGION_NACK;
    }
    
    return REGION_CHECKSUM;
}

static volatile bool isFirstByte;

void twi_receive_started ( )
{
    isFirstByte = true;
    
    // if the current address is 0xEF, NACK the first byte of this transfer,
    // and then reset address to the first byte of the NOWRAP region.
    if (address == 0xBF) {
        blinkDelay(LONG_STRETCH_DURATION_SECONDS);
        address = REGION_STRETCH_LONG_START;
        twi_ack();
    } if (address == 0xEF) {
        twi_nack();
        address = REGION_NACK_START;
    } else {
        twi_ack();
    }
}

void twi_byte_received ( uint8_t data )
{
    if (isFirstByte) {
        if (data == 0xEE) {
            twi_nack();
            region = REGION_NACK;
            address = data;
            isFirstByte = false;
        } else {
            if (data == 0xBE) {
                blinkDelay(LONG_STRETCH_DURATION_SECONDS);
            }
            twi_ack();
            // determine region based on first byte of transfer (the address byte)
            region = region_from_address(data);
            address = data;
            isFirstByte = false;
        }
        
        return;
    }
    
    switch (region) {
    case REGION_WRAP:
        twi_ack();
        storage[address] = data;
        ++address;
        if (address >= REGION_WRAP_END) {
            address = REGION_WRAP_START;
        }
        break;
    case REGION_STRETCH_250MS:
        storage[address] = data;
        ++address;
        if (address >= REGION_STRETCH_250MS_END) {
            address = REGION_STRETCH_250MS_START;
        }
        // XXX implement me
        twi_ack();
        break;
    case REGION_STRETCH_LONG:
        ++address;
        if (address >= REGION_STRETCH_LONG_END) {
            address = REGION_STRETCH_LONG_START;
            blinkDelay(LONG_STRETCH_DURATION_SECONDS);
        }
        twi_ack();
        break;
    case REGION_NACK:
        if (address >= 0xed) {
            twi_nack();
        } else {
            twi_ack();
        }
        storage[address] = data;
        ++address;
        break;
    case REGION_CHECKSUM:
        twi_ack();
        if (address == 0xff) {
            // reset the checksum to 0
            crc = 0;
        } else {
            // add data to checksum
            update_checksum(data);
        }
        break;
    default:
        twi_nack();
    }
}

void twi_byte_requested ( )
{
    uint8_t addr = address;

    // region was determined by previous write
    switch (region) {
    case REGION_WRAP:
    {
        TWDR = storage[addr];
        twi_ack();
        addr = (addr + 1) % REGION_WRAP_END;
        
        uint8_t status;
        uint8_t preload = storage[addr];
        do {
            status = TW_STATUS;
            if (status == TW_ST_DATA_ACK) {
                TWDR = preload;
                twi_ack();
                
                addr = (addr + 1) % REGION_WRAP_END;
                preload = storage[addr];
            }
        } while (status != TW_ST_DATA_NACK);
        break;
    }
    case REGION_STRETCH_250MS:
        TWDR = storage[address];
        ++address;
        if (address >= REGION_STRETCH_250MS_END) {
            address = REGION_STRETCH_250MS_START;
        }
        delayMicroseconds(100);
        twi_ack();
        break;
    case REGION_STRETCH_LONG:
        TWDR = addr;
        ++addr;
        if (addr >= (REGION_STRETCH_LONG_END - 2)) {
            addr = REGION_STRETCH_LONG_START;
            blinkDelay(LONG_STRETCH_DURATION_SECONDS);
        }
        twi_ack();
        break;
    case REGION_NACK:
        // reads in the NACK region return current address
        TWDR = addr;
        twi_ack();
        break;
    case REGION_CHECKSUM:
        TWDR = static_cast<uint8_t>(crc >> 8);
        twi_ack();
        break;
    default:
        TWDR = 0x55;
        twi_ack();
    }
    
    address = addr;
}

void twi_stop_received ( )
{
    // do nothing
}

void setup()
{
    // set LED pin as output
    pinMode(LED_BUILTIN, OUTPUT);
    // Serial.begin(115200);
    
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
    twi_state_machine();
}

