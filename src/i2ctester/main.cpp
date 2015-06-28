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

//
// Blink the LED on a period that's independent of the
// delay duration.
//
void blinkDelay ( unsigned long timeInMs )
{
    interrupts();
    unsigned long end = millis() + timeInMs;
    unsigned long nextToggle = 0;
    uint8_t i = 0;
    
    unsigned long now;
    while ((now = millis()) < end) {
        if (now >= nextToggle) {
            digitalWrite(LED_BUILTIN, ++i & 1);
            nextToggle += 500;
        }
    }
    
    digitalWrite(LED_BUILTIN, LOW);
}

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define TWI_FREQ 100000L

static volatile uint8_t twi_error;

static void twi_addressed_for_write ( );
static void twi_byte_received ( uint8_t data );
static void twi_byte_requested ( bool isStart );
static void twi_stop_received ( );

/* 
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init (void)
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
void twi_setAddress (uint8_t address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}

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
void twi_releaseBus (void)
{
  // release bus
  TWCR = _BV(TWEN) /*| _BV(TWIE)*/ | _BV(TWEA) | _BV(TWINT);

  // update twi state
  // twi_state = TWI_READY;
}

static uint8_t portBLEDOn;
static uint8_t portBLEDOff;

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
      fatalError();
      break;

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
        PORTB = portBLEDOn;
        twi_addressed_for_write();
        break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
        PORTB = portBLEDOn;
        twi_byte_received(TWDR);
        break;
    case TW_SR_STOP: // stop or repeated start condition received
        PORTB = portBLEDOn;
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
        twi_byte_requested(true);
        PORTB = portBLEDOn;
        break;
    case TW_ST_DATA_ACK: // byte sent, ack returned        
        twi_byte_requested(false);
        PORTB = portBLEDOn;
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
  PORTB = portBLEDOff;
}

ISR(TWI_vect)
{
    fatalError();
}


/* 8-bit CRC with polynomial x^8+x^6+x^3+x^2+1, 0x14D.
   Chosen based on Koopman, et al. (0xA6 in his notation = 0x14D >> 1):
   http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 */

static uint8_t crc8_table[] = {
    0x00, 0x3e, 0x7c, 0x42, 0xf8, 0xc6, 0x84, 0xba, 0x95, 0xab, 0xe9, 0xd7,
    0x6d, 0x53, 0x11, 0x2f, 0x4f, 0x71, 0x33, 0x0d, 0xb7, 0x89, 0xcb, 0xf5,
    0xda, 0xe4, 0xa6, 0x98, 0x22, 0x1c, 0x5e, 0x60, 0x9e, 0xa0, 0xe2, 0xdc,
    0x66, 0x58, 0x1a, 0x24, 0x0b, 0x35, 0x77, 0x49, 0xf3, 0xcd, 0x8f, 0xb1,
    0xd1, 0xef, 0xad, 0x93, 0x29, 0x17, 0x55, 0x6b, 0x44, 0x7a, 0x38, 0x06,
    0xbc, 0x82, 0xc0, 0xfe, 0x59, 0x67, 0x25, 0x1b, 0xa1, 0x9f, 0xdd, 0xe3,
    0xcc, 0xf2, 0xb0, 0x8e, 0x34, 0x0a, 0x48, 0x76, 0x16, 0x28, 0x6a, 0x54,
    0xee, 0xd0, 0x92, 0xac, 0x83, 0xbd, 0xff, 0xc1, 0x7b, 0x45, 0x07, 0x39,
    0xc7, 0xf9, 0xbb, 0x85, 0x3f, 0x01, 0x43, 0x7d, 0x52, 0x6c, 0x2e, 0x10,
    0xaa, 0x94, 0xd6, 0xe8, 0x88, 0xb6, 0xf4, 0xca, 0x70, 0x4e, 0x0c, 0x32,
    0x1d, 0x23, 0x61, 0x5f, 0xe5, 0xdb, 0x99, 0xa7, 0xb2, 0x8c, 0xce, 0xf0,
    0x4a, 0x74, 0x36, 0x08, 0x27, 0x19, 0x5b, 0x65, 0xdf, 0xe1, 0xa3, 0x9d,
    0xfd, 0xc3, 0x81, 0xbf, 0x05, 0x3b, 0x79, 0x47, 0x68, 0x56, 0x14, 0x2a,
    0x90, 0xae, 0xec, 0xd2, 0x2c, 0x12, 0x50, 0x6e, 0xd4, 0xea, 0xa8, 0x96,
    0xb9, 0x87, 0xc5, 0xfb, 0x41, 0x7f, 0x3d, 0x03, 0x63, 0x5d, 0x1f, 0x21,
    0x9b, 0xa5, 0xe7, 0xd9, 0xf6, 0xc8, 0x8a, 0xb4, 0x0e, 0x30, 0x72, 0x4c,
    0xeb, 0xd5, 0x97, 0xa9, 0x13, 0x2d, 0x6f, 0x51, 0x7e, 0x40, 0x02, 0x3c,
    0x86, 0xb8, 0xfa, 0xc4, 0xa4, 0x9a, 0xd8, 0xe6, 0x5c, 0x62, 0x20, 0x1e,
    0x31, 0x0f, 0x4d, 0x73, 0xc9, 0xf7, 0xb5, 0x8b, 0x75, 0x4b, 0x09, 0x37,
    0x8d, 0xb3, 0xf1, 0xcf, 0xe0, 0xde, 0x9c, 0xa2, 0x18, 0x26, 0x64, 0x5a,
    0x3a, 0x04, 0x46, 0x78, 0xc2, 0xfc, 0xbe, 0x80, 0xaf, 0x91, 0xd3, 0xed,
    0x57, 0x69, 0x2b, 0x15};



enum REGISTERS : uint8_t {
    EEPROM_ADDRESS_MAX = 0x7F,
    REG_SCL_HOLD_MILLIS_HI = 0xF9,
    REG_SCL_HOLD_MILLIS_LO = 0xFA,
    REG_HOLD_READ_CONTROL = 0xFB,
    REG_HOLD_WRITE_CONTROL = 0xFC,
    REG_NAK_CONTROL = 0xFD,
    REG_CHECKSUM_UPDATE = 0xFE,
    REG_CHECKSUM_RESET = 0xFF,
};

enum : uint16_t {
    SCL_HOLD_DEFAULT = 15000
};

enum STATE : uint8_t {
    STATE_NORMAL = 0,
    STATE_NAK_WRITE = 0x1,    // NAK_WRITE and HOLD_WRITE are mutually exclusive
    STATE_HOLD_WRITE = 0x2,
    STATE_HOLD_READ = 0x4,      // HOLD_READ can be superimposed with NAK_WRITE or HOLD_WRITE
};

static uint8_t state = STATE_NORMAL;

// provide 256 bytes of storage in our virtual eeprom
static uint8_t storage[256];

static uint32_t crc;    
void update_checksum ( uint8_t data )
{
    crc = crc8_table[uint8_t((crc ^ 0xff) ^ data)] ^ 0xff;
    
    // update current register values
    uint8_t crc8 = uint8_t(crc & 0xff);
    storage[REG_CHECKSUM_UPDATE] = crc8;
    storage[REG_CHECKSUM_RESET] = crc8;
}

void delay_current_hold_millis ( )
{
    unsigned long timeInMillis = 
        (storage[REG_SCL_HOLD_MILLIS_HI] << 8) |
        storage[REG_SCL_HOLD_MILLIS_LO];
    blinkDelay(timeInMillis);
}

// current EEPROM address
static uint8_t address = 0;

static uint8_t countdown;

static bool isFirstByte;
void twi_addressed_for_write ( )
{
    isFirstByte = true;
    
    // if NAK is armed, check if NAK length is 0, or set up
    // NAK state
    if (storage[REG_NAK_CONTROL] != 0xff) {
        if (storage[REG_NAK_CONTROL] == 0) {
            twi_nack();
        } else {
            twi_ack();
            countdown = storage[REG_NAK_CONTROL];
            state |= STATE_NAK_WRITE;
        }
        // NAK is always one-shot. This resets NAK control.
        storage[REG_NAK_CONTROL] = 0xff;
    } else if (storage[REG_HOLD_WRITE_CONTROL] != 0xff) {
        if (storage[REG_HOLD_WRITE_CONTROL] == 0) {
            delay_current_hold_millis();
        } else {
            // enter hold write state
            countdown = storage[REG_HOLD_WRITE_CONTROL];
            state |= STATE_HOLD_WRITE;
        }
        
        twi_ack();
        
        // Hold write is always one-shot. This resets hold write.
        storage[REG_HOLD_WRITE_CONTROL] = 0xff;
    } else {
        twi_ack();
        
        // if write NAK or HOLD_WRITE are not armed, ensure we
        // are not in those states
        state &= ~(STATE_HOLD_WRITE | STATE_NAK_WRITE);
    }
    
    // turn off interrupts while receive is in progress
    noInterrupts();
}

void twi_byte_received ( uint8_t data )
{
    if (state & STATE_NAK_WRITE) {
        // count down until it's time to NAK.
        // zero-length NAK would have been handled in twi_addressed_for_write
        if (--countdown == 0) {
            // NAKing here means the next byte that the master sends will
            // be NAKed.
            twi_nack();
            state &= ~STATE_NAK_WRITE;
        } else {
            twi_ack();
        }
    } else if (state & STATE_HOLD_WRITE) {
        if (--countdown == 0) {
            delay_current_hold_millis();
            state &= ~STATE_HOLD_WRITE;
        }
        
        // data received in HOLD_WRITE mode is ignored
        
        twi_ack();
    } else {
        twi_ack();
        
        if (isFirstByte) {
            address = data;
            isFirstByte = false;
            return;
        }
        
        // if we're in eeprom range, increment and wrap address
        if (address <= EEPROM_ADDRESS_MAX) {
            storage[address] = data;
            address = (address + 1) % (EEPROM_ADDRESS_MAX + 1);
            return;
        }
        
        // update register with data. address does not auto increment
        // for register writes
        switch (address) {
        case REG_SCL_HOLD_MILLIS_HI:
            // increment address so that both HI and LO bytes can
            // be written in a single operation
            storage[address] = data;
            ++address;
            break;
        case REG_SCL_HOLD_MILLIS_LO:
        case REG_HOLD_READ_CONTROL:
        case REG_HOLD_WRITE_CONTROL:
        case REG_NAK_CONTROL:
            storage[address] = data;
            break;
        case REG_CHECKSUM_UPDATE:
            update_checksum(data);
            break;
        case REG_CHECKSUM_RESET:
            crc = 0;
            break;
        default:
            // writes to reserved registers are ignored
            break;
        }
    }
}

void twi_byte_requested ( bool isStart )
{
    if (isStart) {
        // check if hold read is armed
        if (storage[REG_HOLD_READ_CONTROL] != 0xff) {
            if (storage[REG_HOLD_READ_CONTROL] == 0) {
                countdown = 0;
                delay_current_hold_millis();
            } else {
                // enter the hold read state
                countdown = storage[REG_HOLD_READ_CONTROL];
                state |= STATE_HOLD_READ;
            }
            
            TWDR = countdown;
            twi_ack();
            
            // ensure that hold read is a one-shot operation
            storage[REG_HOLD_READ_CONTROL] = 0xff;
        } else {
            // all other reads give back current register contents
            TWDR = storage[address];
            twi_ack();
            
            // addresses within the eeprom increment and wrap
            if (address <= EEPROM_ADDRESS_MAX) {
                address = (address + 1) % (EEPROM_ADDRESS_MAX + 1);
            }
        }
    } else if (state & STATE_HOLD_READ) {
        // hold read state counts down until it's time to hold
        if (--countdown == 0) {
            delay_current_hold_millis();
        }
        
        TWDR = countdown;
        twi_ack();
    } else {
        // all other states return current register value
        TWDR = storage[address];
        twi_ack();
        
        if (address <= EEPROM_ADDRESS_MAX) {
            // addresses within the eeprom increment and wrap
            address = (address + 1) % (EEPROM_ADDRESS_MAX + 1);
        }
    }
}

void twi_stop_received ( )
{
    // reenable interrupts
    interrupts();
}

void setup()
{
    // set LED pin as output
    pinMode(LED_BUILTIN, OUTPUT);
    // Serial.begin(115200);
    
    // fill storage
    for (unsigned i = 0; i < sizeof(storage); ++i) {
        storage[i] = 0x55;
    }
    
    // load default register values
    storage[REG_SCL_HOLD_MILLIS_HI] = 0x3A,
    storage[REG_SCL_HOLD_MILLIS_LO] = 0x98,
    storage[REG_HOLD_READ_CONTROL] = 0xFF,
    storage[REG_HOLD_WRITE_CONTROL] = 0xFF,
    storage[REG_NAK_CONTROL] = 0xFF,
    storage[REG_CHECKSUM_UPDATE] = 0,
    storage[REG_CHECKSUM_RESET] = 0,
    
    twi_init();
    twi_setAddress(SLAVE_ADDR);
    
    portBLEDOn = PORTB | _BV(5);
    portBLEDOff = PORTB;
}

void loop()
{
    twi_state_machine();
}

