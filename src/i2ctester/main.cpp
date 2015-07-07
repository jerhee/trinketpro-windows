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

// CRC16 table for CCITT polynomial (0x1021)
static const uint16_t crc16tab[] = {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};




enum REGISTERS : uint8_t {
    EEPROM_ADDRESS_MAX = 0x7F,
    REG_DISABLE_REPEATED_STARTS = 0xF8,
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
    crc = uint32_t(crc << 8) ^ uint32_t(crc16tab[uint8_t((crc >> 8) ^ data)]);
    
    // update current register values
    storage[REG_CHECKSUM_UPDATE] = uint8_t(crc >> 8);
    storage[REG_CHECKSUM_RESET] = uint8_t(crc);
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
            storage[REG_CHECKSUM_UPDATE] = 0;
            storage[REG_CHECKSUM_RESET] = 0;
            break;
        default:
            // writes to reserved registers are ignored
            break;
        }
    }
}

void twi_byte_requested ( bool isStart )
{
    if (isStart && (storage[REG_HOLD_READ_CONTROL] != 0xff)) {
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
        } else {
            // address increments and wraps
            ++address;
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
    storage[REG_DISABLE_REPEATED_STARTS] = 0;
    storage[REG_SCL_HOLD_MILLIS_HI] = 0x3A;
    storage[REG_SCL_HOLD_MILLIS_LO] = 0x98;
    storage[REG_HOLD_READ_CONTROL] = 0xFF;
    storage[REG_HOLD_WRITE_CONTROL] = 0xFF;
    storage[REG_NAK_CONTROL] = 0xFF;
    storage[REG_CHECKSUM_UPDATE] = 0;
    storage[REG_CHECKSUM_RESET] = 0;
    
    twi_init();
    twi_setAddress(SLAVE_ADDR);
    
    portBLEDOn = PORTB | _BV(5);
    portBLEDOff = PORTB;
}

void loop()
{
    twi_state_machine();
}

