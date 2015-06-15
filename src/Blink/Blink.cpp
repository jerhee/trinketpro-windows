/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
#include <Arduino.h>

// use PB0 (Digital 8)

void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  //pinMode(13, OUTPUT);     
  // DDRx controls direction (1 = output, 0 = input)
  // PORTx must be 0 to switch off pull up resistor
  // swithc DDR between 0 and 1
  PORTB &= ~0x1;
  DDRB &= ~0x1;
}

void loop() {
  DDRB |= 0x1;
  delay(100);
  DDRB &= ~0x1;
  delay(1000);
}
