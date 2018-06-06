#include <Arduino.h>

const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;
const int ESC_SIGNAL_PERIOD_S = 30; //20

const int CH1_PULSE = 1000;
const int CH2_PULSE = 1000;
const int CH3_PULSE = 1000;
const int CH4_PULSE = 1000;

int ESCsignals[4] = {1000, 1000, 1000, 1000};

void setup()
{
  DDRD |= B11110000; // set pins 4-7 as outputs
  armESCs();  // send a 1000 us pulse to arm the ESC's
}

void loop()
{
  writeToESCs(ESCsignals);
}

/**
 * @brief: send a ESC_MIN_US pulse to all ESC's to arm them.
 */
void armESCs()
{
  PORTD |= B11110000;
  delayMicroseconds(ESC_MIN_US);
  PORTD &= B00000000;
  delay(ESC_SIGNAL_PERIOD_S - (ESC_MIN_US/1000));
  delay(1000);
}

/**
 * @brief: write pulses to ESC's simultaneously
 * @param esc_signals[4] array of ESC signals
 */
void writeToESCs(int esc_signals[4])
{
  // timers
  static unsigned long timeStart;
  static unsigned long channelTimer;

  // get the start time stamp
  timeStart = micros();

  // pull all of the ESC pins high.
  PORTD |= B11110000;
  
  while (PORTD >= B00001111) {
    channelTimer = micros() - timeStart;
    if (channelTimer >= esc_signals[0]) PORTD &= B01111111;
    if (channelTimer >= esc_signals[1]) PORTD &= B10111111;
    if (channelTimer >= esc_signals[2]) PORTD &= B11011111;
    if (channelTimer >= esc_signals[3]) PORTD &= B11101111;
  }
  delay(20);
}
