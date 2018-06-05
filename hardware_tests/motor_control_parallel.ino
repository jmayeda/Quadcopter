#include <Arduino.h>

const int ESC_MIN_US = 1000;        // minimum ESC signal pulse width [microseconds]
const int ESC_MAX_US = 2000;        // maximum ESC signal pulse width [microseconds]
const int ESC_SIGNAL_PERIOD_S = 20; // period of the Servo lib. signals [milliseconds]

/* for debugging */
const int CH1_PULSE = 1100;
const int CH2_PULSE = 1200;
const int CH3_PULSE = 1300;
const int CH4_PULSE = 1400;

/**
 * @brief: main Arduino function. Runs once on start-up.
 */
void setup()
{
  DDRD |= B11110000; // set pins 4-7 as outputs
  armESCs();  // send a 1000 us pulse to arm the ESC's
}

/**
 * @brief: main Arduino function. Runs continuously in loop.
 */
void loop()
{
  PORTD |= B11110000;
  while (PORTD > B00001111)
  {
    unsigned long channelTimer = micros();
    if (channelTimer >= CH1_PULSE) PORTD &= B01111111;
    if (channelTimer >= CH2_PULSE) PORTD &= B10111111;
    if (channelTimer >= CH3_PULSE) PORTD &= B11011111;
    if (channelTimer >= CH4_PULSE) PORTD &= B11101111;
  }
}

/**
 * @brief: write one pulse value to all channels (pins 4,5,6,7)
 * @param pulseWidth width of square wave pulse in microseconds
 */
void writeAllChannels(int pulseWidth)
{
  PORTD |= B11110000;
  delayMicroseconds(pulseWidth);
  PORTD &= B00000000;
  delay(ESC_SIGNAL_PERIOD_S - (pulseWidth/1000));
}

/**
 * @brief: main function to write to all motors simulatenously. Functions by
 *         pulling all ESC signal pins (4,5,6,7) HIGH then subsequently pulls
 *         the pins LOW after their respective pulse width has expired.
 * @param pulseWidth array of width of square wave pulse in microseconds
 *                   corresponding to channels 7,6,5,4 respectively.
 */
void writeToMotors(int[4] pulseWidth)
{
  PORTD |= B11110000;
  while (PORTD > B00001111)
  {
    unsigned long channelTimer = micros();
    if (channelTimer >= CH1_PULSE) PORTD &= B01111111;
    if (channelTimer >= CH2_PULSE) PORTD &= B10111111;
    if (channelTimer >= CH3_PULSE) PORTD &= B11011111;
    if (channelTimer >= CH4_PULSE) PORTD &= B11101111;
  }
}

/**
 * @brief: write 1000 microsecond pulse to ESC's in order to arm them.
 */
void armESCs()
{
  writeAllChannels(ESC_MIN_US);
  delay(1000);
}
