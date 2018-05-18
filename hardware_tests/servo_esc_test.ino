/**
 * @file: servo_esc_test.ino
 *
 * @brief: control ESC's with Arduino Servo library.
 * TODO: Compare signals on o-scope.
 */

/* Libraries */
#include <Servo.h>  // built-in Servo library will create signals needed for esc

/* Pin Definitions */
#define RED_LED_PIN 9
#define GREEN_LED_PIN 8
#define POT_PIN A0
#define ESC_PIN 6

/* ESC signal constants */
#define ESC_LOW_LIMIT 1000   // [microseconds] corresponds to zero throttle
#define ESC_UPP_LIMIT 2000   // [microseconds] corresponds to full throttle
#define ANALOG_READ_MAX 1023 // maximum value for the 10 bit ADC on arduino

/* Servo Object */
Servo esc;
int analogPotVal, escOutputSignal;

/* Setup */
void setup() {
  // set pin modes
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  // turn on Red LED to show setup
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);

  esc.attach(ESC_PIN);  // "initializes" ESC at ESC_PIN
  esc.writeMicroseconds(ESC_LOW_LIMIT);
  delay(500); // give the ESC some time to arm

  // turn off Red LED to show that setup is over
  digitalWrite(RED_LED_PIN, LOW);
}

/* Main loop */
void loop() {
  digitalWrite(GREEN_LED_PIN, HIGH);  // turn on green LED to indicate ESC is live

  analogPotVal = analogRead(POT_PIN);  // read the potentiometer for user input

  // map() takes five arguments and maps a variable from one range to another
  //   (1): variable to be mapped
  //   (2): lower limit of current range
  //   (3): upper limit of current range
  //   (4): lower limit of desired range
  //   (5): upper limit of desired range
  escOutputSignal = map(analogPotVal, 0, ANALOG_READ_MAX, ESC_LOW_LIMIT, ESC_UPP_LIMIT);

  // write a signal to the ESC corresponding to the potentiometer value
  esc.writeMicroseconds(escOutputSignal);
}
