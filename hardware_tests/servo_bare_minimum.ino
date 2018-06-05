/**
 * @file: servo_esc_test.ino
 *
 * @brief: control ESC's with Arduino Servo library.
 * TODO: Compare signals on o-scope.
 */

/* Libraries */
#include <Servo.h>  // built-in Servo library will create signals needed for esc

/* Pin Definitions */
#define ESC_PIN 6

/* ESC signal constants */
#define ESC_LOW_LIMIT 1000   // [microseconds] corresponds to zero throttle
#define ESC_UPP_LIMIT 2000   // [microseconds] corresponds to full throttle

/* Servo Object */
Servo esc;

/* Setup */
void setup() {
  esc.attach(ESC_PIN);  // "initializes" ESC at ESC_PIN
  esc.writeMicroseconds(ESC_LOW_LIMIT);
  delay(500); // give the ESC some time to arm
}

/* Main loop */
void loop() {
  esc.writeMicroseconds(1500);
}
