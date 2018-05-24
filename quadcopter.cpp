/**
 * @file: quadcopter.cpp
 *
 * @brief: quadcopter flight controller for the ESW Quarterly Project 2018.
 * @date: 05/22/2018
 */

/**
 * TODO: Add any libraries you need here. Add comment to explain what the
 *       library is used for.
 * Ex: #include <math.h>  // used for trigonometric functions sin(), cos()
 * Ex: #include <Wire.h>  // used for I2C communication with devices
 */
 #include <Arduino.h>

/**
 * TODO: Define constants here. There are two common ways to define constants
 *       in C.
 * Ex: #define GREEN_LED_PIN 10
 * Ex: #define PI 3.14
 * Ex: const int GREEN_LED_PIN = 10;
 */
 #define PI 3.14
 const float ANOTHER_PI = 3.14;

/**
 * TODO: Add Global Variables here. Global variables can be accessed throughout
 *       the code.
 */
float globalVariable;


/**
 * Setup
 *
 * @brief: runs once at startup and is used for initialize pins, serial monitor.
 *         keyword 'void' indicates that setup() does not return anything.
 */
void setup() {
  /* Here are some examples of typical setup stuff */

  // pinMode(10, INPUT); // designate digital pin 10 as an input
  // Serial.begin(9600); // start the Serial monitor at 9600 bits/s
  // Serial.println("We are in setup."); // print some message to the serial monitor
  // globalVariable = globalVariable + 10; // increment the globalVariable by 10
}

/**
 * loop
 *
 * @brief: runs continuously and is the main part of the code. This is where
 *         everything happens.
 */
void loop() {
  /* loop Psuedocode
   *
   * 0. pollReceiver();
   *    function runs on interrupt. when a new receiver signal is registered
   *
   * 1. gyroMeasure();
   *    read the gyro and get angular rate and angle data.
   *
   * 2. recordSignals();
   *    record signals coming in from receiver and outputs pulse lengths.
   *
   * 3. PID();
   *    run PID algorithim and output the corrected pulse lengths.
   *
   * 4. writeToMotors();
   *    write corrected pulse lengths to the ESC's to actuate the quadcopter.
   */

}


int calculatePID() {

}
