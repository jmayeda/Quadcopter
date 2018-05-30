/*******************************************************************************
 * @file: quadcopter.cpp
 *
 * @author:
 * @about: Quadcopter flight controller for the ESW Quarterly Project 2018.
 * @date:
 ******************************************************************************/

/**
 * TODO: Add any libraries you need here. Add comments to explain what the
 *       library is used for.
 * Ex: #include <math.h>  // used for trigonometric functions sin(), cos()
 * Ex: #include <Wire.h>  // used for I2C communication with devices
 */
#include <Wire.h> // I2C communication with devices

/*******************************************************************************
 * TODO: Define constants here. There are two common ways to define constants
 *       in C.
 * Ex: #define GREEN_LED_PIN 10
 * Ex: #define PI 3.14
 * Ex: const int GREEN_LED_PIN = 10;
 ******************************************************************************/
#define PI 3.14
const float ANOTHER_PI = 3.14;

/*******************************************************************************
 * TODO: Define Structs or other custom variable types here.
 ******************************************************************************/

/**
 * enum to designate whether or not the quadcopter motors are armed.
 */
typedef enum arm_state_t{
  ARMED,
  DISARMED
}arm_state_t;

/**
 * Struct to hold all receiver signals. Should be 1000-2000 us pulses
 */
typedef struct rc_signals_t {
  uint16_t CH1;
  uint16_t CH2;
  uint16_t CH3;
  uint16_t CH4;
}rc_signals_t;

/**
 * Struct to hold the roll, pitch, yaw deg/s data from the MPU6050.
 */
typedef struct gyro_t {
  float roll;
  float pitch;
  float yaw;
}gyro_t;

/*******************************************************************************
 * Global Variables
 *
 ******************************************************************************/
gyro_t gyro;
rc_signals_t rc_signal;
arm_state_t arm_state;

/*******************************************************************************
 * Function Declaration
 *
 ******************************************************************************/
void readGyro(gyro_t *gyro);
int calculatePID();

/**
 *
 * @brief: Setup runs once and is used to setup and initialize all of the pins, and
 *         methods we will use later.
 */
void setup()
{
  /* TODO: Add anything needed for setup here. */

  /* Here are some examples of typical setup stuff */

  // pinMode(10, INPUT); // designate digital pin 10 as an input
  // Serial.begin(9600); // start the Serial monitor at 9600 bits/s
  // Serial.println("We are in setup."); // print some message to the serial monitor
  // globalVariable = globalVariable + 10; // increment the globalVariable by 10
}

/**
 *
 * @brief: runs continuously and is the main part of the code. This is where
 *         everything happens.
 */
void loop()
{
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

/**
 * @brief: calculate the PID outputs for the roll, pitch, and yaw controllers.
 * @return [description]
 */
int calculatePID()
{
  /* TODO: */
  return 0;
}

/**
 * @brief: Read the gyro angular rates in deg/s and pass to gyro data struct.
 * @param gyro pointer to struct to hold gyro angular rates.
 */
void readGyro(gyro_t *gyro)
{
  // in C, "static" keyword initializes variable once and value persists between calls
  static float gyroRaw[3];

  /* TODO: read gyro here */
  gyroRaw[3] = {99.0, 99.0, 99.0}; // NOTE: placeholder; remove

  // add gyro data to the gyro signal struct
  gyro->roll  = gyroRaw[0];
  gyro->pitch = gyroRaw[1];
  gyro->yaw   = gyroRaw[2];
}
