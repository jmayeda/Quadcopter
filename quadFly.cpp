/*******************************************************************************
 * @file: quadcopter.cpp
 *
 * @author:
 * @about: Quadcopter flight controller for the ESW Quarterly Project 2018.
 * @date:
 ******************************************************************************/

#include <Wire.h> // I2C communication with devices
#include <Arduino.h>
#include <EnableInterrupt.h>  // interrupts for reading receiver signals

/*******************************************************************************
 * Constants
 * TODO: Define constants here. There are two common ways to define constants
 *       in C/C++.
 * Ex: #define GREEN_LED_PIN 10
 * Ex: #define PI 3.14
 * Ex: const int GREEN_LED_PIN = 10;
 ******************************************************************************/
#define RC_NUM_CHANNELS 4
// channel numbers for the receiver 0+
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 2
#define RC_CH4 3

// IMU constants 
const float ACCEL_RAW_SCALER = 16384.0;  // NOTE: see datasheet for FSR, resolution
const float GYRO_RAW_SCALER = 131.0;     // NOTE: see datasheet for FSR, resolution
const int BAUD_RATE = 9600;

/*******************************************************************************
 * TODO: Define Structs or other custom variable types here.
 ******************************************************************************/
typedef struct gyro_t {
  float roll;    // angular rotation in the X direction
  float pitch;   // angular rotation in the Y direction
  float yaw;     // angular rotation in the Z direction
  float gForceX; // linear acceleration in the local X direction
  float gForceY; // linear acceleration in the local Y direction
  float gForceZ; // linear acceleration in the local Z direction
}gyro_t;

typedef struct Pid_t{
  float pgain;  // proportional gain
  float igain;  // integral gain
  float dgain;  // derivative gain
  float input;  // input: our current angular rotation rate (deg/s)
  int max;
  int setpoint;
  float error;
  float memory;
  float last_error;
  float output;
}Pid_t;

typedef struct rc_signals_t {
  uint16_t CH1;
  uint16_t CH2;
  uint16_t CH3;
  uint16_t CH4;
}rc_signals_t;

typedef enum arm_state_t{
  ARMED,
  DISARMED
}arm_state_t;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
// Initialize arrays for receiver data storing
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
// some intermediate global variables for hyroscope
long accelX, accelY, accelZ; // raw accelerometer data
long gyroX, gyroY, gyroZ;    // raw gyro data

rc_signals_t rc_signal;
arm_state_t arm_state;
Pid_t theta;  // roll
Pid_t phi;    // pitch
Pid_t psi;    // yaw
gyro_t mpu6050;

/*******************************************************************************
 * Function Declarations
 ******************************************************************************/
void setupMPU6050();
void readAccelData(gyro_t *gyro);
void readGyroData(gyro_t *gyro);
void printData(gyro_t *gyro);

// calc_input to read each channel when interrupts occur
void calc_ch1() { calc_input(RC_CH1, PB0); }
void calc_ch2() { calc_input(RC_CH2, PB1); }
void calc_ch3() { calc_input(RC_CH3, PB2); }
void calc_ch4() { calc_input(RC_CH4, PB3); }

void calculate_pid(Pid_t *angle);
void rc_read_values();
void calc_ch1();
void calc_ch2();
void calc_ch3();
void calc_ch4();

/*******************************************************************************
 * void setup()
 *
 * @brief: Setup runs once and is used to setup and initialize all of the pins,
 *         and methods we will use later.
 ******************************************************************************/
void setup()
{
   // put your setup code here, to run once:
   Serial.begin(BAUD_RATE);

   DDRB &= B00000000; // set D8 to D13 as inputs

   enableInterrupt(8, calc_ch1, CHANGE);
   enableInterrupt(9, calc_ch2, CHANGE);
   enableInterrupt(10, calc_ch3, CHANGE);
   enableInterrupt(11, calc_ch4, CHANGE);
   Wire.begin();
   setupMPU6050();

  // put your setup code here, to run once:
  theta.pgain = 1;
  theta.igain = 1;
  theta.dgain = 1;
  theta.input = mpu6050.roll;
  theta.max = 400;
  theta.setpoint = 0;

  phi.pgain = 1;
  phi.igain = 1;
  phi.dgain = 1;
  phi.input = mpu6050.pitch;
  phi.max = 400;
  phi.setpoint = 0;

  psi.pgain = 1;
  psi.igain = 1;
  psi.dgain = 1;
  psi.input = mpu6050.yaw;
  psi.max = 400;
  psi.setpoint = 0;

}

/*******************************************************************************
 * void loop()
 *
 * @brief: runs continuously and is the main part of the code. This is where
 *         everything happens.
 ******************************************************************************/
void loop()
{
   // read the accelerometer and pass the data to the gyro struct mpu6050
  readAccelData(&mpu6050);
  // read the gyro and pass the data to the gyro struct
  readGyroData(&mpu6050);

  rc_read_values();

  // print the data to the serial monitor for debugging
  printData(&mpu6050);

  theta.input = mpu6050.roll;
  phi.input   = mpu6050.pitch;
  psi.input   = mpu6050.yaw;

  delay(1100);

  // printing for debugging
  Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
  Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);

  delay(1200);

  calculate_pid(&theta);
  calculate_pid(&phi);
  calculate_pid(&psi);

  Serial.println(theta.output);
  Serial.println(theta.input-theta.setpoint);
  Serial.println(phi.output);
  Serial.println(phi.input-phi.setpoint);
  Serial.println(psi.output);
  Serial.println(psi.input-psi.setpoint);

  delay(1500);
}

void calculate_pid(Pid_t *angle)
{
  angle->error = angle->input - angle->setpoint;
  angle->memory += angle->igain * angle->error;
  if( angle->memory > angle->max ) {
      angle->memory = angle->max;
  } else if (angle->memory < angle->max * -1) {
      angle->memory = angle->max * -1;
  }
  angle->output = (angle->pgain * angle->error) + (angle->memory) + (angle->dgain * (angle->error - angle->last_error));
  angle->memory = angle->last_error;
};

void setupMPU6050() {
  Wire.beginTransmission(0b1101000); // this is the I2C address of the MPU
  Wire.write(0x6B); // Access register 6B - power management
  Wire.write(0b0000000); // setting sleep register to 0
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s; note that higher degree registers have lower sensitivity
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  // Wire.write(0b00011000) set accel to +/- 16g by changing bit 3 and 4 to '11' = 3 in binary (see datasheet)
  Wire.endTransmission();
}

void readAccelData(gyro_t *gyro)
{
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x3B); // write to register for Accel Readings
  Wire.endTransmission();
  // Request Accel Registers (3B - 40) these are the 6 acceleration registers where data is stored
  Wire.requestFrom(0b1101000,6);

  while(Wire.available() < 6);
  //Serial.print("setup");
  // read the raw accelermeter data from the IMU
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

  // NOTE: see datasheet for more information
  gyro->gForceX = accelX / ACCEL_RAW_SCALER;
  gyro->gForceY = accelY / ACCEL_RAW_SCALER;
  gyro->gForceZ = accelZ / ACCEL_RAW_SCALER;
}

/**
 * @brief: read gyro data from the MPU6050 and process data according
 *         to desired resolution and full scale range.
 * @param gyro pointer to struct that holds the MPU6050 sensor data.
 */
void readGyroData(gyro_t *gyro)
{
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)

  while(Wire.available() < 6);

  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();

  // We have to scale the gyro data because the raw data is not particularly
  // useful. GYRO_RAW_SCALER is determined in the datasheet according to the
  // desired Full scale range (FSR) and resolution.
  gyro->roll = gyroX / GYRO_RAW_SCALER;
  gyro->pitch = gyroY / GYRO_RAW_SCALER;
  gyro->yaw = gyroZ / GYRO_RAW_SCALER;
}

void printData(gyro_t *gyro)
{
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(gyro->roll);
  Serial.print(" Y=");
  Serial.print(gyro->pitch);
  Serial.print(" Z=");
  Serial.print(gyro->yaw);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gyro->gForceX);
  Serial.print(" Y=");
  Serial.print(gyro->gForceY);
  Serial.print(" Z=");
  Serial.println(gyro->gForceZ);
}

// Function to read and copy values from rc_shared into array rc_values
void rc_read_values()
{
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

// Function to calculate the pulse width of each incoming signal
void calc_input(uint8_t channel, uint16_t input_pin)
{
  if ((PINB & (1 << input_pin))  >  0) {
    rc_start[channel] = micros(); // obtain time position when pin goes HIGH
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]); // obtain time position when pin goes LOW and calculate time difference for pulse width
    rc_shared[channel] = constrain(rc_compare,1000,2000); // constrain values between 1000-2000ms
  }
}
