/**
 * @file: MPU6050_gyro_read.ino
 *
 * @brief: read the MPU 6050 gyroscope, and display yaw, pitch, roll values.
 *
 * @hardware:
 *  MPU6050 --> Arduino
 *     VCC  -->   3.3V
 *     GND  -->   GND
 *     SCL  -->   A5
 *     SDA  -->   A4
 *     INT  -->   D2
 */

/* Libraries */
#include "I2Cdev.h"  // for I2C communications with our gyro
#include "MPU6050_6Axis_MotionApps20.h"

/* MPU 6050 Object */
MPU6050 mpu;         // create the MPU6050 object

/* Pin definitions & Constants */
#define INTERRUPT_PIN 2   // Arduino Uno external interrupt on pin 2
#define LED_PIN       13  // LED pin for status
#define BAUD_RATE     115200

/* MPU Control/status */
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

/* Orientation/Motion */
Quaternion q;           // Quaternion container
VectorFloat gravity;
float ypr[3];           // [yaw, pitch, roll] yaw pitch roll container

/* Interrupt Detection Routine */
volatile bool mpuInterrupt = false;  // volatile keyword indicates that
                                     // variable can change at any time

/*******************************************************************************
 * void dmpDataReady()
 *
 * @brief: This routine operates on an interrupt. Indicates whether or not the
 *         MPU interrupt pin has gone HIGH.
 ******************************************************************************/
void dmpDataReady() {
  mpuInterrupt = true;
}

/*******************************************************************************
 * void setup()
 *
 * @brief: setup() is one of two functions that are mandatory in all Arduino
 *         programs. Runs once.
 ******************************************************************************/
void setup() {
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();                 // initialize the MPU6050
  devStatus = mpu.dmpInitialize();  // initialize the DMP

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // set INTERRUPT_PIN as an interrupt for reading MPU values
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  packetSize = mpu.dmpGetFIFOPacketSize();

  Serial.begin(BAUD_RATE); // start the serial monitor foe debugging
  while(!Serial);          // wait for serial to start up
}

/*******************************************************************************
 * void loop()
 *
 * @brief:
 ******************************************************************************/
void loop() {
  // if setup failed, don't do anything
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize);

  // reset the interrupt flag and get the INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // check for overflow
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);


}
