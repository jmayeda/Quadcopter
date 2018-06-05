/**
 * @file: MPU6050_bare_minimum.ino
 *
 * @brief: read the MPU 6050 for gyro and accerolerometer data.
 */

/* Libraries */
#include <Wire.h> // Wire library for I2C Communication

/* Define Constants */
const float ACCEL_RAW_SCALER = 16384.0;  // NOTE: see datasheet for FSR, resolution
const float GYRO_RAW_SCALER = 131.0;     // NOTE: see datasheet for FSR, resolution
const int BAUD_RATE = 9600;

/* Define Variable Types */
typedef struct gyro_t {
  float roll;    // angular rotation in the X direction
  float pitch;   // angular rotation in the Y direction
  float yaw;     // angular rotation in the Z direction
  float gForceX; // linear acceleration in the local X direction
  float gForceY; // linear acceleration in the local Y direction
  float gForceZ; // linear acceleration in the local Z direction
}gyro_t;

/* Global Variables */
// some intermediate global variables
long accelX, accelY, accelZ; // raw accelerometer data
long gyroX, gyroY, gyroZ;    // raw gyro data

// initialize a struct of type "gyro_t" to hold our gyro data
gyro_t mpu6050;

/* Function Declaration */
void setupMPU6050();
void readAccelData(gyro_t *gyro);
void readGyroData(gyro_t *gyro);
void printData(gyro_t *gyro);

/**
 * @brief: setup runs once and initializes communication with the Gyro.
 */
void setup()
{
  Serial.begin(BAUD_RATE);
  Wire.begin();
  setupMPU6050();
}

/**
 * @brief: loop() runs continuously and prints the gyro and accelerometer data.
 */
void loop()
{
  // read the accelerometer and pass the data to the gyro struct mpu6050
  readAccelData(&mpu6050);
  // read the gyro and pass the data to the gyro struct
  readGyroData(&mpu6050);
  // print the data to the serial monitor for debugging
  printData(&mpu6050);
  delay(100);
}

/**
 * @brief: setup the MPU6050 for reading gyro data and setting for resolution.
 */
void setupMPU6050()
{
  Wire.beginTransmission(0b1101000) // this is the I2C address of the MPU
  Wire.write(0x6B);                 // Access register 6B - power management
  Wire.write(0b0000000);            // setting sleep register to 0
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);// I2C address of the MPU
  Wire.write(0x1B); // Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); // Setting the gyro to full scale +/- 250deg./s; note that higher degree registers have lower sensitivity
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  // Wire.write(0b00011000) set accel to +/- 16g by changing bit 3 and 4 to '11' = 3 in binary (see datasheet)
  Wire.endTransmission();
}

/**
 * @brief: read accelerometer data from the MPU6050 and process data according
 *         to desired resolution and full scale range.
 * @param gyro pointer to struct that holds the MPU6050 sensor data.
 */
void readAccelData(gyro_t *gyro)
{
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x3B); // write to register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); // Request Accel Registers (3B - 40) these are the 6 acceleration registers where data is stored

  while(Wire.available() < 6);

  // read the raw accelermeter data from the IMU
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

  // 16384 comes from the least significant bit rate at +/- 2g
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
