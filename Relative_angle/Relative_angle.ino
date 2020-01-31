#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#include "debug.h"

#define PRINT_SPEED 100 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Weights
#define GYRO_WEIGHT .5
#define ACCEL_WEIGHT 1 - GYRO_WEIGHT

// Prototypes
void calcAngle();
void calcGyroAngle(float);
float fuseAngles();

// Create the sensor
LSM9DS1 imu;

// Store the roll (rotation about x-axis) and pitch (rotation about y-axis)
float roll, pitch;
float gyroAngle;
static unsigned long lastGyroRead; // Keep track of last time gyro data was read

void setup() {
  Serial.begin(115200);

  Wire.begin();

  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
  }
  
  // Calibrate the Accelerometer
  imu.calibrate();
}

void loop() {
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
    calcGyroAngle((millis() - lastGyroRead) / 1000);
    lastGyroRead = millis();
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel();
    calcAccelAngles(imu.ax, imu.ay, imu.az);
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    DPRINT(fuseAngles());
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }
}

void calcAccelAngles(float ax, float ay, float az) {
  // Calculate Angles from acceleration
  roll = atan2(ay, az); // Rotation about x-axis
  pitch = atan2(-ax, sqrt(ay * ay + az * az)); // Rotation about y-axis

  // Convert to degress
  roll *= RAD_TO_DEG;
  pitch *= RAD_TO_DEG;
}

void calcGyroAngle(float deltaTime) {
  // Calculate the angle from the gyro. Use the y rotation because of mounting posistion and axis on sensor
  gyroAngle = imu.calcGyro(imu.gx) * deltaTime;
}

float fuseAngles() {
  // Use y-axis rotation based on mounting orientation and the sensor axis
  return GYRO_WEIGHT * gyroAngle + ACCEL_WEIGHT * roll;
}
