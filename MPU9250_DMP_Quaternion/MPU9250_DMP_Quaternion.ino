/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB
#define ERROR_TOLERANCE 2 // 2 degrees tolerance
#define ERROR_SCALE 0.6558  // Slope from measurements
MPU9250_DMP imu;

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles angles;

float rollScaled, pitchScaled, yawScaled;


void setup() 
{
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               20); // Set DMP FIFO rate to 20 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      printData(imu.pitch, imu.roll, imu.yaw);
      calcEulerAngles();
      printData(degrees(angles.roll), degrees(angles.pitch), degrees(angles.yaw));
    }
  }
}

void printData(float p, float r, float y)
{ 
  //rollScaled = scaleAngle(imu.roll);
  //pitchScaled = scaleAngle(imu.pitch);
  //yawScaled = scaleAngle(imu.yaw);
  
  SerialPort.println("R/P/Y: " + String(r) + ", "
            + String(p) + ", " + String(y));
  SerialPort.println();
}

float scaleAngle(float eularAngle)
{
  if (eularAngle < ERROR_TOLERANCE || eularAngle > 360 - ERROR_TOLERANCE )
    return 0;
  if (eularAngle > 180)
    return (eularAngle - 360) * ERROR_SCALE;
  else
    return eularAngle * ERROR_SCALE;
}

void calcEulerAngles() {

    float dqw = imu.calcQuat(imu.qw);
    float dqx = imu.calcQuat(imu.qx);
    float dqy = imu.calcQuat(imu.qy);
    float dqz = imu.calcQuat(imu.qz);

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (dqw * dqx + dqy * dqz);
    double cosr_cosp = 1 - 2 * (dqx * dqx + dqy * dqy);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (dqw * dqy - dqz * dqx);
    if (abs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (dqw * dqz + dqx * dqy);
    double cosy_cosp = 1 - 2 * (dqy * dqy + dqz * dqz);
    angles.yaw = atan2(siny_cosp, cosy_cosp);
}
