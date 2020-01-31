#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#include "debug.h"

#define PRINT_SPEED 100 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Weights
#define GYRO_WEIGHT .5
#define ACCEL_WEIGHT 1 - GYRO_WEIGHT
#define kp 1.0 // Proportional constant
#define ki 1.0 // Integral constant
#define kd 1.0 // Derivative constant

#define DESIRED_ANGLE 0
#define TOLERANCE 5
#define throttle 1300

// Prototypes
void calcAngle();
void calcGyroAngle(float);
float fuseAngles();
void calcPid();
float limitPid(float pid);
float limitPwm(float pwm);

// Create the sensor
LSM9DS1 imu;

// Store the roll (rotation about x-axis) and pitch (rotation about y-axis)
float roll, pitch;
float gyroAngle, curAngle;
float angleError, prevError;
static unsigned long lastGyroRead; // Keep track of last time gyro data was read
float pid_p, pid_i, pid_d, PID, pwm_left, pwm_right;

Servo left_motor, right_motor;

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

  // Attach motors
  left_motor.attach(3);
  left_motor.writeMicroseconds(1000);
  delay(5000); // Wait 2 seconds
  right_motor.attach(5);
  right_motor.writeMicroseconds(1000);
  delay(5000); // Wait 2 seconds
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
    Serial.println();

    lastPrint = millis(); // Update lastPrint time
  }
  
  DPRINT(fuseAngles());
  calcPid();
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

void calcPid() {
  curAngle = fuseAngles();
  angleError = curAngle - DESIRED_ANGLE;

  // Calculate P of PID
  pid_p = kp * angleError;

  PID = limitPid(pid_p);
  pwm_left = limitPwm(throttle - PID);
  pwm_right = limitPwm(throttle + PID);

  left_motor.writeMicroseconds(pwm_left);
  right_motor.writeMicroseconds(pwm_right);
}

float limitPid(float pid){
  if(pid < -1000)
  {
    pid = -1000;
  }
  if(pid > 1000)
  {
    pid = 1000;
  }
  return pid;
}

float limitPwm(float pwm){
  if(pwm < 1000)
  {
    pwm = 1000;
  }
  if(pwm > 1500)
  {
    pwm = 1500;
  }
  return pwm;
}
