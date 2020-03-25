#include <Arduino.h>
#include <Servo.h>
#include "Serial_Receive.h"

EulerAngle angles;
extern boolean newData;
extern char receivedChars[];
extern char tempChars[];

//Motor constants
static const int MOTOR_OUT_MIN = 1100;
static const int MOTOR_OUT_MAX = 1450;
static const int throttle = 1275;
static const float ANGLE = 7.0f;

// PI
double k_p = 0.5;
double k_i = 0.075;
float pid_p; // Proportional part of the PID per axis (x, y)
float pid_i; // Integral part of the PID per axis (x, y)
float PID; // Total PID values per axis (x, y)
float leftPWM, rightPWM;
float prevAngleError, curAngleError, desiredAngle= 6.0f;

Servo rMotor, lMotor;

void balanceMotors();
float limitPWM(float);
void calcPI();

void setup()
{
  Serial.begin(1000000);
  rMotor.attach(3); // right motor
  lMotor.attach(5); // left motor
  rMotor.writeMicroseconds(MOTOR_OUT_MIN);
  lMotor.writeMicroseconds(MOTOR_OUT_MIN);
  delay(2000);
  while (!Serial)
  {
  };
  delay(10000); // Wait for 10 seconds to calibrate sensor
}

void loop()
{
  recvWithStartEndMarkers();
  if (newData == true)
  {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData(angles);
    showParsedData(angles);
    newData = false;

    // Do motor balancing here, after the new data was obtained
    balanceMotors();
  }
}

void balanceMotors()
{
  calcPI();
  lMotor.writeMicroseconds(leftPWM);
  rMotor.writeMicroseconds(rightPWM);
}

// CLimit PWM to min and max
float limitPWM(float pwm)
{
  if (pwm < MOTOR_OUT_MIN)
  {
    pwm = MOTOR_OUT_MIN;
  }
  if (pwm > MOTOR_OUT_MAX)
  {
    pwm = MOTOR_OUT_MAX;
  }
  return pwm;
}

void calcPI()
{
  curAngleError = angles.pitch - desiredAngle;
  // PID Calculation
  // Proportional
  pid_p = k_p * curAngleError;

  // Integral
  // Only function on small angles to tune the error
  if (curAngleError > -2.0 && curAngleError < 2.0)
  {
    pid_i = pid_i + (k_i * curAngleError);
  }

  // Total PID is sum of all PIDs, and limit the pid to proper values
  PID = pid_p + pid_i;

  // Calculate the pwm from the throttle and PID, and limit it to not mess up the ESC
  leftPWM = limitPWM(throttle + PID);
  rightPWM = limitPWM(throttle - PID);

  // Store previous error
  prevAngleError = curAngleError;
}