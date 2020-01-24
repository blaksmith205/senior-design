// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <SparkFunLSM9DS1.h>

// Number of axis to track
#define AXIS 1 // Can be 1 or 2

// Sensor
LSM9DS1 imu;

// Motors
Servo motor[2 * AXIS]; // 2 motors per axis (x_left, x_right, y_left, y_right)
int motorPins[] = {5, 3}; // PINS each motor is attached to. Same order as motors

// Addresses of sensor magnetometer, accelerometer, and gyroscope 
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Constants
#define GYRO_WEIGHT 0.98
#define ACCEL_WEIGHT 0.02
double k_p = 1;
double k_i = 1;
double k_d = 1;

// Output macros
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Function definitions
float limitPID(float pid);
float limitPWM(float pwm);
void updateRotation(float ax, float ay, float az);
void calcPID();
void printAccel();
void printRotation();

// Variables
float throttle = 1300; // Initial value to send to motor
static float DESIRED_ANGLE = 0; // Desired angle of axis (x, y)
float accelAngle[2]; // angles from accelerometer (roll, pitch)
float gyroAngle[2]; // gyro dps * elapsed time (x, y)
float totalAngle; // Current angle of the system (x, y)
float angleError; // How far off the current angle is from the desired angle (x, y)
float prevError; // Store previous error per axis (x, y)
int16_t *accelData[AXIS]; // pointers to raw acceleration data (x, y)
int16_t *gyroData[AXIS];  // pointer to raw gyro data (x, y)

float pid_p; // Proportional part of the PID per axis (x, y)
float pid_i; // Integral part of the PID per axis (x, y)
float pid_d; // Derivative part of the PID per axis (x, y)
float PID; // Total PID values per axis (x, y)
float pwm[2 * AXIS]; // PWM vlues per motor per axis (x_left, x_right, y_left, y_right)
float prevTime;

void setup() 
{
  
  Serial.begin(115200);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  // Attach motors to pins
  for (int8_t i = 0; i < 2 * AXIS; i++){
    motor[i].attach(motorPins[i]);
    motor[i].writeMicroseconds(1000);
    delay(1000);
  }
  prevTime = millis();
}

void loop() {
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel();
    updateRotation(imu.ax, imu.ay, imu.az);
  }
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    //printAccel();
    //printRotation();
    //Serial.println();
    lastPrint = millis(); // Update lastPrint time
  }
  calcPID(millis() - prevTime);
  prevTime = millis();
}

void updateRotation(float ax, float ay, float az)
{
  // Obtain the angle from acceleration. From LSM9DS1_Basic_I2C example. 
  accelAngle[0] = atan2(ay, az);
  accelAngle[1] = atan2(-ax, sqrt(ay * ay + az * az));

  // Convert from radians to degrees
  accelAngle[0] *= RAD_TO_DEG;
  accelAngle[1] *= RAD_TO_DEG;
}

// Assuming the values from sensor have been updated before calling this method
void calcPID(unsigned long deltaTime)
{
    /* To self balance a plane every loop:
     * 1. Calculate the roll (around x axis) and pitch (around y axis) from acceleration
     * 2. Obtain angle dps from gyro in x and y direction
     * 3. Integrate gyro_x, gyro_y (multiply by delta time since last read)
     * 4. Add previous gyro angles per axis with new value
     * 5. Weighted sum of result from 4 and result from 1 per axis for an approx_angle (sensor fusion)
     * 6. Find current error from desired angle per axis (error_axis = angle_axis - desired_angle_axis)
     * 7. Proportional of PID: multiply error by a constant (k_1*error)
     * 8. Integral of PID: multiply error by a constant and add to previous intgral value (running sum of k_2*error)
     * 9. Derivative of PID: k_3(current_error - previous_error / delta time)
     * 10. PID = sum(results from 7,8,9)
     * 11. Left_motor_axis pwm = speed (constant) + PID. value should be 1000<=<x<=2000
     * 12. Right_motor_axis pwm = speed (constant) - PID. value should be 1000<=x<=2000
     * 13. Send the pwm signals to the motors
     * 14. record current angle as previous angle
     */
   
  //for (unsigned char i = 0; i < AXIS; i++){
      // Update and obtain the angle from gyro
      gyroAngle[0] = imu.calcGyro(*(gyroData[0])) * deltaTime;

      // Fuse sensor data together to get ~ angle in axis and obtain the error angles
      totalAngle = GYRO_WEIGHT *(totalAngle + gyroAngle[0]) + ACCEL_WEIGHT * accelAngle[0];
      angleError = totalAngle - DESIRED_ANGLE;
      Serial.println(totalAngle);
//      // PID Calculation
//      // Proportional
//      pid_p = k_p * angleError;
//
//      // Integral
//      // Only function on small angles to tune the error
//      if(-3 < angleError <3)
//      {
//        pid_i = pid_i + (k_i*angleError);  
//      }
//
//      // Derivative
//      pid_d = k_d*((angleError - prevError) / deltaTime);
//    
//      // Total PID is sum of all PIDs, and limit the pid to proper values
//      PID = limitPID(pid_p + pid_i + pid_d);
//    
//      // Calculate the pwm from the throttle and PID, and limit it to not mess up the ESC
//      pwm[0] = limitPWM(throttle + PID);
//      pwm[1] = limitPWM(throttle - PID);
      
//      // Store previous error
//      prevError = angleError;
//  //}
//  
//  // Send the PWM pulse to the motors
//  for (int8_t i = 0; i < 2 * AXIS; i++){
//    motor[i].writeMicroseconds(pwm[i]);
//  }
}

float limitPID(float pid){
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

float limitPWM(float pwm){
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

void setupPointers(){
      accelData[0] = &imu.ax;
      gyroData[0] = &imu.gx;
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif
}

void printRotation()
{  
  Serial.print("roll: ");
  Serial.print(accelAngle[0], 2);
  Serial.println(" degrees");
  Serial.print("pitch: ");
  Serial.print(accelAngle[1], 2);
  Serial.println(" degrees");
}
