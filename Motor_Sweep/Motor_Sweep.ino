/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2019
 by William Blanc
*/

#include <Servo.h>
//#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)    //DPRINT is a macro, debug print
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)    //now defines a blank line
  #define DPRINTLN(...)   //now defines a blank line
#endif

#define SWEEP_SIZE 64
#define DELAY 100 // Delay is in ms
static const int VCC = 5;
static const int MOTOR_OUT_MIN = 50;
static const int MOTOR_OUT_MAX = 90;

int motor_sweep[SWEEP_SIZE]; // Values to control the motor automatically
int motor_out; // Output for the motor
char str[32]; // Output string buffer

Servo motor1;  // create servo object to control the motor
Servo motor2;

void setup() {
  Serial.begin(115200);
  Serial.println("Outputting to motor:");
  while (!Serial){}
  #ifndef DEBUG // Attach motors if not debugging
    motor1.attach(3);  // attaches the servo on pin 9 to the servo object
    motor2.attach(5);
    motor1.write(MOTOR_OUT_MIN);
    motor2.write(MOTOR_OUT_MIN);
    delay(2000);
  #endif
  
  // Fill the array with values from 50 to 90
  for (int i = MOTOR_OUT_MIN; i <= MOTOR_OUT_MAX; i+=1){
    motor_sweep[i - MOTOR_OUT_MIN] = i; 
    // DEBUG CODE
    DPRINT("motor_sweep[");
    DPRINT(i - MOTOR_OUT_MIN);
    DPRINT("] = ");
    DPRINTLN(motor_sweep[i - MOTOR_OUT_MIN]);
  }
  motor2.write(70);
}

void loop() {
  for (int i = 0; i < SWEEP_SIZE; i++){
    motor_out = motor_sweep[i];
    if (motor_out == 0)
      continue;
    sprintf(str, "Val: %d", motor_out);
    Serial.println(str);
    #ifndef DEBUG
      motor1.write(motor_out);                  // sets the servo position according to the scaled value
      //motor2.write(motor_out);
      switch (motor_out){
        case MOTOR_OUT_MIN:
        case MOTOR_OUT_MAX:
          delay(10*DELAY); // waits for the motor to get there
          break;
        default:
          delay(DELAY);
          break;
      }
    #endif
  }
}
