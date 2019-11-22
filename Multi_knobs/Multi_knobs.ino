/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

static const int VCC = 5;
Servo motor1;  // create servo object to control the motor
Servo motor2;

int potpin = 0;  // analog pin used to connect the potentiometer
int potpin2 = 1;
int analog;
int analog2;
int val;    // variable to read the value from the analog pin
int val2;

char str[32];

void setup() {
  Serial.begin(9600);
  Serial.println("Analog\tMapped");
  motor1.attach(9);  // attaches the servo on pin 9 to the servo object
//  motor2.attach(10);
}

void loop() {
  analog = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  analog2 = analogRead(potpin2);
  val = map(analog, 0, 1083, 50, 90);     // scale it to use it with the servo (value between 0 and 180)
  sprintf(str, "Val: %d", val);
  Serial.println(str);
  val2 = constrain(analog2, 50,90);
  motor1.write(val);                  // sets the servo position according to the scaled value
//  motor2.write(val2);
  delay(15);                           // waits for the servo to get there
}
