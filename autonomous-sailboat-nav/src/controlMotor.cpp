#include "controlMotor.h"

controlMotor::controlMotor() : pwm(Adafruit_PWMServoDriver()) {
  angle_rudder = 0;
  angle_sail = 0;
  Serial.println("Initialising Motors...");
  pwm.begin();
  Serial.println("pwm ready...");
  pwm.setPWMFreq(50); // Defines the frequency we'll use to communicate with the servos.
  delay(10);
}

int controlMotor::get_com_rud(){ // Rudder command getter
  return com_rud;
}

int controlMotor::get_com_sail(){ // Sail command getter
  return com_sail;
}

void controlMotor::set_angle_sail(int angle){ // Sets the sail position to the given angle. Angle must be given in degrees and be between 0 and 90 degrees.
  com_sail = map(angle, 0, 90, SERVOMAX_SAIL, SERVOMIN_SAIL);
  //Serial.print("Sail command:");
  //Serial.println(com_sail);
  pwm.setPWM(SERVO_SAIL, 0, com_sail);
}

void controlMotor::set_angle_rudder(int angle) // Sets the rudder position to the given angle. Angle must be given in degrees and be between 0 and 50 degrees.
{
  com_rud = map(angle, -50, 50, SERVOMAX_RUDDER, SERVOMIN_RUDDER);
  pwm.setPWM(SERVO_RUDDER, 0, com_rud);
}

void controlMotor::send_com_rudder(int com){
  com = constrain(com, SERVOMIN_RUDDER, SERVOMAX_RUDDER);
  pwm.setPWM(SERVO_RUDDER, 0, com);
  com_rud = com;
}

void controlMotor::send_com_sail(int com){
  com = constrain(com, SERVOMIN_SAIL, SERVOMAX_SAIL);
  pwm.setPWM(SERVO_SAIL, 0, com);
  com_sail = com;
}