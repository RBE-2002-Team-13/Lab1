#include <Arduino.h>
#include <Romi32U4.h>
#include "chassis.h"

RomiChassis chassis;

enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVING};
ROBOT_STATE robot_state = ROBOT_IDLE;

long time = 0;

Romi32U4ButtonA buttonA;

void setup() {
    Serial.begin(115200);
}

void loop() 
{
  switch(robot_state)
  {
    case ROBOT_IDLE:
      if(buttonA.getSingleDebouncedRelease()) 
      {
        time = millis();
        chassis.StartDriving(50, 50, 10000); //contains your program that the robot executes when pushbutton A is pressed
        robot_state = ROBOT_DRIVING;
      }
      break;

    case ROBOT_DRIVING:
      chassis.MotorControl();
      if((millis() - time) % 100 == 0){
        chassis.SerialPlotter(chassis.SpeedLeft(), chassis.SpeedRight(), chassis.getEffortLeft(), chassis.getEffortRight(), chassis.getErrorLeft(), chassis.getErrorRight(), (millis()-time));
      }
      if(chassis.CheckDriveComplete()) 
      {
        chassis.Stop();
        robot_state = ROBOT_IDLE;
      }
      if(buttonA.getSingleDebouncedRelease()) 
      {
        chassis.Stop();
        robot_state = ROBOT_IDLE;
      }
  }
}