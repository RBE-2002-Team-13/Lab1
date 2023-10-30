#include <Romi32U4.h>
#include "chassis.h"

float RomiChassis::SpeedLeft(void)
{
    float denom = 0;
    float numerator = 0;
    numerator = 1000 * C_wheel*(count_left-prev_count_left);//[mm] 1000 to convert to sec from mil
    denom = N_wheel * interval;//[s]
    return numerator / denom; //[mm/s]
}

float RomiChassis::SpeedRight(void)
{
    float denom = 0;
    float numerator = 0;
    numerator = 1000 * C_wheel*(count_right-prev_count_right);//[mm] 1000 to convert to sec from mil
    denom = N_wheel * interval; //[s]
    return numerator / denom; //[mm/s]
}
float RomiChassis::getEffortLeft(void)
{
  return u_left;
}
float RomiChassis::getEffortRight(void)
{
  return u_right;
}
float RomiChassis::getErrorLeft(void)
{
  return eLeft;
}
float RomiChassis::getErrorRight(void)
{
  return eRight;
}

void RomiChassis::UpdateEffortDriveWheels(int left, int right)
{ 
    motors.setEfforts(left,right);
}

void RomiChassis::UpdateEffortDriveWheelsPI(int target_speed_left, int target_speed_right)
{
  // !!! ATTENTION !!!
  // Assignment 2
  {
    eRight = target_speed_right - SpeedRight();
    eLeft = target_speed_left - SpeedLeft();
    Esum_right = Esum_right + eRight;
    Esum_left = Esum_left + eLeft;
    u_right = Kp*eRight + Ki*Esum_right;
    u_left = Kp*eLeft + Ki*Esum_left;
    motors.setEfforts(u_left, u_right);
  }
}

void RomiChassis::SerialPlotter(float a, float b, float c, float d, float e, float f, float g)
{
    // !!! ATTENTION !!!
    // USE this function for assignment 3!
    Serial.print(a);
    Serial.print(',');
    Serial.print(b);
    Serial.print(',');
    Serial.print(c);
    Serial.print(',');
    Serial.print(d);
    Serial.print(',');
    Serial.print(e);
    Serial.print(',');
    Serial.print(f);
    Serial.print(',');
    Serial.print(g);
    Serial.println();
}

void RomiChassis::MotorControl(void)
{
  uint32_t now = millis();
  if(now - last_update >= interval)
  {    
    prev_count_left = count_left;
    prev_count_right = count_right;
    count_left = encoders.getCountsLeft();
    count_right = encoders.getCountsRight();
    previous_time = millis();
    UpdateEffortDriveWheelsPI(target_left, target_right);
    last_update = now;
  }
}

void RomiChassis::StartDriving(float left, float right, uint32_t duration)
{
  target_left = left; target_right = right;
  start_time = millis();
  last_update = start_time;
  end_time = start_time + duration; //fails at rollover
  Esum_left = 0;
  Esum_right = 0;
}

bool RomiChassis::CheckDriveComplete(void)
{
  return millis() >= end_time;
}

void RomiChassis::Stop(void)
{
  target_left = target_right = 0;
  motors.setEfforts(0, 0);
}