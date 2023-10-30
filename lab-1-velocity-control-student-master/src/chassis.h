#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <Romi32U4.h>

class RomiChassis{
    private:
        //Romi specific constants:
        // !!! ATTENTION !!!
        const float N_wheel = 1440; //how many counts equate to one wheel rotation?
        const float R_wheel = 35; //what is the radius of a Romi wheel in [mm]
        const float C_wheel = 2*PI*R_wheel; //circumference of wheel

        //declare variables for PI controller
        float target_left = 0;
        float target_right = 0;
        float Kp = .7;
        float Ki = 0.3;
        float Esum_left = 0; //accumulated errors
        float Esum_right = 0;
        float eLeft = 0;
        float eRight= 0;
        float u_right = 0;
        float u_left = 0;

        //encoder and motor objects
        Romi32U4Encoders encoders;
        Romi32U4Motors motors;

        //declare variables for for keeping track of counts and conversion to velocities
        uint32_t start_time = 0;
        uint32_t end_time = 0;
        float interval = 50; // in [ms]
        uint32_t last_update = 0;
        int count_left = 0;
        int count_right = 0;
        int prev_count_left = 0;
        int prev_count_right = 0;
        float previous_time = 0;


    public:
        float SpeedLeft(void);
        float SpeedRight(void);
        float getEffortLeft(void);
        float getEffortRight(void);
        float getErrorLeft(void);
        float getErrorRight(void);

        void UpdateEffortDriveWheels(int a, int b);
        void UpdateEffortDriveWheelsP(int a, int b);
        void UpdateEffortDriveWheelsPI(int a, int b);

        void MotorControl(void);
        void SerialPlotter(float a, float b, float c, float d, float e, float f, float g);

        void StartDriving(float, float, uint32_t);
        bool CheckDriveComplete(void);
        void Stop(void);
};

extern RomiChassis chassis;

#endif