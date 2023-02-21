#include "datastructs.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <vector>
using namespace pros;
using namespace std; 

#ifndef DATA_H
#define DATA_H

extern struct CoreData info;
extern pros::Controller Con;
extern ADIEncoder Encoder;
//extern pros::Motor RightDriveMotor1;
//extern pros::Motor RightDriveMotor2;

//extern pros::motor_group LeftDriveTrain;
//extern pros::motor_group RightDriveTrain;
extern Motor_Group RightDrive;
extern Motor_Group LeftDrive;
extern ADIDigitalIn limitSwitch;
extern Motor IntakeM;
extern Motor CataM;
extern Imu InertialSensor;

extern const double ENC_L_DIST;
extern const double ENC_R_DIST;
extern const double WHEEL_RAD;
extern const double FlywheelSpeed;
extern const float ANGLE_SCALE_VAL;
extern const double AutonSpeed;
extern const double IntakeSpeed;
extern const double IntakeSpeedSlowed;
extern ADIDigitalOut Cat;
extern ADIDigitalIn Catataaa;

#endif

#ifndef CONSTS
#define CONSTS

#define getDR() (double)R.get_value()
#define getDL() (double)L.get_value()
#define PI 3.14159265358979323846

struct Slew_t{
    double SlewRate, output;
    int prevTime;
    Slew_t(float SlewRate=105);
    double update(double in);
};

#endif