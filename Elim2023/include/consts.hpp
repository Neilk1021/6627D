#include "datastructs.h"
#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
using namespace pros;
using namespace pros::c;
using namespace std; 

#ifndef DATA_H
#define DATA_H

extern struct CoreData info;
extern pros::Controller Con;

extern ADIEncoder encL;
extern ADIEncoder encR;

extern const double ENC_L_DIST;
extern const double ENC_R_DIST;
extern const double WHEEL_RAD;
extern const double FlywheelSpeed;
extern const float ANGLE_SCALE_VAL;
extern const double AutonSpeed;
extern const double IntakeSpeed;
extern const double IntakeSpeedSlowed;


//extern pros::Motor RightDriveMotor1;
extern ADIEncoder L;
extern pros::Motor Flywheel;
extern pros::Motor Flywheel2;
extern ADIEncoder R;
//extern pros::Motor RightDriveMotor2;
extern pros::Motor IntakeM;
extern pros::Motor IntakeM2;
extern pros::ADIEncoder FlywheelEnc;

//extern pros::motor_group LeftDriveTrain;
//extern pros::motor_group RightDriveTrain;
extern pros::ADIDigitalOut Indexer;
extern pros::ADIDigitalOut Cat;
extern Motor_Group RightDrive;
extern Motor_Group LeftDrive;

#endif

#define getDR() (double)R.get_value()
#define getDL() (double)L.get_value()
#define PI 3.14159265358979323846
#define ToRad() 180/PI