#include "datastructs.h"
#include "main.h"
#include "pros/adi.hpp"
using namespace pros;

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

extern pros::Motor LeftDriveMotor1;
extern pros::Motor LeftDriveMotor2;

extern pros::Motor RightDriveMotor1;
extern pros::ADIEncoder L;
extern pros::Motor Flywheel;
extern pros::Motor Flywheel2;
extern pros::ADIEncoder R;
extern pros::Motor RightDriveMotor2;
extern pros::Motor IntakeM;
extern pros::Motor IntakeM2;
extern pros::ADIEncoder FlywheelEnc;

//extern pros::motor_group LeftDriveTrain;
//extern pros::motor_group RightDriveTrain;
extern pros::ADIDigitalOut Indexer;
extern pros::ADIDigitalOut Cat;

#endif