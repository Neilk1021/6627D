#include "consts.h"

CoreData info;

pros::Controller Con(CONTROLLER_MASTER);

const double ENC_L_DIST = 6.40625;
const double ENC_R_DIST = 6.40625;
const double WHEEL_RAD = 3.25/2;
//325
const double FlywheelSpeed = 145;
const float ANGLE_SCALE_VAL =  49.5 * 1.5;
const double AutonSpeed =  166;
const double IntakeSpeed = 420;
const double IntakeSpeedSlowed = 300;

//encoder encL('C', 'D', true);
//encoder encR('E', 'F');  

Motor LeftDriveMotor1(11, E_MOTOR_GEARSET_18, false);
Motor LeftDriveMotor2(12, E_MOTOR_GEARSET_18, false);
Motor RightDriveMotor1(13, E_MOTOR_GEARSET_18, false);
Motor RightDriveMotor2(14, E_MOTOR_GEARSET_18, false);
Motor Flywheel(6, E_MOTOR_GEARSET_18, false);
Motor Flywheel2(8, E_MOTOR_GEARSET_18, true);
Motor IntakeM(9 , E_MOTOR_GEARSET_06, true);
Motor IntakeM2(7 , E_MOTOR_GEARSET_06, false);

//motor_group LeftDriveTrain(LeftDriveMotor1, LeftDriveMotor2);
//motor_group RightDriveTrain(RightDriveMotor1, RightDriveMotor2);
#define L_TOP_PORT 'A'
#define L_BOTTOM_PORT 'B'
#define R_TOP_PORT 'C'
#define R_BOTTOM_PORT 'D'

 ADIEncoder L(L_TOP_PORT, L_BOTTOM_PORT); 
ADIEncoder R(R_TOP_PORT, R_BOTTOM_PORT); 
ADIEncoder FlywheelEnc('E', 'F'); 
ADIDigitalOut Indexer('G');
ADIDigitalOut Cat('H');