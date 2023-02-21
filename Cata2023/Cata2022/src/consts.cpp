#include "consts.hpp"
#include "pros/motors.hpp"
#define L_TOP_PORT 'A'
#define L_BOTTOM_PORT 'B'
#define R_TOP_PORT 'C'
#define R_BOTTOM_PORT 'D'

CoreData info;

pros::Controller Con(CONTROLLER_MASTER);

const double ENC_L_DIST = 6.40625;
const double ENC_R_DIST = 6.40625;
const double WHEEL_RAD = 3.25/2;
//325
const float ANGLE_SCALE_VAL =  49.5 * 1.5;
const double AutonSpeed =  166;
const double IntakeSpeed = 420;
const double IntakeSpeedSlowed = 300;
//encoder encL('C', 'D', true);
//encoder encR('E', 'F');  

Motor LeftDriveMotor1(18, E_MOTOR_GEAR_BLUE, false);
Motor LeftDriveMotor2(19, E_MOTOR_GEAR_BLUE, false);
Motor LeftDriveMotor3(20, E_MOTOR_GEAR_BLUE, false);
Motor RightDriveMotor1(11, E_MOTOR_GEAR_BLUE, false);
Motor RightDriveMotor2(15, E_MOTOR_GEAR_BLUE, false);
Motor RightDriveMotor3(16, E_MOTOR_GEAR_BLUE, false);

Motor IntakeM(1 , E_MOTOR_GEARSET_18, true);
Motor CataM(14, E_MOTOR_GEAR_RED, false);

Motor_Group LeftDrive({LeftDriveMotor1, LeftDriveMotor2, LeftDriveMotor3});
Motor_Group RightDrive({RightDriveMotor1, RightDriveMotor2, RightDriveMotor3});

Imu InertialSensor(17);
ADIEncoder Encoder({L_TOP_PORT, L_BOTTOM_PORT});
ADIDigitalOut Cat('D');
ADIDigitalIn Catataaa('E');
//motor_group LeftDriveTrain(LeftDriveMotor1, LeftDriveMotor2);
//motor_group RightDriveTrain(RightDriveMotor1, RightDriveMotor2);
