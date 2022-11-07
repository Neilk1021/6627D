#include "../include/consts.hpp"
#include "../include/main.h"
#include "pros/adi.h"
#include <string>
using namespace pros;

// A global instance of brain used for printing to the V5 brain screen

/**
 * Used to initialize code/tasks/devices added using tools in proscode Pro.
 *
 * This should be called at the start of your int main function.
 */
// void proscodeInit(void) {

//   RightDriveMotor1.setPosition(0, rotationUnits::deg);
//   RightDriveMotor2.setPosition(0, rotationUnits::deg);
//   LeftDriveMotor1.setPosition(0, rotationUnits::deg);
//   LeftDriveMotor2.setPosition(0, rotationUnits::deg);
//   RightDriveTrain.setPosition(0, rotationUnits::deg);
//   LeftDriveTrain.setPosition(0, rotationUnits::deg);
//   R.resetRotation();
//   L.resetRotation();
//   pros::task runE(eFunc);
//   //encL.resetRotation();
//   //encR.resetRotation();
//   info.driveticks = 0;
//   info.direc = 0;
//   info.resetVal = 0;
//   info.resetDirecc = true;
//   pros::task runTracking(OdomTrack);
// }

void proscodeInit(){   
    Indexer.set_value(true);
    R.reset();
    L.reset();
    RightDrive.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
    LeftDrive.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
    info.driveticks = 0;
    Cat.set_value(false);
    info.direc = 0;
    info.resetVal = 0;
    info.desiredFlywheelSpeed = 0;
    Task Run(Intake);
    Task runFlyWheel(flywheelWheelPID);
    Task runTracking(task_tracking);
}

// void competitions(){
//      info.AutonNumber = 0;
//  bool AutonSelected = false;
//  while(!AutonSelected){
//     if(Con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
//         info.AutonNumber --;
//     } 
//     else if(Con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//         info.AutonNumber ++;
//     }else if(Con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) AutonSelected = true;

//     lcd::set_text(0, "Auton Selected" + std::to_string( info.AutonNumber));
//     c::delay(50);
//  }
// }


/*
}

extern pros::motor LeftDriveMotor1(pros::PORT15, ratio18_1, true);
extern pros::motor LeftDriveMotor2(pros::PORT14, ratio18_1, true);

extern pros::motor RightDriveMotor1(pros::PORT7, ratio18_1, true);
extern pros::motor RightDriveMotor2(pros::PORT6, ratio18_1, true);

extern pros::motor leftLift(pros::PORT11, ratio36_1, false);
extern pros::motor rightLift(pros::PORT12, ratio36_1, false);

extern pros::motor leftlift2(pros::PORT18, ratio18_1, true);
extern pros::motor rightlift2(pros::PORT13, ratio18_1, false);

extern pros::motor_group LeftDriveTrain(LeftDriveMotor1, LeftDriveMotor2);
extern pros::motor_group RightDriveTrain(RightDriveMotor1, RightDriveMotor2);
extern pros::motor_group FourBar(leftLift, rightLift);
extern pros::motor_group LiftBack(leftlift2, rightlift2);
extern pros::motor fwbw(pros::PORT9, ratio18_1, false);
*/