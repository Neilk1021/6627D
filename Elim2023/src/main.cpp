#include "main.h"
#include "consts.hpp"
#include <cmath>

bool RunningPiston = false;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void RunPistonIndex(){
  RunningPiston = true;
  Indexer.set_value(false);
  pros::delay(125);
  Indexer.set_value(true);
  RunningPiston = false;
}

void initialize() {
  	proscodeInit();
}
void disabled() {}

void competition_initialize() {
  proscodeInit();
}


inline void switchPiston(){
  if(RunningPiston == false)
  Task RunPiston(RunPistonIndex);
}

void Ct(){
  Cat.set_value(true);
}

void AutonTemp(){
  DriveFor(26, 70, 0, true);
    SwitcherFunc(AutonFirst);
  pros::delay(200);
  R.reset();
  L.reset();
  turnDeg(-90, 1.4);
  DriveFor(2.5, 75, 0, true);
  SwitchIntake();
  pros::delay(300);
  SwitchIntake();
  DriveFor(-1.5, 70, 0, true);
  pros::delay(200);
  turnDeg(-98.75, 1.4);
  pros::delay(1050);
  switchPiston();
  SwitcherFunc(Auton);
  pros::delay(550);
  switchPiston();
  pros::delay(550);
  SwitcherFunc(Auton);
}

void disk(){
    DriveTo(1.4, 1, nullptr, 0, false);
    pros::delay(100);
  R.reset();
  L.reset();
  turnDeg(-90, 1.4);
    DriveFor(2.5, 75, 0, true);
  SwitchIntake();
  pros::delay(300);
  SwitchIntake();
  DriveFor(-1.5, 70, 0, true);
  turnDeg(140, 1.4);
  SwitchIntake();
  pros::delay(50);
  DriveTo(3.8, 1, nullptr, 0, false);
  SwitcherFunc(Auton);
  pros::delay(50);
  turnDeg(-135, 1.4);
  switchPiston();
  pros::delay(550);
  switchPiston();
  pros::delay(550);
    SwitcherFunc(Auton);
}

void WorkingAuton(){
   DriveTo(1.2, 1, nullptr, 0, false);
    pros::delay(100);
  R.reset();
  L.reset();
  turnDeg(-90, 1.4);
    DriveFor(2.6, 75, 0, true);
  SwitchIntake();
  pros::delay(400);
  SwitchIntake();
  DriveFor(-1.5, 70, 0, true);
  turnDeg(144, 1.4);
  SwitchIntakeBack();
  pros::delay(50);
  DriveTo(7.5, 1, SwitchIntake, 1.3, true);
  DriveFor(-2.375, 75, 0, true);
  SwitcherFunc(AutonFirst);
  turnDeg(180, 1.4);
  DriveFor(2.5, 75, 0, true);
  pros::delay(300); 
  SwitchIntake();
  DriveFor(-1.5, 75, 0, true);
    turnDeg(180, 1.4);
    R.reset();
    L.reset();
        turnDeg(4, 1.4);
            switchPiston();
  SwitcherFunc(Auton);
  pros::delay(550);
  switchPiston();
  pros::delay(550);
  switchPiston();
  pros::delay(550);
  SwitcherFunc(AutonFirst);
}

void WPProgress(){
  DriveTo(1.2, 1.1, nullptr, 0, false);
  R.reset();
  L.reset();
  turnDeg(-90, 1.5);
    DriveFor(2.7, 170, 0, true);
  SwitchIntake();
  pros::delay(400);
  SwitchIntake();
  DriveFor(-1.5, 100, 0, true);
  turnDeg(136, 1.425);
  SwitchIntakeBack();
  pros::delay(100);
  DriveTo(7.25, 1.1, SwitchIntake, 1.5, true);
  DriveFor(-1.5, 100, 0, true);
  SwitcherFunc(AutonFirst);
  turnDeg(162, 1.4);
  DriveFor(2.5, 150, 0, true);
  pros::delay(300); 
  SwitchIntake();
  DriveFor(-1.5, 150, 0, true);
    turnDeg(175, 1.7);
            switchPiston();
  SwitcherFunc(Auton);
  pros::delay(550);
  switchPiston();
  pros::delay(550);
  switchPiston();
  pros::delay(550);
  SwitcherFunc(AutonFirst);
}

void autonomous(){
  //Turns on flywheel and drives then turns 90 degrees
  SwitcherFunc(Auton);
  DriveToPoint(2.25, .75, info.direc);
  pros::delay(50);
  turnDeg(-90, 0.75, 2500);
  //drives into and backs out of roller
  DriveToPoint(0.8, 0.9, info.direc);
  SwitchIntake();
  pros::delay(500);
  SwitchIntake();
  DriveToPoint(-0.4, 1.5, info.direc, 1300);
  turnDeg(-101.5, 1.1, 1900);
  pros::delay(100);
  switchPiston();
  pros::delay(800);
  switchPiston();
  pros::delay(200);
    SwitcherFunc(AutonSlow);
  turnDeg(137.5, 0.75, 2500);
  SwitchIntakeBackSlow();
  pros::delay(100);
  DriveToPoint(6.95, .75, info.direc, 2700);
  pros::delay(100);
  info.resetDirecc = true;
  pros::delay(5);
  turnDeg(90.4, 0.8, 2300);
  switchPiston();
  SwitchIntake();
  pros::delay(500);
  switchPiston();
  pros::delay(600);
  switchPiston();
  
} 

void ArcadeTwoStick()
  {
  
  float AxisOne = Con.get_analog(ANALOG_RIGHT_X)*1.25;
  float AxisThree = Con.get_analog(ANALOG_LEFT_Y)*2;

  float *ptrA1 = &AxisOne;
  float *ptrA3 = &AxisThree;

  while(true){
     
  if(Con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP))
    SwitcherFunc(Normal);
  if(Con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    SwitcherFunc(Slow);
  if(Con.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT))
    switchPiston();
  if(Con.get_digital_new_press(E_CONTROLLER_DIGITAL_R1))
    SwitchIntake();
  if(Con.get_digital_new_press(E_CONTROLLER_DIGITAL_R2))
    SwitchIntakeBack();
  if(Con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    Ct();
  if(Con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    SwitchIntakeSpeed();

  *ptrA1 = Con.get_analog(ANALOG_RIGHT_X)*1.25;
  *ptrA3 = Con.get_analog(ANALOG_LEFT_Y)*2;

    if(std::abs(*ptrA3) > 2 || std::abs(*ptrA1) > 2) {
      LeftDrive.move_velocity(-(AxisThree + AxisOne));
      RightDrive.move_velocity((AxisThree - AxisOne));

    }else{
        RightDrive.brake();
        LeftDrive.brake();
    }
    pros::delay(10);

  } 

}

void opcontrol() {
	initialize();
  //Gif gif("s/usd/trollface.gif", lv_scr_act())
  ArcadeTwoStick();

  delay(20);
}
