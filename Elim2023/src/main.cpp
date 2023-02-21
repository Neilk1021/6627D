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
  pros::lcd::initialize();
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

void driveAuton(){
    //Turns on flywheel and drives then turns 90 degrees
  SwitcherFunc(Auton);
  DriveToPoint(2.3, 1.1, info.direc);
  pros::delay(50);
  turnDeg(-90, 0.67);
  //drives into and backs out of roller
  DriveToPoint(0.8, 0.9, info.direc);
  SwitchIntake();
  pros::delay(400);
  SwitchIntake();
  DriveToPoint(-0.4, 1 , info.direc, 2100, true);
  turnDeg(-100.25, 0.80);
  switchPiston();
  pros::delay(800);
  switchPiston();
  pros::delay(250);
   SwitcherFunc(AutonSlow);
  turnDeg(138, 0.55);
    info.resetDirecc = true;
  pros::delay(5);
  SwitchIntakeBackSlow();
  DriveToPoint(6.75, 0.9, info.direc, 2700);
  turnDeg(90, 0.67);
  switchPiston();
  SwitchIntake();
  pros::delay(800);
  switchPiston();
  pros::delay(800);
  switchPiston();

}
void SkillAuton(){
  SwitcherFunc(Normal);
  DriveToPoint(2.3, 1, 0);
  pros::delay(50);
  turnDeg(-90, 0.67);
  //drives into and backs out of roller
  DriveToPoint(0.8, 0.9, info.direc);
  SwitchIntake();
  pros::delay(800);
  SwitchIntake();
  DriveToPoint(-2.2, 1.7, info.direc, 1100);
  turnDeg(0, 0.6);
  DriveToPoint(3.5, 1, 0);
  SwitchIntake();
  pros::delay(800);
  SwitchIntake();
  DriveToPoint(-0.5, 1.7, info.direc, 1100);
  turnDeg(90, 0.6);
  DriveToPoint(5.5, 0.8, info.direc);
  turnDeg(-86, 0.6);
  switchPiston();
  pros::delay(500);
  switchPiston();
  pros::delay(500);
  switchPiston();

}

void autonomous(){
  driveAuton();
} 

void ArcadeTwoStick()
  {
  
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

  float AxisOne = Con.get_analog(ANALOG_RIGHT_X)*1.25;
  float AxisThree = Con.get_analog(ANALOG_LEFT_Y)*2;

    if(std::abs(AxisThree) > 2 || std::abs(AxisOne) > 2) {
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
