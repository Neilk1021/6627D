#include "main.h"
#include "consts.h"
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
    SwitcherFunc(Auton);
  DriveToPoint(2, 1, info.direc);
  pros::delay(50);
  turnDeg(-90, 1, 1500);
  DriveToPoint(0.375, 1.5, info.direc);
    SwitchIntake();
  pros::delay(350);
  SwitchIntake();
  DriveToPoint(-0.425, 2, info.direc);
  turnDeg(-102.1, 1, 1500);
    pros::delay(150);
  switchPiston();
  pros::delay(800);
  switchPiston();
    pros::delay(200);
  SwitcherFunc(Auton);
  turnDeg(137.5, 0.8, 1900);
  SwitchIntakeBackSlow();
  pros::delay(100);
  DriveToPoint(6.4, .62, info.direc, 3300);
    pros::delay(200);
      SwitcherFunc(AutonSlow);
          pros::delay(100);
          info.resetDirecc = true;
  pros::delay(5);
  turnDeg(85.25, 1, 1550);
    DriveToPoint(-0.2, 1.6, info.direc);
      pros::delay(1400);
  switchPiston();
  SwitchIntake();
  pros::delay(600);
  switchPiston();
    pros::delay(600);
  switchPiston();
    SwitcherFunc(AutonSlow);
  
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
      LeftDriveMotor1.move_velocity(-(AxisThree + AxisOne));
      LeftDriveMotor2.move_velocity(-(AxisThree + AxisOne));
      RightDriveMotor1.move_velocity((AxisThree - AxisOne));
      RightDriveMotor2.move_velocity((AxisThree - AxisOne));

    }else{
        RightDriveMotor1.brake();
        RightDriveMotor2.brake();

        LeftDriveMotor1.brake();
        LeftDriveMotor2.brake();
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
