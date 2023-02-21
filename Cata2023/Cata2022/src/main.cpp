#include "main.h"
#include "consts.hpp"
#include <vector>
#include "Utils.h"


void InitializeCata(){
	
		while (Catataaa.get_value() == 0){
			CataM.move(-127);
		}
		CataM.brake();
}

void initialize() {
	InertialSensor.reset();
    RightDrive.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
    LeftDrive.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
	CataM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		Cat.set_value(true);
    Task Run(Intake);
	Task Track(Tracker);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() 
{
	// InertialSensor.reset();
    // RightDrive.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
    // LeftDrive.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
	// CataM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // Task Run(Intake);
	// Task Track(Tracker);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
bool firing = false;

float DelayMax = 0;

void Deactivate(){
	pros::delay(DelayMax);
	firing = true;
	while (pros::c::adi_digital_read('d') == 1) {
		CataM.move(-127);
	}
	CataM.brake();
	firing = false;
}

void FireCata(){
	pros::delay(DelayMax);
	if(firing) return;
		firing = true;
		while (Catataaa.get_value() == 1) {
			CataM.move(-127);
		}

		CataM.brake();
		pros::delay(300);

		while (Catataaa.get_value() == 0){
			CataM.move(-127);
		}
		CataM.brake();
		firing = false;
};

	void LaunchEndgame(){
		Cat.set_value(false);
	}


void Skills(){
	while (InertialSensor.is_calibrating()) continue;
		SwitchIntakeBack();
	DriveToPoint(6, 2, 0);
	pros::delay(450);
	SwitchIntakeBack();
		DriveToPoint(-0.9, 1);
	turnDeg(-135, 0.8);
	SwitchIntake();
	DriveToPoint(10, 1, info.direc);
	SwitchIntake();
	turnDeg(-90, 1);
	DriveToPoint(10, 1, info.direc);
		SwitchIntakeBack();
	pros::delay(400);
	SwitchIntakeBack();
	DriveToPoint(-0.5, 1, info.direc);
	turnDeg(0, 0.9);
	DriveToPoint(-31, 1, info.direc);
	turnDeg(-12, 0.75);
	Task e(FireCata);
	pros::delay(500);
	turnDeg(0, 0.75);
	DriveToPoint(31, 1, info.direc);
	turnDeg(-45, 0.75);
	LaunchEndgame();
// 	turnDeg(0, 0.75);
// 	DriveToPoint(4, 1, 0);
// 	turnDeg(90, 0.8);
// 	SwitchIntake();
// 	DriveToPoint(15, 0.75, info.direc, 3400);
// 	DriveToPoint(-15.5, 1, info.direc);
// 	turnDeg(-12, 0.75);
// 		SwitchIntake();
// 	Task ee(FireCata);
// }
}
void Match(){
	while (InertialSensor.is_calibrating()) continue;
	DriveToPoint(-6, 2, 0);
	SwitchIntake();
	pros::delay(200);
	 SwitchIntake();
	// DriveToPoint(-3, 1.1, 0);
	// turnDeg(-45, 0.75);
	// //DelayMax = 250;
	// DriveToPoint(-31, 1, info.direc);
	// turnDeg(45, 0.9);
	// //Task ee(FireCata);
	// DriveToPoint(-3.5, 1, info.direc);
	// SwitchIntake();
	// pros::delay(1400);
	//Task eee(FireCata);
	//DriveToPoint(-3, 1.2, info.direc);
	// SwitchIntake();
	// DriveToPoint(33.5, 0.9, info.direc, 1700);
	// SwitchIntake();
	// turnDeg(90,0.75);
	// DriveToPoint(10, 2, info.direc);
	// SwitchIntakeBack();
	// pros::delay(250);
	// SwitchIntakeBack();
}

void DRIVEFOR(){
	DriveToPoint(-24, 1, info.direc);
	turnDeg(-45, .8);
	Task ee(FireCata);
		pros::delay(1500);
	SwitchIntake();
	pros::delay(1400);
	Task eee(FireCata);
	turnDeg(44, .8);
	SwitchIntake();
	DriveToPoint(33.5, 0.9, info.direc, 1700);
	turnDeg(0, .9);
	
}
void autonomous() {
	Match();
}

void opcontrol() {
	Utils::Button LaunchEg(LaunchEndgame, Con, E_CONTROLLER_DIGITAL_UP);
	Utils::Button SwitchIntakeFB(SwitchIntake, Con, E_CONTROLLER_DIGITAL_R2);
	Utils::Button SwitchIntakeBB(SwitchIntakeBack, Con, E_CONTROLLER_DIGITAL_R1);
	Utils::Button FireCataB(FireCata, Con, E_CONTROLLER_DIGITAL_B);

	vector<Utils::Button> ControllerButtons = {
		LaunchEg, SwitchIntakeFB, SwitchIntakeBB, FireCataB
	};


	Utils::Controller BaseController (ControllerButtons);
}
