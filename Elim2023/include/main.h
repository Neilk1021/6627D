/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif

enum Running {
  Stop,
  Slow,
  Normal,
  Auton,
  AutonFirst,
  AutonSlow,
  AutonSlowFirst
};

void proscodeInit();
int signOf(double);
void reduceDirec(double&);
void toShortestAngle(double&);
void toGearRotation(int&);
void turnTo(double, double);
void turnDeg(double, double, int = 1900);
//int eFunc();
int flywheelWheelPID();
void clamp(double&, double, double);
void switchIntake();
void SwitchIntakeBack();
void DriveFor(double, double, double, bool);
void DriveTo(double, double, void() = nullptr, float = 0, bool = false);
void DriveToPoint(double, double, double = 0, int = 1800,  bool = false, void() = nullptr, float = 0);
int task_tracking();
int OdomTrack();
void Intake();

//Flywheel
void SwitcherFunc(Running);
//Intake
void SwitchIntakeSpeed();
void SwitchIntake();
void SwitchIntakeBack();
void SwitchIntakeBackSlow();


void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol();
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
