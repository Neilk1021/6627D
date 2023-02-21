#include "main.h"
#include "consts.hpp"
#include <stdio.h>

Running running;
double TargetSpeed = 0; 

void UpdateSpeed(){
  switch (running) {
    case  Stop:
      info.desiredFlywheelSpeed = 0;
      break;
    case Slow:
      info.desiredFlywheelSpeed = 154;
      break;
    case Normal:
      info.desiredFlywheelSpeed = FlywheelSpeed;
      break;
    case Auton:
      info.desiredFlywheelSpeed = AutonSpeed;
      break;
    case AutonSlow:
      info.desiredFlywheelSpeed = 153;
      break;
    case AutonFirst: 
      //4
      info.desiredFlywheelSpeed = FlywheelSpeed+3;
      break;
    case AutonSlowFirst: 
      //4
      info.desiredFlywheelSpeed = AutonSpeed/2+8 ;
      break;
  }
}

void SwitcherFunc(Running run){
 running == run ? running = Stop : running = run;
 UpdateSpeed();
}

 /**
 Flywheel PID loop to maintain constant rpm of the flywheel. 
 */ 
int flywheelWheelPID(){
  //Constants//
float kp = 0.05;
float ki = 0;
//11
float kd = 0.8;
//PID Variables Here//
double currentVelocity = Flywheel2.get_actual_velocity();
double error = info.desiredFlywheelSpeed - currentVelocity;
int lastError = 0;
int totalError = 0;
int integralActiveZone = 15;

float finalAdjustment = error * kp; //add the rest of PID to this calculation
double currentFlywheelVoltage= 0;
//Slew obj to handle voltage change
Slew_t slew(100);

while (true)
{
		currentVelocity = Flywheel2.get_actual_velocity();

		error = info.desiredFlywheelSpeed - currentVelocity;

		if (std::abs(error) < integralActiveZone && error != 0)
			totalError += error;
		else
			totalError = 0;

    //PID voltage
		finalAdjustment = ((error * kp) + (totalError * ki) + ((error - lastError) * kd));
  
    // if (std::abs(currentVelocity/info.desiredFlywheelSpeed) < .2) 
    //    finalAdjustment = 127;

		currentFlywheelVoltage += finalAdjustment;
    clamp(currentFlywheelVoltage, -127, 127);
    //Update to voltage over time

    printf("%f\n",  currentVelocity);
    // printf("P: %d I: %d D: %d Error: %f\n", (int)(error*kp), (int) (totalError * ki), (int)((error-lastError)*kp), currentVelocity);
    double pwr = slew.update(currentFlywheelVoltage);

  	Flywheel.move(pwr);
		Flywheel2.move(pwr);
    lastError = error;
    pros::delay(50);
}
}