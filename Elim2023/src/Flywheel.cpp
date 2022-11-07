#include "main.h"
#include "consts.hpp"


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
      info.desiredFlywheelSpeed = 152;
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

  
int flywheelWheelPID(){
  //Constants//
float kp = .057;
float ki = 0;
float kd = .11;

//PID Variables Here//
double currentVelocity = Flywheel2.get_actual_velocity();
double error = info.desiredFlywheelSpeed - currentVelocity;
int lastError = 0;
int totalError = 0;
int integralActiveZone = 15;

float finalAdjustment = error * kp; //add the rest of PID to this calculation
double currentFlywheelVoltage= 0;

while (true)
{
		currentVelocity = Flywheel2.get_actual_velocity();

		error = info.desiredFlywheelSpeed - currentVelocity;

		if (std::abs(error) < integralActiveZone && error != 0)
			totalError += error;
		else
			totalError = 0;

		finalAdjustment = ((error * kp) + (totalError * ki) + ((error - lastError) * kd));
  
    if (std::abs(currentVelocity/info.desiredFlywheelSpeed) < .85) 
       finalAdjustment = 127;

		currentFlywheelVoltage += finalAdjustment;
    clamp(currentFlywheelVoltage, -127, 127);

  	Flywheel.move(currentFlywheelVoltage);
		Flywheel2.move(currentFlywheelVoltage);
    lastError = error;
    c::delay(10);
}
}