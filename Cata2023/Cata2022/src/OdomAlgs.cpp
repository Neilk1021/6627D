#include "consts.hpp"
#include "main.h"
#include <cmath>


Slew_t::Slew_t(float SlewRate){
  this->SlewRate=SlewRate; output = 0;
  prevTime = millis();
};

//changes drive voltage over time rather than immediately to reduce strain and variation
double Slew_t::update(double in){
  int dt = millis() - prevTime;
  if(dt > 1000) dt = 0;
  prevTime = millis();
  double maxIncrease = SlewRate * dt;
  double outputRate = (double)(in - output) / (double)dt;
  if (fabs(outputRate) < SlewRate) output = in;
  else if (outputRate > 0) output += maxIncrease;
  else output -= maxIncrease;
  return output;
}


int signOf(double x) {
  return (x < 0 ? -1: 1);
}

void reduceDirec(double& direc){
  while(direc >= 360){
    direc -= 360;
  }

  while(direc < 0){
    direc += 360;
  }
}

void toShortestAngle(double& theta){
  if(std::abs(theta-info.direc) >  180){
    theta -= signOf(theta-info.direc) * 360;
  }
}

void clamp(double& val, double lower, double upper){
  if(val > upper){
    val = upper;
  }else if (val < lower) {
    val = lower; 
  }
}

void DriveToPoint(double dis, double spd, double Heading, int minClamp, bool disableNotMoveCheck ,bool hasFunc, void DistanceFunc(), float distanceToFireFunc) {
  double startTicks = info.encTicks;
  double motorScale;
  double deltaO;
  double targetCount = 0;

  double Kp = 550;
  double Ki = 0;
  double Kd = 10000;

  double RightScale;
  double LeftScale;
  double correctionAmount = 4;

  double Integral;
  double derivative;
  double prevDeltaO;
  Slew_t slewL(50);
  Slew_t slewR(50);
  //function to run while driving 
  void (*RunFunc)() = DistanceFunc;
  bool Fired = false;
  do{
    deltaO = info.encTicks - startTicks + dis;
    Integral = Integral + deltaO;

    if(Integral > 100)
      Integral = 100;

    derivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;
    //pid motor voltage calculation
    motorScale = (deltaO * Kp) + (Integral* -Ki) + (derivative*Kd);
    if(hasFunc && !Fired && std::abs(deltaO) < distanceToFireFunc) {
      DistanceFunc();
      Fired = true;
    }
    //scales motor power based off of heading of the robot.
      RightScale = 1 + sin((info.direc - Heading)*PI/180)*correctionAmount*signOf(motorScale);
      LeftScale = 1 - sin((info.direc - Heading)*PI/180)*correctionAmount*signOf(motorScale);
    // RightScale = 1;
    // LeftScale = 1;

    int SaveDir = signOf(motorScale);
    motorScale = abs( motorScale);
    clamp(motorScale, minClamp, infinity());

    if(std::abs(deltaO) < 0.15 ){
      motorScale = 0;
      targetCount++;
    }
    else if (derivative == 0 && !disableNotMoveCheck) targetCount += .45;
    else targetCount =0;

    double pwrR = slewR.update(spd*motorScale*SaveDir)*RightScale;
    double pwrL = slewL.update(spd*motorScale*SaveDir)*LeftScale;
    printf("RightScale: %f\n", RightScale);


    RightDrive.move_voltage(pwrR);
    LeftDrive.move_voltage(-pwrL);
    pros::delay(10);

  }while(targetCount < 10);

  RightDrive.brake();

  LeftDrive.brake();

}

void turnDeg(double degrees, double spd){
  double tempAngle = info.direc;
  double angleRad = degrees;
  double target; //total amsount to change
  double motorScale;
  double deltaO;
  double targetCount = 0;

  double Kp = 250;
  double Ki = 0;
  double Kd = 290;

  double Integral;
  double derivative;
  double prevDeltaO;
  double Error ;
  //reduces direction of the current heading to within 360 degrees
  target =angleRad; 
  //toShortestAngle(target);
  Slew_t slew;

  while(targetCount < 6){
    
    tempAngle = info.direc;
    deltaO = target + info.direc;
    Error = std::abs(deltaO);
    Integral = Integral + (cbrt(Error)*signOf(deltaO)*16);

    if(Integral > 1900)
      Integral = 1900;
    else if (Integral < -1900) Integral = -1900;
    derivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;
    //Combines P I D values to a draft voltage to be sent to the slew object
    motorScale = (deltaO * Kp) + (Integral * Ki) + (derivative*Kd);
    //if the degree is within a certain range the code stops.
    if(std::abs(deltaO) < 0.25){
      Integral = 0;
      motorScale = 0;
      targetCount++;
    }
    else if (std::abs(derivative) < 0.05) targetCount += 0.1125;
    else targetCount =0;

    //Calcuates power to apply to motors based on current voltage.
    double pwr = slew.update(spd*motorScale);
    RightDrive.move_voltage(pwr);
    LeftDrive.move_voltage(pwr);
    //Debug to keep track of PID
    //printf("P: %d I: %d D: %d Error: %f\n", (int)(deltaO*Kp), (int) (Integral * Ki), (int)(derivative*Kd), deltaO);

    delay(10);
  }

  RightDrive.brake();
  LeftDrive.brake();
}

