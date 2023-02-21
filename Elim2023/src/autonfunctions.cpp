#include "consts.hpp"
#include <atomic>
#include <cmath>
#include <type_traits>

Slew_t::Slew_t(float SlewRate){
  this->SlewRate=SlewRate; output = 0;
  prevTime = millis();
};

//changes flywheel voltage over time rather than immediately to reduce strain and variation
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

void DriveFor(double dis, double spd, double head, bool slowOn) {
  // negative spd = drive backwards
  double encTicks = dis;
  double startTicks = info.driveticks;
  //double angleRad = head*(3.1415962/180.0);
  //double currentDirec;
  double absDif;
  double deltaO;
  double motorScale = 1;
  double rightScale = 1;
  double leftScale = 1;
  double tempDist = dis;

  if ( dis > 10 ) {
    tempDist = 10;
  }

  do {
    absDif = std::abs(info.driveticks - startTicks);
    //printf("%*.*f\n", 5, 4, info.driveticks);
    if (slowOn) {
      motorScale = std::abs(absDif - encTicks)*WHEEL_RAD/(tempDist/30); //scales within a third of the driving distance or 10 inches
      motorScale = ( motorScale >= 1 ? 1 : motorScale );
      clamp(motorScale, .2, 1);
    } else {
      motorScale = 1;
    }
      rightScale = 1;
      leftScale = 1;
    //}

    RightDrive.move_velocity(spd*motorScale*rightScale*signOf(dis));

    LeftDrive.move_velocity(-spd*motorScale*leftScale*signOf(dis));

    //RightDriveTrain.spin(directionType::rev, spd*motorScale*rightScale, pct);
    //LeftDriveTrain.spin(directionType::fwd, spd*motorScale*leftScale, pct);

    pros::delay(5);

  } while (absDif < std::abs(encTicks));

  RightDrive.brake();

  LeftDrive.brake();


 // RightDriveTrain.brake(brakeType::brake);
  //LeftDriveTrain.stop(brakeType::brake);
}

void DriveTo(double dis, double spd, void DistanceFunc(), float distanceToFireFunc, bool hasFunc) {
  double encTicks = dis* 19;
  double startTicks = info.driveticks;
  double absDif;
  double motorScale;
  double deltaO;
  int targetCount = 0;

  double Kp = 7.2;
  double Ki = 0;
  double Kd = 5;

  double Integral;
  double derivative;
  double prevDeltaO;
  
  void (*RunFunc)() = DistanceFunc;
  bool Fired = false;
  do{
    absDif = info.driveticks - startTicks;
    deltaO = info.driveticks - startTicks +encTicks;
    Integral = Integral + deltaO;

    if(Integral > 100)
      Integral = 100;

    derivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;

    motorScale = (deltaO * Kp) + (Integral* -Ki) + (derivative*Kd);
    if(hasFunc && !Fired && std::abs(deltaO) < distanceToFireFunc*19) {
      DistanceFunc();
      Fired = true;
    }

    if(std::abs(deltaO) < 1){
      motorScale = 0;
      targetCount++;
    }else targetCount =0;

    RightDrive.move_velocity(spd*motorScale);

    LeftDrive.move_velocity(-spd*motorScale);
    pros::delay(5);
  }while(targetCount < 8);

  RightDrive.brake();

  LeftDrive.brake();

}
/**
drives x distance at a certain speed and attempts to maintain a certain heading while driving
*/
void DriveToPoint(double dis, double spd, double Heading, int minClamp, bool disableNotMoveCheck ,bool hasFunc, void DistanceFunc(), float distanceToFireFunc) {
  double encTicks = dis* 19;
  double startTicks = info.driveticks;
  double motorScale;
  double deltaO;
  double targetCount = 0;

  double Kp = 630;
  double Ki = 0;
  double Kd = 11000;

  double RightScale;
  double LeftScale;
  double correctionAmount = .05;

  double Integral;
  double derivative;
  double prevDeltaO;
  Slew_t slewL(75);
  Slew_t slewR(75);
  //function to run while driving 
  void (*RunFunc)() = DistanceFunc;
  bool Fired = false;
  do{
    deltaO = info.driveticks - startTicks +encTicks;
    Integral = Integral + deltaO;

    if(Integral > 100)
      Integral = 100;

    derivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;
    //pid motor voltage calculation
    motorScale = (deltaO * Kp) + (Integral* -Ki) + (derivative*Kd);
    if(hasFunc && !Fired && std::abs(deltaO) < distanceToFireFunc*19) {
      DistanceFunc();
      Fired = true;
    }
    //scales motor power based off of heading of the robot.
    if(fabs(info.direc - Heading) > 0.05)  {
      RightScale = 1 + (info.direc - Heading)*correctionAmount*signOf(motorScale);
      LeftScale = 1 - (info.direc - Heading)*correctionAmount*signOf(motorScale);
    }else {
      RightScale = 1;
      LeftScale = 1;
    }

    int SaveDir = signOf(motorScale);
    clamp(motorScale, minClamp, 10000000);

    if(std::abs(deltaO) < 0.5 ){
      motorScale = 0;
      targetCount++;
    }
    else if (RightDrive.get_actual_velocities()[0] == 0 && !disableNotMoveCheck) targetCount += .25;
    else targetCount =0;

    double pwrR = slewR.update(spd*motorScale*SaveDir*RightScale);
    double pwrL = slewL.update(spd*motorScale*SaveDir*LeftScale);

    RightDrive.move_voltage(pwrR);
    LeftDrive.move_voltage(-pwrL);
    pros::delay(10);
  }while(targetCount < 10);

  RightDrive.brake();

  LeftDrive.brake();

}
/**
turns a given amount of degrees to turn to the heading specified when the function is run
*/
void turnDeg(double degrees, double spd){
  double tempAngle = info.direc;
  double angleRad = degrees;
  double target; //total amsount to change
  double motorScale;
  double deltaO;
  double targetCount = 0;

  double Kp = 925;
  double Ki = 0;
  double Kd = 17000;

  double Integral;
  double derivative;
  double prevDeltaO;
  double Error ;
  //reduces direction of the current heading to within 360 degrees
  reduceDirec(tempAngle); 
  target =angleRad; 
  reduceDirec(target);
  toShortestAngle(target);
  //creates a slew object to handle the voltage sent to each motor
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
    if(std::abs(deltaO) < 0.4){
      Integral = 0;
      motorScale = 0;
      targetCount++;
    }
    else if (std::abs(RightDrive.get_actual_velocities()[0]) < 0.002) targetCount += .4;
    else targetCount =0;

    //Calcuates power to apply to motors based on current voltage.
    double pwr = slew.update(spd*motorScale);

    RightDrive.move_voltage(pwr);
    LeftDrive.move_voltage(pwr);
    //Debug to keep track of PID
    printf("P: %d I: %d D: %d Error: %f\n", (int)(deltaO*Kp), (int) (Integral * Ki), (int)(derivative*Kd), info.direc);

    delay(10);
  }

  RightDrive.brake();
  LeftDrive.brake();
}

