#include "consts.hpp"
#include <atomic>
#include <cmath>

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

void DriveToPoint(double dis, double spd, double Heading, int minClamp, bool hasFunc, void DistanceFunc(), float distanceToFireFunc) {
  double encTicks = dis* 19;
  double startTicks = info.driveticks;
  double absDif;
  double motorScale;
  double deltaO;
  double targetCount = 0;

  double Kp = 400;
  double Ki = 1;
  double Kd = 50;

  double RightScale;
  double LeftScale;
  double correctionAmount = .01;

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

    //printf("%*.*f\n", 5, 4, deltaO);
    // "Proportional" but scuffed
    motorScale = (deltaO * Kp) + (Integral* -Ki) + (derivative*Kd);
    //motorScale = ( motorScale >= 1 ? 1 : motorScale );
    //clamp(motorScale, .25, 1);     } 
    if(hasFunc && !Fired && std::abs(deltaO) < distanceToFireFunc*19) {
      DistanceFunc();
      Fired = true;
    }

    if((info.direc - Heading) > 0.5)  {
      RightScale = 1 + (info.direc - Heading)*correctionAmount;
      LeftScale = 1 - (info.direc - Heading)*correctionAmount;
    }else if ((info.direc - Heading) < -0.5) {
      RightScale = 1 - (info.direc - Heading)*correctionAmount;
      LeftScale = 1 + (info.direc - Heading)*correctionAmount;
    }else {
      RightScale = 1;
      LeftScale = 1;
    }
    clamp(motorScale, minClamp, 10000000);


    if(std::abs(deltaO) < 0.75 ){
      motorScale = 0;
      targetCount++;
    }
    else if (RightDrive.get_actual_velocities()[0] == 0) targetCount += .66;
    else targetCount =0;

    RightDrive.move_voltage(spd*motorScale*RightScale*signOf(deltaO));

    LeftDrive.move_voltage(-spd*motorScale*LeftScale*signOf(deltaO));
    pros::delay(10);
  }while(targetCount < 10);

  RightDrive.brake();

  LeftDrive.brake();

}

void turnDeg(double degrees, double spd, int minClamp){
  double tempAngle = info.direc;
  double angleRad = degrees;
  double target; //total amount to change
  double currentAmount; //stacking total of amount that has changed
  double motorScale;
  double deltaO;
  double targetCount = 0;

  double Kp = 350 *0.8;
  double Ki = 0;
  double Kd = 25;

  double Integral;
  double derivative;
  double prevDeltaO;

  reduceDirec(tempAngle); 
  //target = angleRad - tempAngle;
  target =angleRad; 

  reduceDirec(target);
  toShortestAngle(target);

  while(targetCount < 8){
    
    tempAngle = info.direc;
    deltaO = target + info.direc;
    Integral = Integral + deltaO*0.10;

    if(Integral > 6000)
      Integral = 6000;
    else if (Integral < -6000) Integral = -6000;

    derivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;

    motorScale = (deltaO * Kp) + (Integral * Ki) + (derivative*Kd);
    motorScale < 0 ? 
    clamp(motorScale, -INFINITY, -minClamp) :
    clamp(motorScale, minClamp, INFINITY);

    if(std::abs(deltaO) < 0.4){
      motorScale = 0;
      targetCount++;
    }
    else if (std::abs(RightDrive.get_actual_velocities()[0]) < 0.1) targetCount += .66;
    else targetCount =0;


    RightDrive.move_voltage(spd*motorScale);
    //RightDrive.move_voltage(spd*motorScale);

    LeftDrive.move_voltage(spd*motorScale);
    //LeftDrive.move_voltage(spd*motorScale);

    delay(10);
  }

  //RightDriveMotor1.brake();
  RightDrive.brake();

  //LeftDriveMotor1.brake();
  LeftDrive.brake();
}

