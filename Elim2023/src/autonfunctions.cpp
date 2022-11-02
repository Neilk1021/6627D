#include "consts.h"
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

    RightDriveMotor1.move_velocity(spd*motorScale*rightScale*signOf(dis));
    RightDriveMotor2.move_velocity(spd*motorScale*rightScale*signOf(dis));

    LeftDriveMotor1.move_velocity(-spd*motorScale*leftScale*signOf(dis));
    LeftDriveMotor2.move_velocity(-spd*motorScale*leftScale*signOf(dis));

    //RightDriveTrain.spin(directionType::rev, spd*motorScale*rightScale, pct);
    //LeftDriveTrain.spin(directionType::fwd, spd*motorScale*leftScale, pct);

    pros::delay(5);

  } while (absDif < std::abs(encTicks));

  RightDriveMotor1.brake();
  RightDriveMotor2.brake();

  LeftDriveMotor1.brake();
  LeftDriveMotor2.brake();


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

    RightDriveMotor1.move_velocity(spd*motorScale);
    RightDriveMotor2.move_velocity(spd*motorScale);

    LeftDriveMotor1.move_velocity(-spd*motorScale);
    LeftDriveMotor2.move_velocity(-spd*motorScale);
    pros::delay(5);
  }while(targetCount < 8);

  RightDriveMotor1.brake();
  RightDriveMotor2.brake();

  LeftDriveMotor1.brake();
  LeftDriveMotor2.brake();

}

void DriveToPoint(double dis, double spd, double Heading, int minClamp, bool hasFunc, void DistanceFunc(), float distanceToFireFunc) {
  double encTicks = dis* 19;
  double startTicks = info.driveticks;
  double absDif;
  double motorScale;
  double deltaO;
  int targetCount = 0;

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


    if(std::abs(deltaO) < 0.75 || derivative == 0){
      motorScale = 0;
      targetCount++;
    }else targetCount =0;

    RightDriveMotor1.move_voltage(spd*motorScale*RightScale*signOf(deltaO));
    RightDriveMotor2.move_voltage(spd*motorScale*RightScale*signOf(deltaO));

    LeftDriveMotor1.move_voltage(-spd*motorScale*LeftScale*signOf(deltaO));
    LeftDriveMotor2.move_voltage(-spd*motorScale*LeftScale*signOf(deltaO));
    pros::delay(10);
  }while(targetCount < 7);

  RightDriveMotor1.brake();
  RightDriveMotor2.brake();

  LeftDriveMotor1.brake();
  LeftDriveMotor2.brake();

}

void turnDeg(double degrees, double spd, int minClamp){
  double tempAngle = info.direc;
  double angleRad = degrees;
  double target; //total amount to change
  double currentAmount; //stacking total of amount that has changed
  double motorScale;
  double deltaO;
  int targetCount = 0;

  // double Kp = 1.60/2;
  // double Ki = 0.000001;
  // double Kd = 0.35;
  
  double Kp = 215;
  double Ki = -5;
  double Kd = -75;

//0.001
//0.3
  double Integral;
  double derivative;
  double prevDeltaO;

  reduceDirec(tempAngle); 
  //target = angleRad - tempAngle;
  target =angleRad; 

  reduceDirec(target);
  toShortestAngle(target);

  while(targetCount < 10){
    tempAngle = info.direc;
    deltaO = target + info.direc;
    Integral = Integral + deltaO*0.10;
    if(std::abs(deltaO) < 1 || derivative == 0){
      Integral = 0;
      targetCount++;
    }
    else{
      targetCount = 0;
    }

    if(Integral > 6000)
      Integral = 6000;
    else if (Integral < -6000) Integral = -6000;

    derivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;

    motorScale = (deltaO * Kp) + (Integral * Ki) + (derivative*Kd);
    motorScale += minClamp * signOf(motorScale);
    //motorScale = ( motorScale >= 1 ? 1 : motorScale );
    //printf("%*.*f\n", 5, 4, deltaO);
    
    RightDriveMotor1.move_voltage(spd*motorScale);
    RightDriveMotor2.move_voltage(spd*motorScale);

    LeftDriveMotor1.move_voltage(spd*motorScale);
    LeftDriveMotor2.move_voltage(spd*motorScale);

    delay(10);
  }

  RightDriveMotor1.brake();
  RightDriveMotor2.brake();

  LeftDriveMotor1.brake();
  LeftDriveMotor2.brake();
}

