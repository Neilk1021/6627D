#include "consts.h"
#include <atomic>
#include <cmath>

struct vector2{
  public:
    double X;
    double Y;
    vector2(double x = 0, double y = 0){
      X = x;
      Y = y;
    }
};

class Line{
  public:
    vector2 startPos;
    vector2 endPos; 

    Line(double startingX = 0, double startingY = 0, double EndingX= 1, double EndingY = 1){
      startPos = vector2(startingX, startingY);
      endPos = vector2(EndingX, EndingY);
    }
    Line(vector2 startPos, vector2 endPos){
      startPos = startPos;
      endPos = endPos;
    }

  void lerp(float pctDone, double& x, double& y){
    x = (endPos.X - startPos.X) * pctDone + startPos.X; 
    y = (endPos.Y - startPos.Y) * pctDone + +startPos.Y;
  }

  double distance(){
    double x = (endPos.X - startPos.X);
    double y = (endPos.Y - startPos.Y);
    return std::sqrt(x*x + y*y);
  }
};

class BezierCurve{
  Line AdjustLineStart;
  Line AdjustLineEnd;
  Line BodyLine;

  BezierCurve(vector2 PathStart, vector2 AdjustPointStart, vector2 AdjustPointEnd, vector2 PathEnd){
    AdjustLineStart = Line(PathStart, AdjustPointStart);
    BodyLine = Line(AdjustPointStart, AdjustPointEnd);
    AdjustLineEnd = Line(AdjustPointEnd, PathEnd);
  }

};

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

    //printf("%*.*f\n", 5, 4, deltaO);
    // "Proportional" but scuffed
    motorScale = (deltaO * Kp) + (Integral* -Ki) + (derivative*Kd);
    //motorScale = ( motorScale >= 1 ? 1 : motorScale );
    //clamp(motorScale, .25, 1);     } 
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

    if(std::abs(deltaO) < 1.5){
      motorScale = 0;
      targetCount++;
    }else targetCount =0;

    RightDriveMotor1.move_voltage(spd*motorScale*RightScale*signOf(deltaO));
    RightDriveMotor2.move_voltage(spd*motorScale*RightScale*signOf(deltaO));

    LeftDriveMotor1.move_voltage(-spd*motorScale*LeftScale*signOf(deltaO));
    LeftDriveMotor2.move_voltage(-spd*motorScale*LeftScale*signOf(deltaO));
    pros::delay(5);
  }while(targetCount < 8);

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
  
  double Kp = 100;
  double Ki = 4;
  double Kd = 10;

//0.001
//0.3
  float xval = 0;
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
    Integral = Integral + deltaO*0.05;
    if(std::abs(deltaO) < 0.9){
      Integral = 0;
      targetCount++;
    }
    else{
      targetCount = 0;
    }

    if(Integral > 6000)
      Integral = 6000;

    derivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;

    motorScale = (deltaO * Kp) + (Integral * Ki) + (derivative*Kd);
    //motorScale = ( motorScale >= 1 ? 1 : motorScale );
    motorScale = std::abs(motorScale);
    if(std::abs(deltaO) > 0.9)clamp(motorScale, minClamp, 120000);
    //printf("%*.*f\n", 5, 4, deltaO);
    
    RightDriveMotor1.move_voltage(spd*motorScale*signOf(deltaO));
    RightDriveMotor2.move_voltage(spd*motorScale*signOf(deltaO));

    LeftDriveMotor1.move_voltage(spd*motorScale*signOf(deltaO));
    LeftDriveMotor2.move_voltage(spd*motorScale*signOf(deltaO));

    xval+=1;    

    delay(5);
  }

  RightDriveMotor1.brake();
  RightDriveMotor2.brake();

  LeftDriveMotor1.brake();
  LeftDriveMotor2.brake();
}

