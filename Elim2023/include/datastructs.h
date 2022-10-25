#include "main.h"
#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

struct EncoderData{
  double encLVal;
  double encRVal;
};

struct AnglerData{
  int resAng;
  int dsAng;
};

struct CoreData{
  bool resetDirecc;
  double direc;//radians
  double leftpct;
  double rightpct; 
  double resetVal;
  double driveticks;
  double encTicks;
  double CurrentRPM;
  int desiredAngle;
  int AutonNumber;
  double desiredFlywheelSpeed;
};

struct Point{
  double x;
  double y;
  double heading;
};

#endif
