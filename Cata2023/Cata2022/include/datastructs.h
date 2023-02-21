#include "main.h"
#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

struct XYData{
  double X;
  double Y;
};

struct AnglerData{
  int resAng;
  int dsAng;
};

struct CoreData{
  bool resetDirecc;
  double direc;//radians
  double x;
  double y;
  double resetVal;
  double driveticks;
  double encTicks;
  const double StartAngle = 0;
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
