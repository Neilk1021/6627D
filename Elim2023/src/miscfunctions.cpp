#include "consts.hpp"

int signOf(double x) {
  return (x < 0 ? -1: 1);
}

void reduceDirec(double& direc){
  while(direc >= 360){
    direc -= 360;
  }

  while(direc < -180){
    direc += 360;
  }
}

void toShortestAngle(double& theta){
  if(std::abs(theta) >  180){
    theta -= signOf(theta) * 360;
  }
}

void clamp(double& val, double lower, double upper){
  if(val > upper){
    val = upper;
  }else if (val < lower) {
    val = lower; 
  }
}
template <typename T>
T R2D(T rad){
  rad*180/PI;
}

template <typename T>
T D2R(T deg){
  deg*PI/180;
}


