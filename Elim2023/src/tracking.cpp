#include "consts.hpp"
#include "main.h"
#include <cmath>
#include <deque>
using namespace std;

int task_tracking(){
  //std::deque<EncoderData> pastVals;
  deque<EncoderData> pastVals;
  
  EncoderData lastVals;
  EncoderData CurrentVals = {0, 0};
  double deltaO;
  double deltaL;
  double deltaR;
  double deltaD;
  //6.4075
  float r = 10.65;

  for(int i = 0; i < 2 ; i++){  
    pastVals.push_front({getDR(), getDL()});
  }

  lastVals = {getDL(), getDR()};

  while(true){
    CurrentVals.encLVal = 0;
    CurrentVals.encRVal = 0;

    pastVals.push_front( {getDL(), getDR()});

    pastVals.pop_back();

    for(int i = 0; i < 2; i ++){
      CurrentVals.encLVal += pastVals[i].encLVal;
      CurrentVals.encRVal += pastVals[i].encRVal;
    }

    CurrentVals.encLVal /= 2;
    CurrentVals.encRVal /= 2;

      deltaL = -(CurrentVals.encLVal - lastVals.encLVal)* 2.75 * PI / 360;
      deltaR = (CurrentVals.encRVal - lastVals.encRVal)* 2.75 * PI / 360;

      deltaO = -(deltaR - deltaL)/r;
      deltaD = (deltaL + deltaR) / 2.0;
      lastVals = CurrentVals;
      info.direc += R2D(deltaO);
      info.driveticks += deltaD;
      if(info.resetDirecc){
        info.direc = info.resetVal;
        info.resetDirecc = false;
      }
      printf("%*.*f\n", 5, 4, info.direc);
    
      delay(10);
  }
}

// double GetRPM(double StartPos, double EndPos, double delay){
//   return  (EndPos - StartPos)/delay/60;
// }

int OdomTrack(){
  float r = 6.4075;
  float lastDistance = 0; 
  float xval = 0;
  while(1){
    float s1 = getDR() * 2.75 * 3.1415926 / 360;
    float s2 = L.get_value() * 2.75 * 3.1415926 / 360;

    float theta = (s1+s2)/r;
    info.direc = theta * (180/3.1415926);
    info.driveticks = (s1 - s2)/2;
    float deltaD = info.driveticks - lastDistance;
    lastDistance = info.driveticks; 
    info.CurrentRPM = Flywheel.get_actual_velocity();  
    pros::delay(10);
    printf("%*.*f\n", 5, 4, info.direc);
    

  }
}