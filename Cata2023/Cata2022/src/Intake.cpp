#include "main.h"
#include "consts.hpp"

bool SlowIntake = false;
float CurrentIntakeSpeed;

void SwitchIntakeSpeed(){
  if(SlowIntake){
    CurrentIntakeSpeed = IntakeSpeed;
    SlowIntake = false;
  }else {
    CurrentIntakeSpeed = IntakeSpeedSlowed;
    SlowIntake = true;
  }
}

enum FWBCS{
  forwardInt, 
  backInt, 
  stopInt,
  slowBackInt
};

FWBCS currentState = stopInt;

void Intake(){
    CurrentIntakeSpeed = IntakeSpeed;
    while(true){
        switch(currentState){
        case forwardInt:
            IntakeM.move_velocity(CurrentIntakeSpeed);
            break;
        case backInt:
            IntakeM.move_velocity(-CurrentIntakeSpeed);
            break;
        case stopInt:
            IntakeM.brake();
            break;
        case slowBackInt:{
            IntakeM.move_velocity(600);
            break;
        }
        }
    }
}

void SwitchIntake(){
  if(currentState == forwardInt){
    currentState = stopInt;
  }else {
    currentState = forwardInt;
  }
}

void SwitchIntakeBack(){
  if(currentState == backInt){
    currentState = stopInt;
  }else {
    currentState = backInt;
  }
}
void SwitchIntakeBackSlow(){
  if(currentState == backInt){
    currentState = stopInt;
  }else {
    currentState = backInt;
  }
}