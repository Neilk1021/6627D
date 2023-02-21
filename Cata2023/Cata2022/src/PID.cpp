#include "chassis.h"

PIDValues::PIDValues(double P, double I, double D, double* Pos){
    this->TrackVal = Pos;
    this->P=P;
    this->I=I;
    this->D=D;
}   

PID::PID(PIDValues * PIDV, double errorAmnt = 0.25, double velocityError = 0.002){
    this->PIDV = PIDV;
    this->errorAmnt =errorAmnt;
    this->velocityError = velocityError;
    targetCount = 0;
    InError = *this->PIDV->TrackVal;
    reset();
    update();
}

void PID::update(){
    deltaO = *PIDV->TrackVal - InError;
    Integral += deltaO;

    if(std::abs(Integral) > 1900)
      Integral = std::clamp<double>(Integral, -1900, 1900);

    Derrivative = deltaO - prevDeltaO;
    prevDeltaO = deltaO;
    *Speed = (deltaO * PIDV->P) + (Integral * PIDV->I) + (Derrivative*PIDV->D);

    if(std::abs(deltaO) < errorAmnt){
      Integral = 0;
      *Speed = 0;
      targetCount++;
    }
    else if (std::abs(Derrivative) < velocityError) targetCount += 0.22;
    else targetCount =0;
}

void PID::setGoal(double deltaO){
    this->deltaO = deltaO;
}

void PID::reset(){
    this->deltaO = 0;
    this->Derrivative = 0;
    this->Integral = 0;
    this->prevDeltaO = 0;
    *this->Speed =0;
    targetCount=0;
}
