#include "chassis.h"
#include "consts.hpp"
// Chassis::Chassis(std::vector<pros::Motor> *LS, std::vector<pros::Motor> *RS, CoreData info){
//     // this->LS = *LS;
//     // this->RS = *RS;
//     // this->info = &info;
// }

Chassis::Chassis(pros::Motor_Group LS, pros::Motor_Group RS, CoreData info){
    for (int i = 0; i < LS.size(); i++) {
        this->LS.push_back(LS[i]);
    }
    for (int i = 0; i < RS.size(); i++) {
        this->RS.push_back(RS[i]);
    }
        this->info = &info;
}

void Chassis::TurnDeg(double deg){
    CoreData Data = *info;
    PIDValues DegVal= PIDValues(400, 0 , 130, &Data.encTicks);
    PID DegPID = PID(&DegVal, 0.25, 0.002);

    Slew_t slew;

    while (DegPID.targetCount < 8) {
        DegPID.update();
        double pwr = slew.update(*DegPID.Speed);
        for (int i = 0; i < LS.size(); i++) {
            LS[i].move_voltage(pwr);
        }
        for (int i = 0; i < RS.size(); i++) {
            RS[i].move_voltage(pwr);
        }
        pros::delay(10);
    }

    for (int i = 0; i < LS.size(); i++) {
        LS[i].brake();
    }
    for (int i = 0; i < RS.size(); i++) {
        RS[i].brake();
    }
} 

void Chassis::MoveToPoint(Vector2 Des, double speed){
    Vector2 Dis = Des - Vector2(info->x, info->y);
    Vector2 Velocity(0,0);
    Vector2 PrevPos(0, 0);
    
    float threshold = 0.1;

    while (Vector2::MAG(Dis) < threshold) {

    }
}
