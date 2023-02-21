#include "datastructs.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <vector>
#include "Vector2.h"

#ifndef CHASSIS
#define CHASSIS

struct PIDValues{
    double P, I, D;
    double *TrackVal;

    PIDValues(double, double, double, double*);
};

class PID{
    private:
        PIDValues * PIDV;
        double deltaO = 0;
        double Integral;
        double Derrivative;
        double prevDeltaO;
        double errorAmnt;
        double velocityError;
        double InError;

    public:
        PID(PIDValues*, double, double);
        void update();
        void setGoal(double);
        double * Speed;
        void reset();
        int targetCount = 0;
};


class Chassis{
    private:
        std::vector<pros::Motor>RS;
        std::vector<pros::Motor> LS;
        CoreData *info;
    public:
        Chassis(std::vector<pros::Motor>*, std::vector<pros::Motor>*, CoreData);
        Chassis(pros::Motor_Group, pros::Motor_Group, CoreData);
        void MoveToPoint(Vector2, double);
        void DriveFor(double, void());
        void TurnDeg(double);
};

#endif