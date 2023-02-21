#include "consts.hpp"
#include "main.h"
#include <deque>
#include <string>
#include <fstream>
#include "Vector2.h"
void Tracker(){

    Encoder.reset();
    deque<double> pastDist;
    deque<XYData> pastXY;
    int Depth = 2;
    for (int i = 0; i < Depth; i++) {
        pastDist.push_front(0);
        pastXY.push_front({0,0});
    }

    while (InertialSensor.is_calibrating()) continue;
            //FILE* stder = fopen("serr", "w+");
    while (true) {

        info.direc = InertialSensor.get_rotation();

        info.encTicks = -Encoder.get_value()/(2000.0/24.0);
        pastDist.push_front(info.encTicks);

        double X = cos((info.direc + info.StartAngle)*PI/180)*(pastDist.front() - pastDist.back());
        double Y = sin((info.direc + info.StartAngle)*PI/180)*(pastDist.front() - pastDist.back());

        pastXY.push_front({X,Y});
        pastXY.pop_back();
        pastDist.pop_back();

        
        info.x += pastXY[0].X;
        info.y += pastXY[0].Y;

        int XINT = info.x * 100;
        int YINT = info.y * 100;


        //fputs(x, stder);
        // printf("Degree:%*.*f ", 5, 4, info.direc);
        // printf("X:%*.*f ", 5, 4, info.x);
        // printf("Y:%*.*f\n", 5, 4, info.y);
        delay(25);
    }
}