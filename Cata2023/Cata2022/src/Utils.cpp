#include "consts.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <vector>
#include "Utils.h"
Utils::Button::Button(void ActionPress(), pros::Controller Cont ,pros::controller_digital_e_t ButtonPress){
    this->ButtonPress = ButtonPress;
    this->Action = ActionPress;
    this->Con = Cont;
}

void Utils::Button::Fire(){
    if(Con.get_digital_new_press(ButtonPress)){
       Task e(Action);
    }
}

void Utils::Controller::Drive(){
    pros::Task task{[=]{
        while (true) {
            float AxisOne = Con.get_analog(ANALOG_RIGHT_X)*1;
            float AxisThree = Con.get_analog(ANALOG_LEFT_Y)*1.5;
                if(std::abs(AxisThree) > 2 || std::abs(AxisOne) > 2) {
                    LeftDrive.move(-(AxisThree + AxisOne));
                    RightDrive.move((AxisThree - AxisOne));
                }else{
                    RightDrive.brake();
                    LeftDrive.brake();
                }
                pros::delay(10);
        }
    }};
}

void Utils::Controller::ButtonsPress(){
        while (true) {
            for (int i = 0; i < this->Buttons.size(); i++) {
                this->Buttons[i].Fire();
            }
            pros::delay(5);
        };
}

Utils::Controller::Controller(std::vector<Utils::Button> Buttons){
    this->Buttons = Buttons;
        Drive();
    ButtonsPress();
}
