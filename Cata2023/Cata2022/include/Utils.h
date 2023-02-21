#ifndef UTILITIES
#define UTILITIES
#include "main.h"

namespace Utils{
    class Button{
        private:
            void (*Action)();
            pros::controller_digital_e_t ButtonPress;
            pros::Controller Con = pros::E_CONTROLLER_MASTER;
        public:
            void Fire();
            Button(void ActionPress(), pros::Controller, pros::controller_digital_e_t);
    };

    class Controller{
      private:
        std::vector<Utils::Button> Buttons;
                void ButtonsPress();
      public: 
        Controller(std::vector<Utils::Button>);
        void Drive();
    };
}

#endif