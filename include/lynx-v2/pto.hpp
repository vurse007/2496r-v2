#pragma once
#include "main.h"

namespace lynx {
    class PTO{
        public:
            bool engaged;
            char piston_port;
            pros::adi::Pneumatics piston;
            void (*chainedengage)();
            void (*chaineddisengage)();


            PTO(char port, bool init_state, void (*chainedengaged)() = nullptr, void (*chaineddisengaged)() = nullptr): piston_port(port), piston(port, init_state), engaged(init_state), chainedengage(chainedengaged), chaineddisengage(chaineddisengaged) {}

            void engage(){
                piston.set_value(true);
                engaged = true;
                if (chainedengage != nullptr) {
                    chainedengage();
                }
            }

            void disengage(){
                piston.set_value(false);
                engaged = false;
                if (chaineddisengage != nullptr) {
                    chaineddisengage();
                }
            }

            void toggle(){
                if (engaged){
                    disengage();
                } else {
                    engage();
                }
            }

    };
}