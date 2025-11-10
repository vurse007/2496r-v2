#pragma once
#include "auton.hpp"
#include "main.h"
#include <vector>

using namespace pros;

inline int auton_selector(std::vector<Auton>& autons, pros::Controller& controller) {
  short int selected = 0;
  int timer = 0;

  while (true) {
    if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      if (timer % 50 == 0 && timer % 100 != 0) {
        controller.print(0, 0, "Select: %s", autons.at(selected).get_name1().c_str());
      }
      if (timer % 200 == 0 && timer % 400 != 0) {
        controller.print(1, 0, "Info: %s", autons.at(selected).get_name2().c_str());
      }

      if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT) &&
          selected > 0) {
          pros::delay(10);
          controller.clear_line(1);
          pros::delay(10);
          
        selected--;
      }
      if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT) &&
          selected < (int)autons.size() - 1) {
        pros::delay(10);
        controller.clear_line(1);
        pros::delay(10); 
        selected++;
      }
    } 
    else {
      controller.clear();
      controller.print(0, 0, "Selected:");
      controller.print(1, 0, "%s %s",
                       autons.at(selected).get_name1().c_str(),
                       autons.at(selected).get_name2().c_str());
      pros::delay(800);
      controller.clear();
      return selected;  // return index instead of full Auton
    }

    pros::delay(1);
    timer++;
  }
}
