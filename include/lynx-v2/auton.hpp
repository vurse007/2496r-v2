#include "main.h"
#include <string>

using AutonFunc = void(*)();

class Auton {
private:
  std::string namep1;
  std::string namep2;
  AutonFunc auton;
  std::string color;

public:
  Auton(const std::string& init_name1,
        const std::string& init_name2,
        AutonFunc init_auton,
        const std::string& init_color)
    : namep1(init_name1), namep2(init_name2), auton(init_auton), color(init_color) {}

  void run() { if (auton) (*auton)(); }

  std::string get_name1() const { return namep1; }
  std::string get_name2() const { return namep2; }
  std::string get_color() const { return color; }
};

extern Auton* auton;       // pointer to currently selected auton
extern std::string names;  // name of currently selected auton