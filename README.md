# 2496R, Version 2

*optimal file tree:*

> 2496r-v2/
> 
> ├── include/
> 
> │   ├── main.h                           (PROS API includes - already good)
> 
> │   ├── api.h                            (PROS dependencies - already good)
> 
> │   ├── config.hpp                       (Global instances only)
> 
> │   │
> 
> │   └── lynx/                            (Rename from lynx-v2)
> 
> │       ├── types/
> 
> │       │   ├── motor_specs.hpp          (motor_specs struct)
> 
> │       │   ├── point.hpp                (point struct for odom)
> 
> │       │   └── constants.hpp            (PID constants struct)
> 
> │       │
> 
> │       ├── core/
> 
> │       │   ├── pid.hpp
> 
> │       │   ├── util.hpp
> 
> │       │   └── timer.hpp                (Extract timer class from util)
> 
> │       │
> 
> │       ├── chassis.hpp                  (depends on motor_specs, pid)
> 
> │       ├── odometry.hpp                 (depends on point, util)
> 
> │       └── global.hpp                   (forward declarations only - no implementation)
> 
> │
> 
> ├── src/
> 
> │   ├── main.cpp                         (entry point)
> 
> │   ├── autonomous.cpp                   (optional: autonomous routines)
> 
> │   └── control.cpp                      (optional: opcontrol code)
> 
> │
> 
> ├── firmware/
> ├── bin/
> └── other build files...
