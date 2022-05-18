#pragma once

#include "main.h"
#include "tomobo/units.hpp"

#define PI 3.14159265358979323846

#include "project/configManager.hpp"

#include "project/arm.hpp"
#include "project/chassis.hpp"
#include "project/conveyor.hpp"
#include "project/mogoClamp.hpp"
#include "project/tilter.hpp"
#include "util.hpp"

#define TILTER_PORT 7
#define INTAKE_PORT 8
#define ARM_PORT 5
#define CLAMP_PORT 3

#define CLAMP_DISTANCE_PORT 17
#define INERTIAL_PORT 18

// drive motor ports
#define FL_PORT 4
#define FR_PORT 10
#define BL_PORT 9
#define BR_PORT 2

namespace robot {

extern std::shared_ptr<pros::Controller> controller;

extern std::shared_ptr<Chassis> chassis;
extern std::shared_ptr<Conveyor> conveyor;
extern std::shared_ptr<MogoClamp> mogoClamp;
extern std::shared_ptr<Tilter> tilter;
extern std::shared_ptr<Arm> arm;
extern std::shared_ptr<ConfigManager> configManager;

extern okapi::QLength CENTER_TO_BACK;

void init();
}
