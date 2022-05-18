#include "main.h"
#include "tomobo/units.hpp"

#include "project/chassis.hpp"

// Allow usage of unit literals
using namespace okapi::literals;

namespace robot {

std::shared_ptr<pros::Controller> controller;

std::shared_ptr<Chassis> chassis;
std::shared_ptr<Conveyor> conveyor;
std::shared_ptr<MogoClamp> mogoClamp;
std::shared_ptr<Tilter> tilter;
std::shared_ptr<Arm> arm;
std::shared_ptr<ConfigManager> configManager;

okapi::QLength CENTER_TO_BACK = 9_in;

void init() {
    // A lot of resources aren't available at global ctor (mainly access to peripherals)
    // We need to run anything dealing with these resources after init.

    controller = std::make_shared<pros::Controller>(pros::E_CONTROLLER_MASTER);

    chassis = std::make_shared<Chassis>();
    conveyor = std::make_shared<Conveyor>();
    mogoClamp = std::make_shared<MogoClamp>();
    tilter = std::make_shared<Tilter>();
    arm = std::make_shared<Arm>();
    configManager = std::make_shared<ConfigManager>();

    robot::configManager->load_config();
};

}
