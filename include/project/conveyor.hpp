#pragma once

#include "api.h"
#include "tomobo/odometry/Point.hpp"
#include "tomobo/units.hpp"

class Conveyor {

public:
    Conveyor();
    void oc_update();
    void setPower(float power);
    void toggleState();

    //void conveyorAmount(int amount);
    //static tomobo::Point relative(tomobo::Point centre_relative,
    //    okapi::QAngle angle);

    enum class States {
        IDLE,
        INTAKING,
        REVERSE
    };

    States state = Conveyor::States::IDLE;

    pros::Motor motor;
};
