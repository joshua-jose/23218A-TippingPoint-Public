#pragma once

#include "api.h"
#include "tomobo/units.hpp"

#include <atomic>

class Arm {

public:
    Arm();
    // void oc_update();

    void setPower(float power);
    void setVelocity(float velocity);
    void setHeight(okapi::QLength height);
    void hardTare();
    okapi::QLength getHeight();

    // okapi::QAngle getAnglePot();
    okapi::QAngle getAngleMotor();

    // pros::ADIAnalogIn potentiometer;
    pros::Motor motor;

    double max_rpm = 200;
    std::atomic_bool tared{ false };

    /*
    pot max/min: Values of potentiometer at max/min height
    rot max/min: Values of motor encoder at max/min height
    deg max/min:        angle of the arm at max/min height
    rot_min is usually 0, rot_max = (deg_max - deg_min) * ratio
    */
    // const double pot_max = 1600, pot_min = 150;
    const double rot_max = 417, rot_min = 0;
    const double deg_max = 65, deg_min = -46;

    const okapi::QLength max_height = 39.5 * okapi::centimeter,
                         min_height = 2 * okapi::centimeter;

    const okapi::QLength arm_radius = 10.5 * okapi::inch;
    const okapi::QLength pivot_height = 7.1 * okapi::inch;

    const okapi::QLength bridgeHeight = 14 * okapi::inch;
};
