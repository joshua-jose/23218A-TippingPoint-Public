#pragma once

#include "api.h"
#include "tomobo/odometry/Point.hpp"
#include "tomobo/units.hpp"

class Tilter {

public:
    Tilter();

    void setPower(float power);
    void updatePosition();
    void ocUpdate();

    bool isScoring();
    bool hasMogo();

    // static tomobo::Point relative(tomobo::Point centre_relative,
    //     okapi::QAngle angle);

    double moveSpeed = 1;
    double ratio = 5;
    double max_rpm = 200;

    double stowedPosition = 0 * ratio;
    double scoringPosition = -11.1 * ratio;
    double disengagedPosition = -55 * ratio;
    double liftingPosition = -125 * ratio;
    double positionDeadband = 7 * ratio;

    uint32_t moveStart = 0;
    uint32_t timeout = 1.5 * 1000; // 1s * 1000 = 1000ms

    std::atomic_bool tared{ false };

    enum class States {
        STOWED, // all the way down
        SCORING, // holding a mogo
        DISENGAGED, // ready to tilt a mogo
        LIFTING
    };
    void setState(States istate);
    States state = States::SCORING;

    pros::Motor motor;
    pros::ADIPotentiometer pot;
    pros::ADIUltrasonic ultrasonic;
};
