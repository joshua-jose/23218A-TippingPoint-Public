#pragma once

#include "main.h"
#include "tomobo/kalman_filter/BivariateKalmanFilter.hpp"
#include "tomobo/odometry/Point.hpp"
#include "tomobo/units.hpp"
#include <atomic>

class MogoClamp {

public:
    MogoClamp();

    void setPower(float power);
    void updatePosition();
    void ocUpdate();
    void hardTare();
    int32_t updateDistance();

    bool isEngaged();
    bool isDisengaged();
    bool getNewMogo();
    bool autonGetNewMogo();

    void startTask();
    void stopTask();

    // static tomobo::Point relative(tomobo::Point centre_relative,
    //     okapi::QAngle angle);

    double ratio = 5;
    double moveSpeed = 1;
    double max_rpm = 200;

    double engagedPosition = 143 * ratio;
    double disengagedPosition = 88 * ratio;
    double positionDeadband = 5 * ratio;

    uint32_t moveStart = 0;
    uint32_t timeout = 1 * 1000; // 1s * 1000 = 1000ms

    std::atomic_bool tared{ false };

    enum class States {
        STOWED,
        DISENGAGED,
        ENGAGED
    };

    States state = States::STOWED;

    void setState(States istate);

    pros::Motor motor;
    pros::ADIButton button;

    uint32_t dt = 0.01;
    tomobo::BivariateKalmanFilter kf{ 0.0025, Gaussian{ 0.0 * dt, pow(.06 * dt, 2) } };
    pros::Distance dsense;
    int32_t lastTimestamp{ 0 };
    okapi::QLength distance = 0 * okapi::meter;
    okapi::QLength lastDistance = 0 * okapi::meter;
    bool mogoNear = false;
    okapi::QLength clampDistance = 2.5_cm;
    okapi::QLength autonClampDistance = 4.5_cm;
    okapi::QLength deadbandDistance = 3_cm;

    bool holding = false;

    std::unique_ptr<pros::Task> task;
    // Run task says whether we *want* the task running
    std::atomic_bool runTask{ false };
    // taskRunning is set by the task itself, it definitively tells us if
    // the task is running or not.
    std::atomic_bool taskRunning{ false };
    pros::Mutex mutex;
};
