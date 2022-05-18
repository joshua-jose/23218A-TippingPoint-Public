#pragma once

#include "api.h"
#include "tomobo/controller/PIDController.hpp"
#include "tomobo/drivetrain/XDriveController.hpp"
#include "tomobo/odometry/EncoderOdometry.hpp"
#include "tomobo/odometry/InertialOdometry.hpp"

class Chassis : public TankDriveController {

public:
    Chassis();
    void oc_update(int32_t joy_y, int32_t joy_a, bool brake_mode = false);

    void updateSensors() override;
    void updateMotors() override;
    void driveStraightHandler() override;

    pros::ADIEncoder tracking_left;
    pros::ADIEncoder tracking_right;
    pros::ADIEncoder tracking_mid;

    pros::Imu inertial;

    pros::Motor fl_motor, fr_motor, bl_motor, br_motor;

    InertialOdometry odometry;
};
