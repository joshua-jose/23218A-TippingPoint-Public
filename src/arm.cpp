#include "project/robot.hpp"

#include "project/arm.hpp"
#include "project/util.hpp"

Arm::Arm() :
    motor(ARM_PORT, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES) {
    motor.set_brake_mode(MOTOR_BRAKE_HOLD);

    max_rpm = gearsetToRPM(motor.get_gearing());
    motor.tare_position();
    // Get how many degrees offset the motor encoder is from true 0
    // double rot_offset = getAnglePot().convert(okapi::degree);
};

void Arm::setPower(float power) {
    // 100% power means we need to send 12V (12000 mV) to the motors
    if (abs(power) < 0.01)
        motor.move_velocity(0); // make sure to hold
    else
        motor.move_voltage(12000 * power);
}

void Arm::setVelocity(float velocity) {
    motor.move_velocity(max_rpm * velocity);
}
/*
okapi::QAngle Arm::getAnglePot(){
    double pot_current = potentiometer.get_value();
    double angle_deg = scale(pot_current, pot_min, pot_max, deg_min, deg_max);
    return okapi::degree * angle_deg;
}
*/

okapi::QAngle Arm::getAngleMotor() {
    double motor_current = motor.get_position();
    double angle_deg = scale(motor_current, rot_min, rot_max, deg_min, deg_max);
    return okapi::degree * angle_deg;
}

void Arm::setHeight(okapi::QLength height) {
    // int pot_delta = abs(pot_max - pot_min);
    // double target = scale(height.convert(meter), min_height.convert(meter), max_height.convert(meter), pot_min, pot_max);

    // y = r sin(theta) + pivot_height
    double target_angle = asin((height - pivot_height).convert(meter) / arm_radius.convert(meter)) * (180 / PI);
    // scale from arm degrees to motor degrees
    double target_rot = scale(target_angle, deg_min, deg_max, rot_min, rot_max);

    if (height >= 4_cm && height <= 6_cm)
        target_rot = 150;

    if (height > 39_cm)
        target_rot = 800;

    if (height < 0.1_cm)
        target_rot = 1.0;
    motor.move_absolute(target_rot, 100);
}

okapi::QLength Arm::getHeight() {
    double angle = scale(motor.get_position(), rot_min, rot_max, deg_min, deg_max);

    return ((sin(angle * (PI / 180)) * arm_radius) + pivot_height);
}

void Arm::hardTare() {
    setPower(-1); // start backing into hard limit
    pros::delay(100); // Wait for clamp to get up to speed
    setPower(-0.75); // dont ram into back at full power
    motor.set_current_limit(500);

    while (abs(motor.get_actual_velocity()) > 50) // wait until we hit something
        pros::delay(20);

    motor.set_current_limit(100);
    setPower(-0.25); // just enough power to keep it pushed against the limit

    motor.tare_position(); // this is our new zero position

    // Let motor return to full(ish) power
    motor.set_current_limit(2500);

    // Nudge the clamp forward just a tiny bit
    setPower(1);
    pros::delay(100);
    setPower(0);

    tared.store(true);
}