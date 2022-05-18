#include "project/conveyor.hpp"
#include "project/robot.hpp"

const okapi::QLength INTAKE_FROM_CENTER = 6_in;

using namespace tomobo;

Conveyor::Conveyor() :
    motor(INTAKE_PORT, pros::E_MOTOR_GEARSET_06, true){};

void Conveyor::oc_update() {
    switch (state) {
    case Conveyor::States::INTAKING:
        setPower(1);
        break;
    case Conveyor::States::IDLE:
        setPower(0);
        break;
    case Conveyor::States::REVERSE:
        setPower(-1);
        break;
    }
};

void Conveyor::setPower(float power) {
    // 100% power means we need to send 12V (12000 mV) to the motors
    motor.move_voltage(12000 * power);
}

void Conveyor::toggleState() {
    switch (state) {
    case Conveyor::States::IDLE:
        state = Conveyor::States::INTAKING;
        break;
    case Conveyor::States::INTAKING:
        state = Conveyor::States::IDLE;
        break;
    }
}

/*

// Transform a point from being relative to the centre of the bot, to being
// relative to the conveyor
Point Conveyor::relative(Point centre_relative, okapi::QAngle angle) {
    double angle_rad = angle.convert(okapi::radian);
    return { centre_relative.x - (sin(angle_rad) * INTAKE_FROM_CENTER),
        centre_relative.y - (cos(angle_rad) * INTAKE_FROM_CENTER) };
}
*/
