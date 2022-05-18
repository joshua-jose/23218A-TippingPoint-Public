#include "project/robot.hpp"

#include "project/tilter.hpp"

Tilter::Tilter() :
    motor(TILTER_PORT, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES),
    pot('C', pros::E_ADI_POT_V2),
    ultrasonic('A', 'B') {
    motor.set_brake_mode(MOTOR_BRAKE_HOLD);
    // motor.set_current_limit(1500);
    max_rpm = gearsetToRPM(motor.get_gearing());
    motor.tare_position();
};
void Tilter::setPower(float power) {
    // 100% power means we need to send 12V (12000 mV) to the motors
    if (abs(power) < 0.01)
        motor.move_velocity(0); // make sure to hold
    // motor.move_absolute(motor.get_position(), max_rpm);
    else
        motor.move_voltage(12000 * power);
    // Add logic to prevent over-rotating

    /*
    // Stuck anywhere
    if (motor.get_current_draw() > 2000 && motor.get_actual_velocity() < 10)
        motor.move_voltage(0);
    */
}

void Tilter::ocUpdate() {
    // if (state == States::ENGAGED && isEngaged() && motor.get_actual_velocity() < 1)
    //     setPower(0);

    // if (button.get_new_press())
    //     setState(States::SCORING);
    // updatePosition();

    uint32_t time = pros::millis();
    if (time > (moveStart + timeout))
        setPower(0);
}
void Tilter::updatePosition() {
    double target = 0;

    if (state == States::STOWED)
        target = stowedPosition;
    else if (state == States::SCORING)
        target = scoringPosition;
    else if (state == States::DISENGAGED)
        target = disengagedPosition;
    else if (state == States::LIFTING)
        target = liftingPosition;

    motor.move_absolute(target, max_rpm);
    /*
    double pid_out = pid.step(target - pot.get_angle());

    if (pot.get_angle() > target + positionDeadband)
        setPower(pid_out);
    else if (pot.get_angle() < target - positionDeadband)
        setPower(pid_out);
    else
        setPower(0);
    */
    // if (pros::millis() > (moveStart + timeout))
    //     setPower(0);

    /*
    if (pot.get_angle() > target + positionDeadband)
        setPower(-1);
    else if (pot.get_angle() < target - positionDeadband)
        setPower(1);
    else
        setPower(0);

    if (pros::millis() > (moveStart + timeout))
        setPower(0);
    */
}

bool Tilter::isScoring() {
    double position = motor.get_position();

    double mogoInTilter = ultrasonic.get_value() < 55;
    if (!mogoInTilter)
        return false;

    return (position > (scoringPosition - positionDeadband)) && (position < (scoringPosition + positionDeadband));
}

bool Tilter::hasMogo() {
    // return false;
    return ultrasonic.get_value() < 65;
}

void Tilter::setState(Tilter::States istate) {
    state = istate;
    moveStart = pros::millis();
    updatePosition();
}
