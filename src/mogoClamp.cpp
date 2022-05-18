#include "project/robot.hpp"

#include "project/mogoClamp.hpp"

extern "C" int32_t vexDeviceGetTimestampByIndex(int32_t index);
// int32_t timestamp = vexDeviceGetTimestampByIndex(ARM_PORT - 1);

MogoClamp::MogoClamp() :
    motor(CLAMP_PORT, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES),
    button('F'),
    dsense(CLAMP_DISTANCE_PORT) {
    motor.set_brake_mode(MOTOR_BRAKE_HOLD);
    // motor.set_current_limit(1500);
    max_rpm = gearsetToRPM(motor.get_gearing());
    motor.tare_position();
};

void MogoClamp::setPower(float power) {
    // 100% power means we need to send 12V (12000 mV) to the motors
    if (abs(power) < 0.01) {
        motor.move_velocity(0);
        // if (!holding)
        //     motor.move_relative(0, max_rpm);
        holding = true;
    } else {
        holding = false;
        motor.move_voltage(12000 * power);
    }

    // Add logic to prevent over-rotating
    /*
    // Stuck anywhere
    if (motor.get_current_draw() > 2000 && motor.get_actual_velocity() < 10)
        return;
    */
}

void MogoClamp::ocUpdate() {
    if (state == States::ENGAGED && isEngaged() && motor.get_actual_velocity() < 1)
        setPower(0);

    // if (button.get_new_press())
    //     setState(States::ENGAGED);

    if (getNewMogo()) {
        setState(States::ENGAGED);
        robot::controller->rumble("-");
    }
    /*
    else if (lastDistance > 3.5_cm && distance <= 3.5_cm) {
        moveStart = pros::millis();
        motor.move_absolute(55 * ratio, max_rpm * moveSpeed);
    } else if (distance > 4_cm && l)
        setState(States::DISENGAGED);
    */

    // updatePosition();

    uint32_t time = pros::millis();
    if (time > (moveStart + timeout))
        setPower(0);
}

void MogoClamp::updatePosition() {

    /*
    double target = 0;

    if (state == States::STOWED)
        target = 0;
    else if (state == States::ENGAGED)
        target = engagedPosition;
    else if (state == States::DISENGAGED)
        target = disengagedPosition;

    if (state == States::ENGAGED)
        motor.set_brake_mode(MOTOR_BRAKE_HOLD);
    else {
        motor.move_velocity(0);
        if (abs(motor.get_actual_velocity()) < 5.0)
            motor.set_brake_mode(MOTOR_BRAKE_COAST);
        else
            motor.set_brake_mode(MOTOR_BRAKE_BRAKE);
    }

    if (motor.get_position() < target + (2 * positionDeadband) && motor.get_position() > target - (2 * positionDeadband)) {
        if (!holding) {
            motor.move_absolute(target, max_rpm);
            printf("holding in deadband\n");
            holding = true;
        }
    } else {
        if (motor.get_position() > target + positionDeadband)
            setPower(-1);
        else if (motor.get_position() < target - positionDeadband)
            setPower(1);
    }

    if (pros::millis() > (moveStart + timeout))
        setPower(0);
        */

    if (state == States::STOWED)
        motor.move_absolute(0, max_rpm * moveSpeed);
    else if (state == States::ENGAGED)
        motor.move_absolute(engagedPosition, max_rpm * moveSpeed);
    else if (state == States::DISENGAGED)
        motor.move_absolute(disengagedPosition, max_rpm * moveSpeed);
}

int32_t MogoClamp::updateDistance() {
    lastDistance = distance;
    int32_t raw_distance = dsense.get();
    double raw_vel = dsense.get_object_velocity();

    if (raw_distance == 0 || raw_distance == PROS_ERR)
        return distance.convert(okapi::millimeter);

    int32_t timestamp = vexDeviceGetTimestampByIndex(CLAMP_DISTANCE_PORT - 1);

    /*
    kf.predict();
    if (timestamp > lastTimestamp && raw_distance != 0) {
        kf.update(raw_distance * millimeter, raw_vel * mps);
        lastTimestamp = timestamp;
    }

    printf("graph:%d,%d,%f,%f,%f\n", pros::millis(), raw_distance, kf.x.mean, raw_vel, kf.dx.mean / 0.01);
    */
    distance = raw_distance * okapi::millimeter;
    // printf("graph:%d,%d,%f,%f,%f\n", pros::millis(), raw_distance, distance.convert(meter), raw_vel, raw_vel);
    return raw_distance;
}

bool MogoClamp::getNewMogo() {
    lastDistance = distance;
    int32_t raw_distance = dsense.get();
    if (raw_distance == 0 || raw_distance == PROS_ERR)
        return false;
    distance = raw_distance * okapi::millimeter;
    bool lastMogoNear = mogoNear;

    if (distance <= clampDistance)
        mogoNear = true;
    else if (distance >= deadbandDistance)
        mogoNear = false;

    return mogoNear && !lastMogoNear;
}

bool MogoClamp::autonGetNewMogo() {

    lastDistance = distance;
    int32_t raw_distance = dsense.get();
    if (raw_distance == 0 || raw_distance == PROS_ERR)
        return false;
    distance = raw_distance * okapi::millimeter;
    bool lastMogoNear = mogoNear;

    auto speed = dsense.get_object_velocity() * okapi::mps;
    auto timeAhead = 0_ms;

    if (distance - (speed * timeAhead) <= autonClampDistance)
        mogoNear = true;
    else if (distance >= deadbandDistance)
        mogoNear = false;

    return mogoNear && !lastMogoNear;
}

void MogoClamp::setState(MogoClamp::States istate) {
    mutex.take(TIMEOUT_MAX);
    state = istate;
    moveStart = pros::millis();
    mutex.give();
    updatePosition();
}

bool MogoClamp::isEngaged() {
    return motor.get_position() >= (engagedPosition - positionDeadband);
};
bool MogoClamp::isDisengaged() {
    return motor.get_position() <= (disengagedPosition + positionDeadband);
};

void MogoClamp::hardTare() {
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

void MogoClamp::startTask() {
    if (task)
        return; // TODO: Log

    runTask.store(true);
    task = std::make_unique<pros::Task>(
        [=] {
            taskRunning.store(true);
            while (true) {
                mutex.take(TIMEOUT_MAX);
                updatePosition();
                mutex.give();
            }
            taskRunning.store(false);
        },
        TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "MogoClamp_Task");
};

void MogoClamp::stopTask() {
    if (!task)
        return; // TODO: Log

    task->notify();
    runTask.store(false);

    while (taskRunning.load())
        pros::delay(5);

    task.reset();
    mutex.give(); // Is this needed? try remove in testing.
};
