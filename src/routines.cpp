#include "main.h"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "project/AsyncTask.hpp"
#include "tomobo/MotionProfileGenerator.hpp"
#include "tomobo/pure_pursuit/Path.hpp"
#include <atomic>
#include <future>

using namespace robot;

void timed_right_side() {
    // Drive to neutral goal
    AsyncTask moveTask([] {
        chassis->oc_update(127, 0);
        pros::delay(1100);
        chassis->oc_update(75, 0);
        pros::delay(350);
        chassis->oc_update(0, 0);
    });

    /*
    while (!tare_done.load()) // make sure done taring before moving subsystems
        pros::delay(10);
    */
    // tareTask.wait();

    arm->setHeight(0_cm);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // start in a ready position

    // wait until we get to goal
    WAIT_UNTIL(mogoClamp->getNewMogo() || moveTask.isDone());

    mogoClamp->setState(MogoClamp::States::ENGAGED);

    // wait for chassis to stop moving
    moveTask.wait();
    pros::delay(400); // wait for clamp to settle

    // reverse
    chassis->oc_update(-120, 0);
    pros::delay(1250);
    chassis->oc_update(0, 0);
}

void right_dash() {
    // Drive to neutral goal
    // chassis->driveStraight(5_ft);

    chassis->reset();

    /*
    while (!tare_done.load()) // make sure done taring before moving subsystems
        pros::delay(10);
    */
    // tareTask.wait();

    // arm->setHeight(0_cm);
    arm->setVelocity(0); // arm hold
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // start in a ready position

    // ramp up power in increments over a short period of time
    const double increment = 0.12;
    for (double i = increment; i <= 1; i += increment) {
        chassis->leftPower = i, chassis->rightPower = i;
        pros::delay(10);
    }

    // wait until we get to goal
    int time = pros::millis();
    // WAIT_UNTIL(mogoClamp->getNewMogo() || chassis->isSettled() || (time + 1500 < pros::millis()));
    WAIT_UNTIL(mogoClamp->dsense.get() <= 120 || ((chassis->leftEncoder - chassis->leftStart) / chassis->trackingScales.straight) * okapi::meter > 5_ft || (time + 1500 < pros::millis()));
    chassis->reset(); // tell robot to stop moving

    WAIT_UNTIL(mogoClamp->dsense.get() <= 110 || ((chassis->leftEncoder - chassis->leftStart) / chassis->trackingScales.straight) * okapi::meter > 5_ft || (time + 1500 < pros::millis()));
    mogoClamp->setState(MogoClamp::States::ENGAGED);

    pros::Task([] {
        double target = mogoClamp->engagedPosition;
        while (mogoClamp->motor.get_position() > target + mogoClamp->positionDeadband || mogoClamp->motor.get_position() < target - mogoClamp->positionDeadband) {
            if (mogoClamp->motor.get_position() < target + (2 * mogoClamp->positionDeadband) && mogoClamp->motor.get_position() > target - (2 * mogoClamp->positionDeadband)) {
                if (!mogoClamp->holding) {
                    mogoClamp->motor.move_absolute(target, mogoClamp->max_rpm);
                    mogoClamp->holding = true;
                }
            } else {
                if (mogoClamp->motor.get_position() > target + mogoClamp->positionDeadband)
                    mogoClamp->setPower(-1);
                else if (mogoClamp->motor.get_position() < target - mogoClamp->positionDeadband)
                    mogoClamp->setPower(1);
            }
            pros::delay(10);
        }
    });

    // pros::delay(50); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly

    pros::delay(150);
    chassis->leftPower = -1, chassis->rightPower = -1;
    pros::delay(200);

    // wait for chassis to stop moving
    // chassis->wait();
}

void single_dash() {
    chassis->forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.005, 100_ms)
            .getSettledUtil());
    chassis->angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.035, 0.01, 100_ms)
            .getSettledUtil());

    chassis->odometry.startTask();
    chassis->odometry.setPose({ 9_ft, 1_ft + 2_in, 0_deg });

    right_dash();
    // reverse

    // chassis->reset();

    chassis->forwardDistanceController->slewEnabled = false;
    /*
    chassis->driveStraight(1.5_ft - chassis->odometry.getPose().y);
    while (chassis->odometry.getPose().y > 3.5_ft && chassis->odometry.getPose().y < 7_ft)
        pros::delay(10);
        */
    chassis->driveStraight(-3.00_ft)->wait();
    chassis->forwardDistanceController->slewEnabled = true;
    // mogoClamp->setState(MogoClamp::States::DISENGAGED);
    arm->setHeight(0_cm);
}

void right_single_wp() {
    chassis->forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.005, 100_ms)
            .getSettledUtil());
    chassis->angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.0035, 0.001, 100_ms)
            .getSettledUtil());

    chassis->odometry.startTask();
    chassis->odometry.setPose({ 9_ft, 2_ft - 2_in, 0_deg });

    right_dash();
    // reverse

    // chassis->reset();

    chassis->forwardDistanceController->slewEnabled = false;
    /*
    chassis->driveStraight(1.5_ft - chassis->odometry.getPose().y);
    while (chassis->odometry.getPose().y > 3.5_ft && chassis->odometry.getPose().y < 7_ft)
        pros::delay(10);
    */

    chassis->driveStraight(-5_ft);
    int time = pros::millis();
    WAIT_UNTIL(tilter->ultrasonic.get_value() < 100 && (time + 1500) < pros::millis());
    chassis->forwardDistanceController->slewEnabled = true;
    chassis->odometry.setPose({ 9_ft, 7.5_in, 0_deg });
    pros::delay(500);
    chassis->driveStraight(2_ft - chassis->odometry.getPose().y)->wait(1000);

    // if (chassis->odometry.getPose().y > 3.5_ft)
    //     chassis->driveStraight(6_ft - chassis->odometry.getPose().y);

    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(300);
    // chassis->lookAt({ 11_ft, 3_ft }, 180_deg)->wait(1000);
    chassis->turnAngle(-90_deg)->wait(1000);
    pros::delay(300);
    // chassis->driveDeltaDistance({ 11_ft, 3_ft }, true);
    chassis->driveStraight(-(2_ft - 5_in));

    time = pros::millis();
    WAIT_UNTIL(/*tilter->hasMogo() ||*/ chassis->isSettled() || (time + 1500 < pros::millis()));
    chassis->reset();
    pros::delay(300);
    tilter->setState(Tilter::States::SCORING);
    pros::delay(1200);
    conveyor->setPower(1);
    chassis->driveStraight(2_ft)->wait(1000);
    for (int j = 0; j < 5; j++) {
        conveyor->setPower(0);
        pros::delay(500);
        conveyor->setPower(1);
        pros::delay(500);
    };
}

void middle_goal() {
    chassis->forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.005, 100_ms)
            .getSettledUtil());
    chassis->angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.035, 0.01, 100_ms)
            .getSettledUtil());

    chassis->odometry.startTask();
    chassis->odometry.setPose({ 9_ft, 1_ft + 2_in, 0_deg });

    chassis->reset();

    /*
    while (!tare_done.load()) // make sure done taring before moving subsystems
        pros::delay(10);
    */
    // tareTask.wait();

    // arm->setHeight(0_cm);
    arm->setVelocity(0); // arm hold
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // start in a ready position

    // ramp up power in increments over a short period of time
    const double increment = 0.12;
    for (double i = increment; i <= 1; i += increment) {
        chassis->leftPower = i, chassis->rightPower = i;
        pros::delay(10);
    }

    // wait until we get to goal
    int time = pros::millis();
    // WAIT_UNTIL(mogoClamp->getNewMogo() || chassis->isSettled() || (time + 1500 < pros::millis()));
    WAIT_UNTIL(mogoClamp->dsense.get() <= 80 || ((chassis->leftEncoder - chassis->leftStart) / chassis->trackingScales.straight) * okapi::meter > 5.5_ft || (time + 1500 < pros::millis()));
    chassis->reset(); // tell robot to stop moving

    WAIT_UNTIL(mogoClamp->dsense.get() <= 70 || ((chassis->leftEncoder - chassis->leftStart) / chassis->trackingScales.straight) * okapi::meter > 5.5_ft || (time + 1500 < pros::millis()));
    mogoClamp->setState(MogoClamp::States::ENGAGED);

    pros::Task([] {
        double target = mogoClamp->engagedPosition;
        while (mogoClamp->motor.get_position() > target + mogoClamp->positionDeadband || mogoClamp->motor.get_position() < target - mogoClamp->positionDeadband) {
            if (mogoClamp->motor.get_position() < target + (2 * mogoClamp->positionDeadband) && mogoClamp->motor.get_position() > target - (2 * mogoClamp->positionDeadband)) {
                if (!mogoClamp->holding) {
                    mogoClamp->motor.move_absolute(target, mogoClamp->max_rpm);
                    mogoClamp->holding = true;
                }
            } else {
                if (mogoClamp->motor.get_position() > target + mogoClamp->positionDeadband)
                    mogoClamp->setPower(-1);
                else if (mogoClamp->motor.get_position() < target - mogoClamp->positionDeadband)
                    mogoClamp->setPower(1);
            }
            pros::delay(10);
        }
    });

    // pros::delay(50); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly

    // wait for chassis to stop moving
    // chassis->wait();

    // reverse

    // chassis->reset();
    pros::delay(100);
    chassis->leftPower = -1, chassis->rightPower = -1;
    pros::delay(50);

    chassis->forwardDistanceController->slewEnabled = false;
    /*
    chassis->driveStraight(1.5_ft - chassis->odometry.getPose().y);
    while (chassis->odometry.getPose().y > 3.5_ft && chassis->odometry.getPose().y < 7_ft)
        pros::delay(10);
        */
    chassis->driveStraight(-5.00_ft)->wait();
    // mogoClamp->setState(MogoClamp::States::DISENGAGED);
    // arm->setHeight(0_cm);
}

void right_side_double() {
    chassis->forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.005, 100_ms)
            .getSettledUtil());
    chassis->angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.035, 0.01, 100_ms)
            .getSettledUtil());

    chassis->odometry.startTask();
    chassis->odometry.setPose({ 9_ft, 1_ft + 2_in, 0_deg });

    right_dash();
    // reverse

    // chassis->reset();

    chassis->forwardDistanceController->slewEnabled = false;
    /*
    chassis->driveStraight(1.5_ft - chassis->odometry.getPose().y);
    while (chassis->odometry.getPose().y > 3.5_ft && chassis->odometry.getPose().y < 7_ft)
        pros::delay(10);
        */
    // chassis->driveStraight(-4.20_ft)->wait();
    chassis->driveStraight(1.5_ft - chassis->odometry.getPose().y);
    while (chassis->odometry.getPose().y > 3.5_ft)
        pros::delay(10);

    mogoClamp->setState(MogoClamp::States::DISENGAGED);
    arm->setHeight(0_cm);
    chassis->forwardDistanceController->slewEnabled = true;
    // if (chassis->odometry.getPose().y > 3.5_ft)
    //     chassis->driveStraight(6_ft - chassis->odometry.getPose().y);

    chassis->wait(850);
    // pros::delay(200);

    // chassis->driveStraight(-0.5_ft)->wait(1000);
    arm->setHeight(0_cm);
    chassis->lookAt({ 6_ft, 6_ft })->wait(800);

    // TODO:
    chassis->driveDeltaDistance({ 5_ft, 5_ft });
    // chassis->driveStraight(5.3_ft);
    /*
    chassis->purePursuit(Path({
                                  { 6_ft, 6_ft } // Waypoints
                              })
                             .withLookAheadDistance(20_cm)
                             .withAngleDampen(2));

    */

    int time = pros::millis();
    WAIT_UNTIL(mogoClamp->dsense.get() <= 45 || chassis->isSettled() || (time + 2000 < pros::millis()));

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    chassis->reset(); // tell robot to stop moving

    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly
    // Can do this instead, passing in front of dropped neutral goal(hopefully)
    /*
    chassis->purePursuit(Path({
                                  { 11_ft, 2.5_ft } // Waypoints
                              })
                             .withLookAheadDistance(20_cm)
                             .withAngleDampen(2)
                             .withReversed(true))
        ->wait();
    */

    chassis->lookAt({ 9_ft, 1.5_ft }, 180_deg)->wait(1000);
    // chassis->turnAngle(-10_deg)->wait(750);
    chassis->driveStraight(-5_ft)->wait(2000);
    return;
    // return;

    tilter->setState(Tilter::States::DISENGAGED);
    chassis->lookAt({ 11_ft, 3_ft }, 180_deg)->wait(1000);
    // mogoClamp->setState(MogoClamp::States::DISENGAGED); TODO:
    pros::delay(200);
    chassis->driveStraight(-2.5_ft);

    time = pros::millis();
    WAIT_UNTIL(tilter->hasMogo() || chassis->isSettled() || (time + 1500 < pros::millis()));

    tilter->setState(Tilter::States::SCORING);
    chassis->reset();
    pros::delay(200);
    // chassis->lookAt({9_ft, 4_ft})->wait(1000);
    chassis->driveStraight(1.5_ft)->wait(1000);
    conveyor->setPower(1);
}

void auton_wp() {
    chassis->odometry.setPose({ 1_ft, 1_ft, 0_deg });
    chassis->odometry.startTask();

    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(-2_ft)->wait(1000); // grab first left side mogo
    tilter->setState(Tilter::States::SCORING);
    pros::delay(300); // make sure we have that mogo
    conveyor->setPower(1);
    chassis->turnAngle(90_deg)->wait(2000);
    tilter->setState(Tilter::States::DISENGAGED);
    chassis->driveStraight(2_ft)->wait(1500);
    tilter->setState(Tilter::States::SCORING);
    chassis->turnAngle(-90_deg)->wait(1500);
    tilter->setState(Tilter::States::DISENGAGED);
    chassis->driveStraight(-7_ft);

    int time = pros::millis();
    WAIT_UNTIL(tilter->hasMogo() || chassis->isSettled() || (time + 3500 < pros::millis()));
    tilter->setState(Tilter::States::SCORING);
    chassis->reset();
    pros::delay(300);
    conveyor->setPower(1);
    chassis->turnAngle(90_deg)->wait(1500);
    chassis->driveStraight(2.5_ft)->wait(1500);
    chassis->driveStraight(-4_ft)->wait(2000);
}

void left_side() {
    // drive to neutral goal

    /*
    auto mp = MotionProfileGenerator({
                                         1.0, // Maximum linear velocity of the Chassis in m/s
                                         2.0, // Maximum linear acceleration of the Chassis in m/s/s
                                         10.0 // Maximum linear jerk of the Chassis in m/s/s/s
                                     },
        chassis->chassisScales);

    mp.generatePath({ { 0_ft, 0_ft, 0_deg }, // Profile starting position, this will normally be (0, 0, 0)
                        { 4_ft, 1_ft, 0_deg } }, // The next point in the profile, 3 feet forward
        "A"); // Profile name)

    auto path = std::make_unique<MotionProfileGenerator::TrajectoryPair>(mp.getPath("A"));
    chassis->motionProfile(std::move(path))->wait(1500);
    */
    // chassis->driveStraight(4.5_ft);
    chassis->odometry.startTask(); // Begin odometry background task
    chassis->odometry.setPose({ 2.7_ft, 1.8_ft, 10_deg });

    /*
    // TODO: Generate this path in advance (a tiny time advantage)
    chassis->purePursuit(Path({
                                  { 1_ft, 5_ft } // Waypoints
                              })
                             .withLookAheadDistance(20_cm)
                             .withAngleDampen(2));


    arm->setHeight(0_cm);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // start in a ready position

    // wait until we get to goal
    int time = pros::millis();
    while (!mogoClamp->getNewMogo() && !chassis->isSettled() && !(time + 1500 < pros::millis()))
        pros::delay(10);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    // wait for chassis to stop moving
    chassis->reset();
    chassis->wait();
    */

    right_dash();

    chassis->forwardDistanceController->slewEnabled = false;
    chassis->driveStraight(-4.5_ft);

    chassis->wait(1500); // back to home zone
    chassis->forwardDistanceController->slewEnabled = true;
    return;

    // pick up red goal
    tilter->setState(Tilter::States::DISENGAGED);
    chassis->lookAt({ 3_ft, 1_ft })->wait(); // turn so back faces red goal
    chassis->driveDeltaDistance({ 3_ft, 1_ft }, true);
    int time = pros::millis();
    WAIT_UNTIL_TIMEOUT(tilter->hasMogo() || chassis->isSettled(), 1500);
    tilter->setState(Tilter::States::SCORING);

    while (!tilter->isScoring())
        pros::delay(10); // wait until we can score rings
    conveyor->setPower(1); // enable conveyor
    chassis->driveStraight(1_ft)->wait();
}

void profiled() {
    auto mp = MotionProfileGenerator({
                                         1.0, // Maximum linear velocity of the Chassis in m/s
                                         2.0, // Maximum linear acceleration of the Chassis in m/s/s
                                         10.0 // Maximum linear jerk of the Chassis in m/s/s/s
                                     },
        chassis->chassisScales);

    mp.generatePath({ { 0_ft, 0_ft, 0_deg }, // Profile starting position, this will normally be (0, 0, 0)
                        { 3_ft, 0_ft, 0_deg } }, // The next point in the profile, 3 feet forward
        "A"); // Profile name)

    auto path = std::make_unique<MotionProfileGenerator::TrajectoryPair>(mp.getPath("A"));
    chassis->motionProfile(std::move(path))->wait();
}
