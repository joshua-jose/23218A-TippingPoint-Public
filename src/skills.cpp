#include "main.h"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "project/AsyncTask.hpp"
#include "tomobo/MotionProfileGenerator.hpp"
#include "tomobo/pure_pursuit/Path.hpp"
#include <atomic>
#include <future>

using namespace robot;

void skills_run() {
    // --------------------------------------------------------------------------------
    // Left side

    while (chassis->inertial.is_calibrating()) {
        pros::delay(20);
    }

    chassis->odometry.setPose({ 1_ft, 1_ft, -90_deg });
    chassis->odometry.startTask();

    pros::Task([] {
        pros::GPS gps(19);
        gps.set_data_rate(5);
        gps.set_position(-(6_ft).convert(meter), -(6_ft).convert(meter), -90);
        gps.set_offset(-0.1, 0); // offset from turning centre

        double a = 0.9;
        std::uint32_t now = pros::millis();

        while (true) {
            auto gps_status = gps.get_status();
            // GPS reads from centre of field as (0,0), and down the auton line as 0 heading
            double gps_rot = gps.get_rotation() + 90;
            double gps_x = gps_status.x + (6_ft).convert(meter);
            double gps_y = gps_status.y + (6_ft).convert(meter);

            if (gps.get_error() < (1.4_in).convert(meter)) {
                // complementary filter here (state.x = 0.9 *state.x + 0.1 * gps.x)
                chassis->odometry.mutex.take();
                tomobo::Pose oldPose = chassis->odometry.getPose();

                double newx = a * oldPose.x.convert(meter) + (1 - a) * gps_x;
                double newy = a * oldPose.y.convert(meter) + (1 - a) * gps_y;
                double newtheta = a * oldPose.theta.convert(degree) + (1 - a) * gps_rot;

                tomobo::Pose newPose{ newx * meter, newy * meter, newtheta * degree };
                chassis->odometry.pose = newPose;
                chassis->odometry.mutex.give();
            }
            pros::Task::delay_until(&now, 10);
        }
    });
    /*
    pros::Task([] {
        pros::GPS gps1(19);
        pros::GPS gps2(20);
        gps1.set_data_rate(5);
        gps2.set_data_rate(5);
        gps1.set_position(-(6_ft).convert(meter), -(6_ft).convert(meter), -90);
        gps2.set_position(-(6_ft).convert(meter), -(6_ft).convert(meter), -90);
        gps1.set_offset(-0.1, 0); // offset from turning centre
        gps2.set_offset(0.1, 0); // offset from turning centre

        double a = 0.9;
        std::uint32_t now = pros::millis();

        while (true) {
            auto gps1_status = gps1.get_status();
            auto gps2_status = gps2.get_status();
            // GPS reads from centre of field as (0,0), and down the auton line as 0 heading
            double gps1_rot = gps1.get_rotation() + 90;
            double gps2_rot = gps2.get_rotation() + 90;

            double gps1_x = gps1_status.x + (6_ft).convert(meter);
            double gps1_y = gps1_status.y + (6_ft).convert(meter);
            double gps2_x = gps2_status.x + (6_ft).convert(meter);
            double gps2_y = gps2_status.y + (6_ft).convert(meter);

            double gps1_error = gps1.get_error();
            double gps2_error = gps2.get_error();

            if (gps1_error < (1.4_in).convert(meter) || gps2_error < (1.4_in).convert(meter)) {

                double gps_x = 0, gps_y = 0, gps_rot = 0;

                if (gps1_error >= (1.4_in).convert(meter)) {
                    gps_x = gps2_x;
                    gps_y = gps2_y;
                    gps_rot = gps2_rot;
                } else if (gps2_error >= (1.4_in).convert(meter)) {
                    gps_x = gps1_x;
                    gps_y = gps1_y;
                    gps_rot = gps1_rot;
                } else {
                    double sum_error = gps1_error + gps2_error;

                    // lower error has higher factor
                    double gps1_factor = gps2_error / sum_error;
                    double gps2_factor = gps1_error / sum_error;

                    gps_x = gps1_factor * gps1_x + gps2_factor * gps2_x;
                    gps_y = gps1_factor * gps1_y + gps2_factor * gps2_y;
                    gps_rot = gps1_factor * gps1_rot + gps2_factor * gps2_rot;
                }

                // complementary filter here (state.x = 0.9 *state.x + 0.1 * gps.x)
                chassis->odometry.mutex.take();
                tomobo::Pose oldPose = chassis->odometry.getPose();

                double newx = a * oldPose.x.convert(meter) + (1 - a) * gps_x;
                double newy = a * oldPose.y.convert(meter) + (1 - a) * gps_y;
                double newtheta = a * oldPose.theta.convert(degree) + (1 - a) * gps_rot;

                tomobo::Pose newPose{ newx * meter, newy * meter, newtheta * degree };
                chassis->odometry.pose = newPose;
                chassis->odometry.mutex.give();
            }
            pros::Task::delay_until(&now, 10);
        }
    });
    */
    AsyncTask convTask([&] {
        while (true) {
            if (tilter->state == Tilter::States::SCORING)
                conveyor->state = Conveyor::States::INTAKING;
            else
                conveyor->state = Conveyor::States::IDLE;
            conveyor->oc_update();
            pros::delay(20);
        }
    });

    // use more aggresive settling parameters
    chassis->forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.01, 50_ms)
            .getSettledUtil());
    chassis->angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.035, 0.01, 50_ms)
            .getSettledUtil());

    int time = 0;

    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(-2_ft); // grab first left side mogo

    time = pros::millis();
    WAIT_UNTIL(tilter->hasMogo() || chassis->isSettled() || (time + 1000 < pros::millis()));
    chassis->reset();
    tilter->setState(Tilter::States::SCORING);
    pros::delay(300); // make sure we have that mogo
    chassis->driveStraight(0.75_ft)->wait(800);
    chassis->setHeading(-90_deg + 22_deg)->wait(500);
    chassis->driveStraight(1_ft)->wait(500);
    /*
    chassis->purePursuit(Path({ { 1_ft, 1.75_ft } })
                             .withLookAheadDistance(20_cm)
                             .withAngleDampen(2))
        ->wait(1500);
    */
    // chassis->turnAngle(90_deg)->wait(2000);
    chassis->lookAt({ 3_ft, 6_ft })->wait(1000);

    /*
    chassis->purePursuit(Path({
                                  { 3_ft, 6_ft } // Waypoints
                              })
                             .withLookAheadDistance(20_cm)
                             .withAngleDampen(2));
    */
    chassis->driveStraight(5_ft);
    time = pros::millis();
    while (!mogoClamp->autonGetNewMogo() && !chassis->isSettled() && !(time + 2000 < pros::millis()))
        pros::delay(10);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(41_cm); // nudge arm up slightly

    // chassis->wait(1000);
    chassis->reset();
    pros::delay(500);
    // chassis->setHeading(30_deg)->wait(1000); // in the middle, pointing at red bridge
    chassis->lookAt({ 6_ft, 8_ft })->wait(1500);
    // chassis->driveDeltaDistance({ 6_ft, 8_ft })->wait(1000); TODO:
    chassis->driveStraight(3_ft)->wait(1500);
    chassis->setHeading(0_deg)->wait(1000);
    chassis->driveStraight(1.4_ft)->wait(750);
    arm->setHeight(25_cm); // tip bridge
    pros::delay(400);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral on bridge
    arm->setHeight(41_cm);
    pros::delay(400);

    chassis->driveStraight(-1.2_ft)->wait(1500);
    chassis->setHeading(90_deg + 27_deg)->wait(1500);
    // chassis->lookAt({0_ft, 12_ft}, 180_deg)->wait(1500);

    arm->setHeight(0_cm);
    tilter->setState(Tilter::States::DISENGAGED);
    chassis->driveStraight(-4.3_ft)->wait(1500);
    chassis->driveStraight(1.8_ft)->wait(1000);
    chassis->lookAt({ 1_ft, 9_ft }, 180_deg)->wait(1000); // turn so back faces nearby blue mogo
    chassis->driveStraight(-3.5_ft);

    time = pros::millis();
    WAIT_UNTIL(tilter->hasMogo() || chassis->isSettled() || (time + 1500 < pros::millis()));
    tilter->setState(Tilter::States::SCORING); // pick up mogo
    chassis->reset();
    pros::delay(400);
    chassis->driveStraight(6_in)->wait(750);
    chassis->lookAt({ 6_ft, 6_ft })->wait(1500);

    chassis->driveStraight(7_ft); // drive across field to bridge

    time = pros::millis();
    while (!mogoClamp->autonGetNewMogo() && !chassis->isSettled() && ((pros::millis() - time < 2500)))
        pros::delay(10);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly

    // pros::delay(200);
    chassis->wait(1500);
    // chassis->turnAngle(20_deg)->wait(500);
    chassis->lookAt({ 9.7_ft, 1_ft })->wait(1000);
    /*
    chassis->driveStraight(3.5_ft)->wait(1500);

    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral

    chassis->driveStraight(-1_ft)->wait(750);
    chassis->setHeading(90_deg)->wait(750);

    tilter->setState(Tilter::States::DISENGAGED); // drop mogo as we approach bridge
    chassis->driveStraight(-2_ft)->wait(1000);
    chassis->driveStraight(1.5_ft)->wait(1000);
    */
    ///*
    chassis->driveStraight(3.5_ft);
    pros::delay(800);
    tilter->setState(Tilter::States::DISENGAGED);
    chassis->wait(750);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral
    tilter->setState(Tilter::States::STOWED);
    pros::delay(300);
    arm->setHeight(0_cm);
    chassis->turnAngle(180_deg)->wait(2000);
    chassis->driveStraight(2.5_ft);

    time = pros::millis();
    while (!mogoClamp->autonGetNewMogo() && !chassis->isSettled() && ((pros::millis() - time < 2500)))
        pros::delay(10);
    mogoClamp->setState(MogoClamp::States::ENGAGED); // grab blue mogo that was on back
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(40_cm); // nudge arm up slightly
    chassis->wait(1500);
    // chassis->driveStraight(1.5_ft)->wait(1500);
    chassis->lookAt({ 6_ft, 1_ft })->wait(1000);
    chassis->driveStraight(3.2_ft)->wait(1500);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral
    chassis->driveStraight(-1_ft)->wait(500);
    arm->setHeight(0_cm);

    // // replace below
    //*/

    arm->setHeight(0_cm);
    // tilter->setState(Tilter::States::SCORING);
    chassis->lookAt({ 11_ft, 3_ft }, 180_deg)->wait(1500);
    tilter->setState(Tilter::States::DISENGAGED);
    // chassis->driveStraight(-2_ft);
    chassis->driveStraight(-5_ft);
    time = pros::millis();
    WAIT_UNTIL(tilter->hasMogo() || chassis->isSettled() || (time + 1000 < pros::millis()));
    chassis->reset();
    // chassis->driveStraight(-4.5_ft)->wait(3000);
    tilter->setState(Tilter::States::SCORING);
    pros::delay(400);
    chassis->driveStraight(9_in)->wait(750);
    chassis->lookAt({ 9_ft, 6_ft })->wait(1000);

    chassis->driveStraight(5_ft); // drive across field to bridge
    // chassis->driveDeltaDistance({9_ft, 6_ft}); TODO:

    time = pros::millis();
    while (!mogoClamp->autonGetNewMogo() && !chassis->isSettled() && ((pros::millis() - time < 2500)))
        pros::delay(10);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(40_cm); // nudge arm up slightly
    chassis->wait(1000);
    // chassis->turnAngle(-5_deg)->wait(500);
    chassis->lookAt({ 6_ft, 11_ft })->wait(1000);
    chassis->driveStraight(2.8_ft)->wait(1750);

    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral on bridge
    pros::delay(400);
    chassis->driveStraight(-6_in)->wait(750);
    chassis->setHeading(-90_deg)->wait(1000);
    arm->setHeight(0_cm);
    chassis->driveStraight(-3_ft)->wait(1500);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(400);
    chassis->driveStraight(1_ft)->wait(750);
    chassis->lookAt({ 8_ft, 11_ft }, 180_deg)->wait(1000);
    chassis->driveStraight(-2_ft);
    time = pros::millis();
    WAIT_UNTIL(tilter->hasMogo() || chassis->isSettled() || (time + 1200 < pros::millis()));
    tilter->setState(Tilter::States::SCORING);
    chassis->reset();
    pros::delay(400);
    ///*
    chassis->purePursuit(Path({ { 10_ft, 6_ft }, // Waypoints
                                  { 10_ft, 2_ft } })
                             .withLookAheadDistance(30_cm)
                             .withAngleDampen(2))
        ->wait();
    //*/
    /*
    chassis->driveStraight(1_ft)->wait(500);
    chassis->lookAt({ 6_ft, 4_ft })->wait(1000);
    chassis->driveDeltaDistance({ 6_ft, 4_ft })->wait(3000);
    chassis->setHeading(180_deg)->wait(500);
    tilter->setState(Tilter::States::DISENGAGED);
    chassis->driveStraight(1_ft)->wait(500);
    tilter->setState(Tilter::States::STOWED);
    chassis->turnAngle(180_deg)->wait(1000);
    chassis->driveStraight(1.5_ft);

    while (!mogoClamp->autonGetNewMogo() && !chassis->isSettled())
        pros::delay(10);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(40_cm); // nudge arm up slightly
    chassis->turnAngle(180_deg)->wait(1000);
    chassis->driveStraight(1.5_ft)->wait(1000);
    mogoClamp->setState(MogoClamp::States::DISENGAGED);
    chassis->driveStraight(-1.5_ft)->wait(1000);
    */

    /* // to other side of field
    chassis->purePursuit(Path({ { 6_ft, 4_ft } // Waypoints
                                   })
                             .withLookAheadDistance(20_cm)
                             .withAngleDampen(2))
        ->wait();
        */
    convTask.cancel();
    chassis->odometry.stopTask();
}
/*
void skills_260pt() {
    while (chassis->inertial.is_calibrating()) {
        pros::delay(20);
    }

    chassis->odometry.setPose({ 1_ft, 1_ft, -90_deg });
    chassis->odometry.startTask();

    AsyncTask convTask([&] {
        while (true) {
            if (tilter->state == Tilter::States::SCORING)
                conveyor->state = Conveyor::States::INTAKING;
            else
                conveyor->state = Conveyor::States::IDLE;
            conveyor->oc_update();
            pros::delay(20);
        }
    });

    // use more aggresive settling parameters
    chassis->forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.01, 50_ms)
            .getSettledUtil());
    chassis->angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.035, 0.01, 50_ms)
            .getSettledUtil());

    int time = 0;

    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(-2_ft); // grab first left side mogo

    WAIT_UNTIL_TIMEOUT(tilter->hasMogo() || chassis->isSettled(), 1000);
    chassis->reset();
    tilter->setState(Tilter::States::SCORING);
    pros::delay(300); // make sure we have that mogo
    chassis->driveStraight(1_ft)->wait(800);

    chassis->lookAt({ 3_ft, 6_ft })->wait(1000);

    chassis->driveDeltaDistance({ 3_ft, 6_ft });
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 2000);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(41_cm); // nudge arm up slightly

    chassis->wait(500);
    // chassis->setHeading(30_deg)->wait(1000); // in the middle, pointing at red bridge
    chassis->lookAt({ 5.5_ft, 8_ft })->wait(1500);
    // chassis->driveDeltaDistance({ 6_ft, 8_ft })->wait(1000); TODO:
    chassis->driveDeltaDistance({ 5.5_ft, 8_ft })->wait(1500);
    chassis->setHeading(0_deg)->wait(1000);
    chassis->driveStraight(1.5_ft)->wait(750);
    arm->setHeight(25_cm); // tip bridge
    pros::delay(400);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral on bridge
    arm->setHeight(41_cm);
    pros::delay(400);

    // score back mogo
    chassis->driveStraight(-1.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::STOWED);
    pros::delay(500);
    chassis->driveStraight(-0.8_ft)->wait(750);
    tilter->setState(Tilter::States::LIFTING);
    pros::delay(500);
    chassis->turnAngle(180_deg)->wait(1000);
    chassis->driveStraight(-1.5_ft)->wait(750);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);

    chassis->lookAt({ 1_ft, 9_ft }, 180_deg)->wait(1000);
    chassis->driveDeltaDistance({ 1_ft, 9_ft });
    WAIT_UNTIL_TIMEOUT(tilter->hasMogo() || chassis->isSettled(), 2000);
    chassis->reset();
    tilter->setState(Tilter::States::SCORING);
    pros::delay(300);

    // pure pursuit would be nice here
    chassis->driveStraight(0.5_ft)->wait(500); // adjust this to adjust angle across field
    chassis->lookAt({ 6_ft, 6_ft })->wait(500);
    chassis->driveStraight(8_ft); // drive across field to bridge

    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 2500);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly

    // pros::delay(200);
    chassis->wait(1500);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral

    chassis->driveStraight(-1_ft)->wait(1000);
    chassis->lookAt({ 9_ft, 6_ft })->wait(1000);
    chassis->driveDeltaDistance({ 9_ft, 6_ft });
    arm->setHeight(0_cm);
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 1500);
    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly

    chassis->lookAt({ 6_ft, 4.5_ft })->wait(1000);
    chassis->driveDeltaDistance({ 6_ft, 4.5_ft })->wait(1000);
    chassis->setHeading(180_deg)->wait(1000);
    chassis->driveStraight(1.5_ft)->wait(750);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral on bridge
    arm->setHeight(41_cm);
    pros::delay(400);

    chassis->driveStraight(-1.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::STOWED);
    pros::delay(500);
    chassis->driveStraight(-0.8_ft)->wait(750);
    tilter->setState(Tilter::States::LIFTING);
    pros::delay(500);
    chassis->turnAngle(180_deg)->wait(1000);
    chassis->driveStraight(-1.5_ft)->wait(750);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);

    chassis->lookAt({ 11_ft, 3_ft })->wait(1500);
    chassis->driveDeltaDistance({ 11_ft, 3_ft });
    arm->setHeight(0_cm);
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 1500);
    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly

    chassis->lookAt({ 6_ft, 11_ft })->wait(1000);
    arm->setHeight(41_cm);
    chassis->driveDeltaDistance({ 6_ft, 11_ft })->wait(3000);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral on bridge
    pros::delay(300);

    chassis->driveStraight(-4_ft)->wait(1000);
    arm->setHeight(0_cm);
    chassis->lookAt({ 8_ft, 11_ft })->wait(1000);
    chassis->driveDeltaDistance({ 8_ft, 11_ft });
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 2000);
    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly
    chassis->driveStraight(-2_ft)->wait(500);

    chassis->lookAt({ 6_ft, 1_ft })->wait(1000);
    arm->setHeight(41_cm);
    chassis->driveDeltaDistance({ 6_ft, 1_ft })->wait(3000);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral on bridge
    pros::delay(300);
    chassis->driveStraight(-2_ft)->wait(1000);
}

void skills_310pt() {
    while (chassis->inertial.is_calibrating()) {
        pros::delay(20);
    }

    chassis->odometry.setPose({ 1_ft, 1_ft, -90_deg });
    chassis->odometry.startTask();

    AsyncTask convTask([&] {
        while (true) {
            if (tilter->state == Tilter::States::SCORING)
                conveyor->state = Conveyor::States::INTAKING;
            else
                conveyor->state = Conveyor::States::IDLE;
            conveyor->oc_update();
            pros::delay(20);
        }
    });

    // use more aggresive settling parameters
    chassis->forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.01, 50_ms)
            .getSettledUtil());
    chassis->angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.035, 0.01, 50_ms)
            .getSettledUtil());

    int time = 0;

    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(-2_ft); // grab first left side mogo

    WAIT_UNTIL_TIMEOUT(tilter->hasMogo() || chassis->isSettled(), 1000);
    chassis->reset();
    tilter->setState(Tilter::States::SCORING);
    pros::delay(300); // make sure we have that mogo
    chassis->driveStraight(1_ft)->wait(800);

    chassis->lookAt({ 3_ft, 6_ft })->wait(1000);

    chassis->driveDeltaDistance({ 3_ft, 6_ft });
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 2000);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(41_cm); // nudge arm up slightly

    chassis->wait(500);
    // chassis->setHeading(30_deg)->wait(1000); // in the middle, pointing at red bridge
    chassis->lookAt({ 5.5_ft, 8_ft })->wait(1500);
    // chassis->driveDeltaDistance({ 6_ft, 8_ft })->wait(1000); TODO:
    chassis->driveDeltaDistance({ 5.5_ft, 8_ft })->wait(1500);
    chassis->setHeading(0_deg)->wait(1000);
    chassis->driveStraight(1.5_ft)->wait(750);
    arm->setHeight(25_cm); // tip bridge
    pros::delay(400);
    mogoClamp->setState(MogoClamp::States::DISENGAGED); // drop neutral on bridge
    arm->setHeight(41_cm);
    pros::delay(400);

    // score back mogo
    chassis->driveStraight(-1.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::STOWED);
    pros::delay(500);
    chassis->driveStraight(-0.8_ft)->wait(750);
    tilter->setState(Tilter::States::LIFTING);
    pros::delay(500);
    chassis->turnAngle(180_deg)->wait(1000);
    chassis->driveStraight(-1.5_ft)->wait(750);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);

    chassis->lookAt({ 9_ft, 6_ft })->wait(1000);
    chassis->driveDeltaDistance({ 9_ft, 6_ft });
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 2000);

    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly
    chassis->reset();

    chassis->lookAt({ 11_ft, 3_ft }, 180_deg)->wait(1000);
    chassis->driveDeltaDistance({ 11_ft, 3_ft });
    WAIT_UNTIL_TIMEOUT(tilter->hasMogo() || chassis->isSettled(), 2000);
    chassis->reset();
    tilter->setState(Tilter::States::SCORING);
    pros::delay(300); // make sure we have that mogo

    chassis->lookAt({ 6.5_ft, 11_ft })->wait(1000);
    chassis->driveDeltaDistance({ 6.5_ft, 10_ft })->wait(2500);
    mogoClamp->setState(MogoClamp::States::DISENGAGED);

    chassis->driveStraight(-1.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::STOWED);
    pros::delay(500);
    chassis->driveStraight(-0.8_ft)->wait(750);
    tilter->setState(Tilter::States::LIFTING);
    pros::delay(500);
    chassis->turnAngle(180_deg)->wait(1000);
    chassis->driveStraight(-1.5_ft)->wait(750);
    chassis->driveStraight(0.5_ft)->wait(750);
    tilter->setState(Tilter::States::DISENGAGED);
    pros::delay(500);

    chassis->lookAt({ 6_ft, 6_ft })->wait(1000);
    chassis->driveDeltaDistance({ 6_ft, 6_ft });
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 2000);
    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly
    chassis->reset();

    chassis->lookAt({ 3_ft, 11_ft }, 180_deg)->wait(1000);
    chassis->driveDeltaDistance({ 3_ft, 11_ft }, true)->wait(1000);
    arm->setHeight(41_cm);
    chassis->setHeading(90_deg)->wait(1000);
    chassis->driveDeltaDistance({ 3.5_ft, 11_ft })->wait(1000);
    arm->setHeight(25_cm);
    pros::delay(750);
    mogoClamp->setState(MogoClamp::States::DISENGAGED);
    pros::delay(500);
    chassis->driveStraight(-1_ft)->wait(500);
    arm->setHeight(0_cm);

    chassis->lookAt({ 1_ft, 9_ft }, 180_deg)->wait(1000);
    chassis->driveDeltaDistance({ 1_ft, 9_ft });
    WAIT_UNTIL_TIMEOUT(tilter->hasMogo() || chassis->isSettled(), 2000);
    chassis->reset();
    tilter->setState(Tilter::States::SCORING);
    pros::delay(300); // make sure we have that mogo

    chassis->lookAt({ 8_ft, 8_ft })->wait(1000);
    chassis->driveDeltaDistance({ 8_ft, 8_ft })->wait(1000);
    chassis->lookAt({ 8_ft, 11_ft })->wait(1000);
    chassis->driveDeltaDistance({ 8_ft, 11_ft });
    WAIT_UNTIL_TIMEOUT(mogoClamp->autonGetNewMogo() || chassis->isSettled(), 1500);
    mogoClamp->setState(MogoClamp::States::ENGAGED);
    pros::delay(400); // wait for clamp to settle
    arm->setHeight(4_cm); // nudge arm up slightly
    chassis->reset();

    chassis->lookAt({ 9_ft, 1_ft }, 180_deg)->wait(1000);
    chassis->driveDeltaDistance({ 9_ft, 1_ft }, true)->wait(1000);
    chassis->setHeading(-90_deg)->wait(1000);
    // climb....
}
*/