#include "main.h"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "project/gui.hpp"
#include "tomobo/MotionProfileGenerator.hpp"
#include "tomobo/pure_pursuit/Path.hpp"
#include <atomic>
#include <future>

using namespace robot;
// std::atomic_bool tare_done{ false };
// std::shared_ptr<AsyncTask> tareTask;

// ------------------------------------------------------------------------------------------------------------------------------

void skills_run();
void timed_right_side();
void single_dash();
void right_side_double();
void left_side();
void skills_run();
void profiled();
void middle_goal();
void right_single_wp();

void auton_init() {

    // configManager->register_auton("right timed", timed_right_side);
    configManager->register_auton("single dash", single_dash);
    configManager->register_auton("right double", right_side_double);
    configManager->register_auton("right wp", right_single_wp);
    configManager->register_auton("left  single", left_side);
    configManager->register_auton("middle goal", middle_goal);
    configManager->register_auton("skills", skills_run);
}

void autonomous() {
    std::shared_ptr<GUI> gui = GUI::get();
    gui->set_splash_hidden(false);

    chassis->inertial.tare_rotation();
    chassis->startTask(); // Begin drive controller background task
    chassis->odometry.startTask();

    /*
    chassis->driveStraight(4_ft)->wait(2000);
    chassis->setHeading(180_deg)->wait(1000);
    // printf("main odom: %s\n", chassis->odometry.getPose().str().c_str());
    chassis->driveStraight(4_ft)->wait(2000);
    chassis->turnAngle(180_deg)->wait(1000);
    */
    // return;
    //    printf("main odom: %s\n", chassis->odometry.getPose().str().c_str());

    // right_side_double();

    if (configManager->auton_routines.size() > configManager->selected_auton) {
        auton_func routine = configManager->get_auton_func(configManager->selected_auton);

        routine(); // nullptr could happen, lets hope it doesn't :o
    } else {
        printf("Selected auton is greater than amount of autons");
    }

    /*
     chassis->turnAngle(180_deg)->wait();
     controller->print(1, 0, "%f", chassis->odometry.getPose().theta.convert(okapi::degree));
     while (true) {
         printf("main odom: %s\n", chassis->odometry.getPose().str().c_str());
         pros::delay(750);
     }
     */

    // chassis->driveStraight(2_ft)->wait();
    // controller->print(1, 0, "%f", chassis->odometry.getPose().y.convert(okapi::meter));

    chassis->odometry.stopTask();
    chassis->stopTask();

    /*
    if (configManager->auton_routines.size() > configManager->selected_auton) {
        auton_func routine = configManager->get_auton_func(configManager->selected_auton);

        routine(); // nullptr could happen, lets hope it doesn't :o
    } else {
        printf("Selected auton is greater than amount of autons");
    }
    */

    // tareTask = std::make_unique<AsyncTask>([] {
    //  TODO:
    /*
        AsyncTask clampTask([] { mogoClamp->hardTare(); });
        AsyncTask tilterTask([] { tilter->hardTare(); });
        AsyncTask armTask([] { arm->hardTare(); });

        clampTask.wait();
        tilterTask.wait();
        armTask.wait();
        */
    //});
    // mogoClamp->motor.tare_position();
    // tilter->motor.tare_position();
    // arm->motor.tare_position();

    // timed_right_side();
    // return;

    // right_side();
    // left_side();

    // chassis->odometry.setPose({ 1_ft, 1_ft, -90_deg });

    /*
    chassis->odometry.setPose({ 1_ft, 1_ft, 90_deg });
    printf("%s\n", robot::chassis->odometry.getPose().str().c_str());
    chassis->turnAngle(90_deg)->wait(1000);
    printf("%s\n", robot::chassis->odometry.getPose().str().c_str());
    */

    // chassis->purePursuit(Path({
    //                               { 3_ft, 6_ft } // Waypoints
    //                           })
    //                          .withLookAheadDistance(20_cm)
    //                          .withAngleDampen(2.2))
    //     ->wait(3000);
    /*
    while (true) {
        printf("%s\n", robot::chassis->odometry.getPose().str().c_str());
        printf("left: %d\n", robot::chassis->tracking_left.get_value());
        printf("right: %d\n", robot::chassis->tracking_right.get_value());
        printf("mid: %d\n", robot::chassis->tracking_mid.get_value());
        printf("\n");
        pros::delay(500);
    }
    */

    // skills_run();

    // chassis->odometry.stopTask();

    // chassis->turnAngle(180_deg)->wait();

    // chassis->odometry.setPose({ 3_ft, 3_ft, 0_deg });
}
