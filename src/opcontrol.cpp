#include "main.h"
#include "okapi/api.hpp"
#include "project/gui.hpp"
#include "tomobo/controller/PIDController.hpp"

#include "tomobo/odometry/InertialOdometry.hpp"

using namespace okapi::literals;
using namespace robot;

/*
-- make sure mogo clamp actually stops at 0 vel
-- timeouts for abs movements
-- limit switches -> auto mogo pick up
implement killable tasks
+ tune PIDs
+ reduce chassis slew
test auton movements
motion profiling
write/implement/tune KVA controllers
+ see if there's a way to make chassis more controllable (cheesy drive?)

+ constant time stow
add GUI
logging (to file and logging errors to screen)
if program ends during match, store positions to file, then load them on restart.
*/

void start_killable_task(std::function<void()> func, pros::controller_digital_e_t button) {
    std::atomic_bool done{ false };

    pros::Task task([&func, &done] {
        func();
        done.store(true);
    },
        TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "KILLABLE_TASK");

    while (controller->get_digital(button) && !done.load())
        pros::delay(10);
    task.remove();
}

void stow() {
    mogoClamp->setState(MogoClamp::States::STOWED);
    tilter->setState(Tilter::States::STOWED);
    arm->setHeight(arm->min_height);
}

void ct_stow() {
    std::valarray<float> times = { 0, 0, 0 };
    std::valarray<float> speeds = { 0, 0, 0 };
    double speed = 0.9;

    // t = d/s
    // t = rotations / rps
    // (degrees/360) /(60*rpm) = (degrees/360)*(1/60*rpm) = degrees/(60*rpm)
    times[0] = mogoClamp->motor.get_position() / (60 * mogoClamp->max_rpm * speed);
    times[1] = tilter->motor.get_position() / (60 * tilter->max_rpm * speed);
    times[2] = arm->motor.get_position() / (60 * arm->max_rpm * speed);

    float slowest_time = times.max();

    // s = d/t
    speeds[0] = mogoClamp->motor.get_position() / (60 * slowest_time);
    speeds[1] = tilter->motor.get_position() / (60 * slowest_time);
    speeds[2] = arm->motor.get_position() / (60 * slowest_time);

    mogoClamp->motor.move_absolute(0, speeds[0]);
    tilter->motor.move_absolute(0, speeds[1]);
    arm->motor.move_absolute(0, speeds[2]);

    mogoClamp->state = MogoClamp::States::STOWED;
    tilter->state = Tilter::States::STOWED;
}

void pid_tuning();

void opcontrol() {
    printf("Entering opcontrol\n");

    /*
    while (true) {
        mogoClamp->updateDistance();
        pros::delay(10);
    }
    */

    // pid_tuning();
    //     return;

    // arm->setHeight(0_cm);
    // pros::delay(1000);

    // Just in case
    chassis->odometry.stopTask();
    chassis->stopTask();
    mogoClamp->stopTask();

    pros::Task([] {
        pros::delay(500);
        while (chassis->inertial.is_calibrating()) {
            pros::delay(20);
        }
        chassis->odometry.startTask(); // Begin odometry background task
        tomobo::EncoderOdometry testodom(chassis->trackingScales, chassis->tracking_left, chassis->tracking_right, chassis->tracking_mid);
        testodom.startTask();

        chassis->odometry.setPose({ 0_m, 0_m, 0_deg });
        testodom.setPose({ 0_m, 0_m, 0_deg });

        while (true) {
            tomobo::Pose pose = chassis->odometry.getPose();
            double x = pose.x.convert(foot);
            double y = pose.y.convert(foot);
            double theta = pose.theta.convert(radian);
            printf("o:%f,%f,%f\n", x, y, theta);

            // printf("main odom: %s\n", chassis->odometry.getPose().str().c_str());
            // printf("three wheel odom: %s\n", testodom.getPose().str().c_str());
            printf("left: %d\n", chassis->tracking_left.get_value());
            printf("right: %d\n", chassis->tracking_right.get_value());
            printf("mid: %d\n", chassis->tracking_mid.get_value());
            printf("\n");

            pros::delay(750);
        }
    });

    /*
    pros::Task([] {
        while (true) {
            int distance = tilter->ultrasonic.get_value();
            printf("u: %d\n", distance);
            pros::delay(750);
        }
    });
    */

    // make sure things are repositioned after motors are disabled
    mogoClamp->updatePosition();
    tilter->updatePosition();

    bool manual_mode = false;
    bool brake_mode = false;
    bool tilterIsScoringLast = tilter->isScoring();

    bool conveyorOverride = false;
    Conveyor::States overrideDesiredState = Conveyor::States::IDLE;

    robot::controller->print(1, 1, "Man: no");
    std::shared_ptr<GUI> gui = GUI::get();
    while (true) {
        double chassis_temp = (chassis->fl_motor.get_temperature() + chassis->fr_motor.get_temperature() + chassis->bl_motor.get_temperature() + chassis->br_motor.get_temperature()) / 4;
        double claw_temp = (mogoClamp->motor.get_temperature());
        double arm_temp = (arm->motor.get_temperature());

        gui->update_inertial_status(robot::chassis->inertial);
        lv_gauge_set_value(gui->chassis_temp_guage, 0, chassis_temp);
        lv_gauge_set_value(gui->arm_temp_guage, 0, arm_temp);
        lv_gauge_set_value(gui->claw_temp_guage, 0, claw_temp);

        // Inputs
        int32_t joy_y = controller->get_analog(ANALOG_LEFT_Y);
        int32_t joy_a = controller->get_analog(ANALOG_LEFT_X);

        int32_t arm_power = controller->get_analog(ANALOG_RIGHT_Y);

        bool toggle_intake = controller->get_digital_new_press(DIGITAL_UP);
        bool reverse_intake = controller->get_digital_new_press(DIGITAL_DOWN);
        bool toggle_manual = controller->get_digital_new_press(DIGITAL_X);
        bool tare_clamp = controller->get_digital_new_press(DIGITAL_B);
        // bool should_stow = controller->get_digital_new_press(DIGITAL_DOWN);
        bool toggle_brake = controller->get_digital_new_press(DIGITAL_LEFT);
        // bool tare_tilter = controller->get_digital_new_press(DIGITAL_Y);
        //  bool tare_arm = controller->get_digital_new_press(DIGITAL_A);
        // bool stow_tiler = controller->get_digital_new_press(DIGITAL_LEFT);

        // user control logic

        if (toggle_manual) {
            manual_mode = !manual_mode;
            if (manual_mode)
                robot::controller->print(1, 1, "Man: yes");
            else
                robot::controller->print(1, 1, "Man: no");
        }
        if (toggle_brake) {
            brake_mode = !brake_mode;
            if (brake_mode)
                robot::controller->print(1, 9, "B: y");
            else
                robot::controller->print(1, 9, "B: n");
        }
        if (manual_mode) {
            bool clamp_up = controller->get_digital(DIGITAL_R1);
            bool clamp_down = controller->get_digital(DIGITAL_R2);
            bool tilter_up = controller->get_digital(DIGITAL_L1);
            bool tilter_down = controller->get_digital(DIGITAL_L2);
            // manual control of the clamp
            mogoClamp->motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            if (clamp_up)
                mogoClamp->setPower(-1);
            else if (clamp_down)
                mogoClamp->setPower(1);
            else
                mogoClamp->setPower(0);

            if (tilter_up)
                tilter->setPower(-1);
            else if (tilter_down)
                tilter->setPower(1);
            else
                tilter->setPower(0);

            if (toggle_intake)
                conveyor->toggleState();

            arm->setVelocity(arm_power / 127.0);

        } else {
            bool clamp_up = controller->get_digital_new_press(DIGITAL_R1);
            bool clamp_down = controller->get_digital_new_press(DIGITAL_R2);
            bool tilter_up = controller->get_digital_new_press(DIGITAL_L1);
            bool tilter_down = controller->get_digital_new_press(DIGITAL_L2);

            if (tare_clamp)
                mogoClamp->hardTare();
            // pros::Task([&]{mogoClamp->tare();});
            // start_killable_task([] { mogoClamp->hardTare(); }, DIGITAL_B);
            // mogoClamp->setPower(0);
            // if (tare_tilter && !tilter->button.get_value())
            //    tilter->hardTare();
            // if (tare_arm)
            //     arm->hardTare();

            // if (should_stow)
            //     stow();

            if (clamp_up)
                mogoClamp->setState(MogoClamp::States::DISENGAGED);
            if (clamp_down)
                mogoClamp->setState(MogoClamp::States::ENGAGED);

            mogoClamp->ocUpdate();

            //  arm->setHeight(arm->min_height+2_in);

            if (tilter_down) {
                if (tilter->state == Tilter::States::LIFTING)
                    tilter->setState(Tilter::States::DISENGAGED);
                else if (tilter->state == Tilter::States::DISENGAGED)
                    tilter->setState(Tilter::States::SCORING);
                else if (tilter->state == Tilter::States::SCORING)
                    tilter->setState(Tilter::States::STOWED);
            }

            else if (tilter_up) {
                if (tilter->state == Tilter::States::STOWED)
                    tilter->setState(Tilter::States::DISENGAGED);
                else if (tilter->state == Tilter::States::SCORING)
                    tilter->setState(Tilter::States::DISENGAGED);
                else if (tilter->state == Tilter::States::DISENGAGED)
                    tilter->setState(Tilter::States::LIFTING);

                conveyor->state = Conveyor::States::IDLE;
            }
            tilter->ocUpdate();

            // if (!tilter->isScoring() && isScoringLast)
            //     conveyor->state = Conveyor::States::IDLE;

            //  arm min height feature
            /*
            if (!armIsLowest && arm->getHeight() <= 0_cm) {
                arm->setHeight(0_cm);
                armIsLowest = true;
            }
            if (arm->getHeight() <= 0_cm && arm_power < 0) {
                arm_power = 0;
            } else {
                arm->setVelocity(arm_power / 127.0);
                armIsLowest = false;
            }
            */
            arm->setVelocity(arm_power / 127.0);
        }
        // if (stow_tiler)
        //     tilter->setState(Tilter::States::STOWED);

        if (toggle_intake || reverse_intake) {
            conveyorOverride = true;
            if (conveyor->state != Conveyor::States::IDLE)
                overrideDesiredState = Conveyor::States::IDLE;
            else if (conveyor->state != Conveyor::States::INTAKING)
                overrideDesiredState = Conveyor::States::INTAKING;

            if (reverse_intake)
                overrideDesiredState = Conveyor::States::REVERSE;
        }
        if (!conveyorOverride) {
            if (tilter->isScoring() && arm->getHeight() > 1_cm)
                conveyor->state = Conveyor::States::INTAKING;

            else
                conveyor->state = Conveyor::States::IDLE;
        } else {
            conveyor->state = overrideDesiredState;
        }

        if (tilter->isScoring() != tilterIsScoringLast)
            conveyorOverride = false;

        tilterIsScoringLast = tilter->isScoring();

        // deadband
        if (abs(arm_power) < 15)
            arm_power = 0;

        conveyor->oc_update();
        chassis->oc_update(joy_y, joy_a, brake_mode);

        // pros::lcd::print(2, "arm rot: %f", arm->motor.get_position());

        pros::delay(10);
    }
}

void graphing() {
    int last_time = pros::millis();

    while (true) {
        if (chassis->getState() == "IDLE")
            continue;
        // double error = chassis->forwardDistanceController->getError();
        double error = chassis->angleController->getError();
        printf("graph:%d,%f,%f\n", pros::millis(), error, chassis->leftPower.load());
        pros::delay(10);
    }
}

void pid_tuning() {
    double scale = 0.1;
    pros::Task graphing_task(graphing); // start graphing task
    chassis->startTask();
    chassis->odometry.startTask();
    //  change to controller we're tuning
    //  dirty raw cast but this is easiest
    // tomobo::FeedbackController* c1 = chassis->forwardDistanceController.get();
    tomobo::FeedbackController* c1 = chassis->angleController.get();
    PIDController* controller = (PIDController*)c1;

    // Print numbers
    robot::controller->clear();
    pros::delay(50);
    PIDController::Gains gains = controller->getGains();
    robot::controller->print(0, 0, "Scale: %f", scale);
    pros::delay(50);
    robot::controller->print(1, 0, "kP: %f", gains.kP);
    pros::delay(50);
    robot::controller->print(2, 0, "kD: %f", gains.kD);
    pros::delay(50);

    int direction = 1;
    while (true) {
        gains = controller->getGains();
        PIDController::Gains newgains = gains;

        bool run = robot::controller->get_digital_new_press(DIGITAL_Y);
        bool stop = robot::controller->get_digital_new_press(DIGITAL_X);
        bool kP_up = robot::controller->get_digital_new_press(DIGITAL_L1);
        bool kP_down = robot::controller->get_digital_new_press(DIGITAL_L2);
        bool kD_up = robot::controller->get_digital_new_press(DIGITAL_R1);
        bool kD_down = robot::controller->get_digital_new_press(DIGITAL_R2);
        bool scale_up = robot::controller->get_digital_new_press(DIGITAL_UP);
        bool scale_down = robot::controller->get_digital_new_press(DIGITAL_DOWN);

        if (run) {
            printf("run\n");
            // Change this for each type
            // chassis->driveStraight(direction * 4_ft);
            chassis->turnAngle(direction * 180_deg);
            direction *= -1;
        }

        if (stop) {
            chassis->reset();
            printf("main odom: %s\n", chassis->odometry.getPose().str().c_str());
        }
        if (scale_up)
            scale *= 2;
        if (scale_down)
            scale /= 2;
        if (kP_up)
            newgains.kP += scale;
        if (kP_down)
            newgains.kP -= scale;
        if (kD_up)
            newgains.kD += scale;
        if (kD_down)
            newgains.kD -= scale;

        if (scale_up || scale_down)
            robot::controller->print(0, 0, "Scale: %f", scale);
        if (kP_up || kP_down)
            robot::controller->print(1, 0, "kP: %f", newgains.kP);
        if (kD_up || kD_down)
            robot::controller->print(2, 0, "kD: %f", newgains.kD);

        if (newgains.kP != gains.kP || newgains.kD != gains.kD) {
            printf("gains: %f, %f\n", newgains.kP, newgains.kD);
            controller->setGains(newgains);
        }
        pros::delay(10);
    }
}
