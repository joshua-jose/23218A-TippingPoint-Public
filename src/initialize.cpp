#include "main.h"
#include "project/gui.hpp"

void auton_init();

void initialize() {
    printf("Initializing...\n");
    printf("run\n"); // reset graph

    // Wait for PROS to initialize.
    pros::delay(150);

    /*
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Goodbye PROS User.");

    pros::lcd::register_btn1_cb(on_center_button);
    */

    robot::init(); // Initialise anything that needs to be.
    auton_init(); // add all auton routines to selector

    std::shared_ptr<GUI> gui = GUI::get();
    gui->gui_build();
    //  pros::lcd::initialize();
    pros::delay(150);
    fprintf(stderr, "Initialized.\n");
}

void disabled() {}

void competition_initialize() {
    // robot::chassis->inertial.reset();
    std::shared_ptr<GUI> gui = GUI::get();
    while (true) {
        gui->update_inertial_status(robot::chassis->inertial);
        pros::delay(20);
    }
}
