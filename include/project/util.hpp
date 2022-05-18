#pragma once
#include "api.h"

#define WAIT_UNTIL(CONDITION) \
    while (!(CONDITION))      \
        pros::delay(10);

#define WAIT_UNTIL_TIMEOUT(CONDITION, TIMEOUT)                \
    time = pros::millis();                                    \
    while (!(CONDITION || (time + TIMEOUT < pros::millis()))) \
        pros::delay(10);

double scale(double x, double min, double max, double b, double a);
int gearsetToRPM(pros::motor_gearset_e_t gearset);