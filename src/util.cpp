#include "project/util.hpp"

double scale(double x, double min, double max, double b, double a) {
    return b - (((b - a) * (x - min)) / (max - min));
}

int gearsetToRPM(pros::motor_gearset_e_t gearset) {
    switch (gearset) {
    case pros::motor_gearset_e_t::E_MOTOR_GEARSET_06:
        return 600;
    case pros::motor_gearset_e_t::E_MOTOR_GEARSET_18:
        return 200;
    case pros::motor_gearset_e_t::E_MOTOR_GEARSET_36:
        return 100;
    default:
        return 200;
    }
}