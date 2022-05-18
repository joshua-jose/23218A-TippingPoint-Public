#pragma once

#include "okapi/api/odometry/stateMode.hpp"
#include "tomobo/units.hpp"

#include <cmath>

namespace tomobo {

struct Point {
    okapi::QLength x;
    okapi::QLength y;

    //okapi::StateMode mode = okapi::StateMode::CARTESIAN;

    Point(okapi::QLength ix, okapi::QLength iy) :
        x(ix), y(iy){};

    Point(okapi::QLength ix, okapi::QLength iy, okapi::StateMode imode) :
        x(ix), y(iy){};

    okapi::QLength distance(const Point other) {
        double dx_m = (other.x - x).convert(okapi::meter);
        double dy_m = (other.y - y).convert(okapi::meter);

        return okapi::meter * std::hypot(dx_m, dy_m);
    };

    okapi::QAngle angle(const Point other) {
        double dx_m = (other.x - x).convert(okapi::meter);
        double dy_m = (other.y - y).convert(okapi::meter);
        // the -theta - 90_deg rotates the angle from
        // 0_deg being along +X and CCW  to being +Y and CW
        // we use +Y and CW because it's more intuitive
        return -(okapi::radian * std::atan2(dy_m, dx_m)) - (90 * okapi::degree);
    };
};

}
