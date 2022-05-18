#pragma once

#include "tomobo/odometry/Point.hpp"
#include "tomobo/units.hpp"
#include <sstream>
#include <string>

namespace tomobo {
struct Pose {
    okapi::QLength x;
    okapi::QLength y;
    okapi::QAngle theta;

    std::string str() const {
        std::ostringstream os;
        os << "Pose(x=" << std::to_string(x.convert(okapi::meter))
           << "m, y=" << std::to_string(y.convert(okapi::meter))
           << "m, theta=" << std::to_string(theta.convert(okapi::degree)) << "deg)";
        return os.str();
    };

    bool operator==(const Pose& rhs) const {
        return x == rhs.x && y == rhs.y && theta == rhs.theta;
    };

    bool operator!=(const Pose& rhs) const {
        return !(*this == rhs);
    };

    okapi::QLength distance(Point other) const {
        return other.distance({ x, y });
    }

    okapi::QAngle angleTo(Point other) const {
        return other.angle({ x, y }) - theta;
    }
    Point toPoint() const {
        return { x, y };
    }
};
}
