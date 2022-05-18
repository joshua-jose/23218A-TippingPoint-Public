#include "tomobo/ramsete/RamseteController.hpp"

using namespace okapi::literals;

namespace tomobo {
RamseteController::RamseteController(Odometry* iodometry, okapi::QLength itrackWidth) :
    odometry(iodometry), trackWidth(itrackWidth){};

void RamseteController::setPath(const std::vector<Point> ipoints) {
}

void RamseteController::executePath() {
    pathIndex = 0;
    lastMovementTime = pros::millis();
}

// This function gathers all the numbers from the path and from config options
// and turns it into a desired velocity and heading.
std::pair<double, double> RamseteController::step() {
    int reversed = 1;
    bool followMirrored = false;

    if (pathIndex >= trajectory->length)
        return { 0, 0 };

    const auto segDT = trajectory->left.get()[pathIndex].dt * okapi::second;
    const uint32_t time = pros::millis();

    if (time + segDT.convert(okapi::millisecond) > lastMovementTime) {
        pathIndex++;
        if (pathIndex >= trajectory->length)
            return { 0, 0 };
        lastMovementTime = time;
    }

    // printf("pathIndex: %d, len: %d, left: %d, right: %d\n", pathIndex, trajectory->length, leftSpeed, rightSpeed);

    double leftVelocity, rightVelocity;

    // Incomplete
    if (followMirrored) {
        leftVelocity = trajectory->right.get()[pathIndex].velocity * reversed;
        rightVelocity = trajectory->left.get()[pathIndex].velocity * reversed;

    } else {
        leftVelocity = trajectory->left.get()[pathIndex].velocity * reversed;
        rightVelocity = trajectory->right.get()[pathIndex].velocity * reversed;
    }

    return { leftVelocity, rightVelocity };
}

bool RamseteController::isSettled() {
    return true;
};

}
