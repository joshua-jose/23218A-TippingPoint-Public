#pragma once

#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "tomobo/odometry/Odometry.hpp"
#include "tomobo/units.hpp"

extern "C" {
#include "okapi/pathfinder/include/pathfinder.h"
}

namespace tomobo {

class RamseteController {
public:
    RamseteController(Odometry* iodometry, okapi::QLength trackWidth);
    ~RamseteController() = default;

    void setPath(const std::vector<Point> ipoints);
    void executePath();

    // Some way to generate paths in advance
    // void preparePath(std::vector<Point> ipoints);

    // Returns linear power, angular delta.
    std::pair<double, double> step();

    bool isSettled();

protected:
    Odometry* odometry;

    okapi::QTime sampleRate = 10 * okapi::millisecond;
    okapi::QTime slewTime = sampleRate;
    okapi::QLength trackWidth{ 0 * okapi::meter };
    double maxVelocity = 1;

    using TrajectoryPtr = std::unique_ptr<TrajectoryCandidate, void (*)(TrajectoryCandidate*)>;
    using SegmentPtr = std::unique_ptr<Segment, void (*)(void*)>;

    struct TrajectoryPair {
        SegmentPtr left;
        SegmentPtr right;
        int length;
    };

    std::unique_ptr<TrajectoryPair> trajectory;
    std::uint32_t pathIndex = 0;
    std::uint32_t lastMovementTime = 0;
};
}
