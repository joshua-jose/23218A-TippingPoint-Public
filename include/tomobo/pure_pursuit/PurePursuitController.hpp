#pragma once

#include "tomobo/controller/FeedbackController.hpp"
#include "tomobo/odometry/Odometry.hpp"
#include "tomobo/pure_pursuit/Path.hpp"

namespace tomobo {

class PurePursuitController {
public:
    PurePursuitController(Odometry* iodometry, okapi::QLength trackWidth);
    ~PurePursuitController() = default;

    void setPath(const Path& ipath);

    // Some way to generate paths in advance
    // void preparePath(std::vector<Point> ipoints);

    // Returns linear power, angular delta.
    std::pair<double, double> step();

    bool isSettled();
    void setMaxVelocity(double imaxVelocity);
    void setSlewTime(okapi::QTime islewTIme);

    std::unique_ptr<Path> path;
    Point lookAheadPoint{ 0 * okapi::meter, 0 * okapi::meter };

protected:
    Odometry* odometry;

    float getCurvatureToPoint(Pose botPosition);

    okapi::QTime sampleRate = 10_ms;
    okapi::QTime slewTime = sampleRate;
    okapi::QLength trackWidth{ 0 * okapi::meter };
    double maxVelocity = 1;
};
}
