#pragma once

#include <atomic>
#include <climits>
#include <valarray>
#include <vector>

#include "api.h"
#include "tomobo/odometry/Point.hpp"

namespace tomobo {

class Path {
public:
    Path(std::vector<Point> icontrolPoints, okapi::QLength ilookAheadDistance = 0.1 * okapi::meter,
        double ikCurvesSlowdown = 2, double iangleDampen = 1, double iangleOutMax = 1,
        okapi::QLength ispacing = 1 * okapi::centimeter, float ia = 0.01, float ib = 0.99, float itolerance = 0.01);

    Path(const Path& other);

    // TODO: Implement these

    Path& withLookAheadDistance(okapi::QLength ilookAheadDistance);
    Path& withKCurvesSlowdown(double ikCurvesSlowdown);
    Path& withAngleDampen(double iangleDampen);
    Path& withAngleOutMax(double iangleOutMax);
    Path& withSpacing(okapi::QLength ispacing);
    Path& withReversed(bool ireversed);
    Path& withMirrored(bool imirrored);

    /*
    Path& withSmoothingConstants(float ia, float ib, float itolerance);
    Path& build();
    // rename init() to build() and don't call in constructor
    */

    Point getLookAheadPoint(Point botPosition);
    double curvature(Point point);
    Point getClosestPoint(Point botPosition);

    bool isAtEnd();

    okapi::QLength lookAheadDistance = 20 * okapi::centimeter;
    double kCurveSlowdown = 2;

    double angleDampen = 1;
    double angleOutMax = 1;

    bool reversed = false, mirrored = false;

    std::vector<std::valarray<float>> controlPoints;
    std::vector<std::valarray<float>> pathPoints;

protected:
    void inject_points();
    void smooth_path();
    void init(std::vector<Point> icontrolPoints);

    int lastPointIndex = 0;

    std::atomic_bool ready{ false };

    // Space between injected points
    float spacing = 0.01;

    // Smoothing constants (b is how smooth the path is,
    // and tolerance relates to how many iterations to do)
    float b = 0.99, a = 1 - b, tolerance = 0.01;
};

}
