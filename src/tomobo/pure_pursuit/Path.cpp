#include "tomobo/pure_pursuit/Path.hpp"

#include <cmath>

// These lines enable more agressive optimisations for PathGenerator.
// This is used because the relatively short path in VEX need a lot of smoothing
// and a lot of samples.
// These two factors means the PathGenerator can be quite slow, and so enabling these
// compiler options are definitely a good idea - especially if we want to generate long paths.
#pragma GCC push_options
#pragma GCC optimize("O3") // Enable 03 optimization
#pragma GCC optimize("fast-math") // Enable fast math

/* TODO:
 * Each point on the path (pathPoints) should have a velocity assosciated with it
 * Extend the above idea to include acceleration, heading, and angular velocity
 * Implement acceleration and velocity with trapezoidal acceleration (S curve motion profile)
 * Use PIDF to get the bot to track these velocities and accelerations (ignore heading and angle for now)
 * Eventually use heading and angular velocity to better track the path, nudge the robot in the right direction,
   at least when we're close to the path
 */

/* 
--------------------------------- READ ME!! ---------------------------------

All of the important code in  here has been redacted, but not as an effort to maintain 
"competitive secrecy" or the likes.
I want this to be an educational resource, rather than a place for a free "hole count", so instead
of giving you the source to just copy from, here's the resources I used to learn how to implement
pure pursuit :)
oh and of course seeing the structure of my implementation hopefully helps.

https://github.com/xiaoxiae/PurePursuitAlgorithm
https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552

--------------------------------- READ ME!! ---------------------------------
*/

namespace tomobo {

// Path constructor, where all the parameters are passed.
Path::Path(std::vector<Point> icontrolPoints, okapi::QLength ilookAheadDistance,
    double ikCurvesSlowdown, double iangleDampen, double iangleOutMax,
    okapi::QLength ispacing, float ia, float ib, float itolerance) {

    a = ia, b = ib, tolerance = itolerance;
    lookAheadDistance = ilookAheadDistance;
    kCurveSlowdown = ikCurvesSlowdown;
    angleDampen = iangleDampen;
    angleOutMax = iangleOutMax;
    spacing = ispacing.convert(okapi::meter);

    init(icontrolPoints);
}

// Copy a path (copy constructor)
Path::Path(const Path& other) {
    a = other.a, b = other.b, tolerance = other.tolerance;
    lookAheadDistance = other.lookAheadDistance;
    kCurveSlowdown = other.kCurveSlowdown;
    angleDampen = other.angleDampen;
    angleOutMax = other.angleOutMax;

    controlPoints = other.controlPoints;
    pathPoints = other.pathPoints;

    lastPointIndex = other.lastPointIndex;
    ready.store(other.ready.load());
};

/*
All the following functions are to initially set up the parameters of the path.
They all return a reference to the current class. This means that these functions can be
chained , allowing for really nice syntax.
*/
Path& Path::withLookAheadDistance(okapi::QLength ilookAheadDistance) {
    lookAheadDistance = ilookAheadDistance;
    return *this;
};
Path& Path::withKCurvesSlowdown(double ikCurvesSlowdown) {
    kCurveSlowdown = ikCurvesSlowdown;
    return *this;
};
Path& Path::withAngleDampen(double iangleDampen) {
    angleDampen = iangleDampen;
    return *this;
};
Path& Path::withAngleOutMax(double iangleOutMax) {
    angleOutMax = iangleOutMax;
    return *this;
};
Path& Path::withSpacing(okapi::QLength ispacing) {
    spacing = ispacing.convert(okapi::meter);
    return *this;
};

Path& Path::withReversed(bool ireversed) {
    reversed = ireversed;
    return *this;
};

Path& Path::withMirrored(bool imirrored) {
    mirrored = imirrored;
    return *this;
};

// Actually generate the path.
void Path::init(std::vector<Point> icontrolPoints) {
    controlPoints.reserve(icontrolPoints.size());
    pathPoints.reserve(icontrolPoints.size());

    for (Point i : icontrolPoints) {
        controlPoints.push_back({ static_cast<float>(i.x.convert(okapi::meter)),
            static_cast<float>(i.y.convert(okapi::meter)) });
    }
    inject_points();
    smooth_path();

    ready.store(true);
};

// Injects lots of points along the line between 2 control points
void Path::inject_points() {
    // Redacted
}

// Takes a path made of a few big straight lines, and turns it into
// a rough curve, made of lots of small straight lines.
void Path::smooth_path() {
    // Redacted
}

// Calculates the curvature of the path at the given point
double Path::curvature(Point point) {
    while (!ready.load())
        pros::delay(10);

    // Redacted
    return 0;
}

// We calculate the lookahead point by solving an equation which gives us the
// intersection of radius lookAheadDistance, and a section of the path.
Point Path::getLookAheadPoint(Point botPosition) {
    while (!ready.load())
        pros::delay(10);

    // Redacted
    return { 0 * okapi::meter, 0 * okapi::meter };
}

// Find the point on the path closest to the robot.
Point Path::getClosestPoint(Point botPosition) {
    std::valarray<float> botPosMeters = {
        static_cast<float>(botPosition.x.convert(okapi::meter)),
        static_cast<float>(botPosition.y.convert(okapi::meter))
    };

    int closestPointIndex = 0;
    float closestDistance = INT_MAX;

    for (int i = 0; i < pathPoints.size(); i++) {
        std::valarray<float> pathPoint = pathPoints[i];
        float distance = std::hypot(pathPoint[0] - botPosMeters[0], pathPoint[1] - botPosMeters[1]);
        if (distance < closestDistance) {
            closestPointIndex = i;
            closestDistance = distance;
        }
    }

    return { pathPoints[closestPointIndex][0] * okapi::meter,
        pathPoints[closestPointIndex][1] * okapi::meter };
}

bool Path::isAtEnd() {
    // We hit the end of the path when we reach the second-last point, aka the start of the last line segment.
    return lastPointIndex == pathPoints.size() - 2;
}

}

#pragma GCC pop_options // Pop special optimizations for Path Generator
