#include "tomobo/pure_pursuit/PurePursuitController.hpp"

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
PurePursuitController::PurePursuitController(Odometry* iodometry, okapi::QLength itrackWidth) :
    odometry(iodometry), trackWidth(itrackWidth){};

void PurePursuitController::setPath(const Path& ipath) {
    lookAheadPoint = { 0_m, 0_m };
    path = std::make_unique<Path>(ipath);
}

// This function gathers all the numbers from the path and from config options
// and turns it into a desired velocity and heading.
std::pair<double, double> PurePursuitController::step() {
    std::pair<double, double> output{ 0, 0 };

    Pose botPosition = odometry->getPose();
    lookAheadPoint = path->getLookAheadPoint({ botPosition.x, botPosition.y });

    okapi::QAngle angleDelta = botPosition.angleTo(lookAheadPoint);
    double curvature = path->curvature(lookAheadPoint);

    // Uncomment below for proper algorithm
    // double velocity = std::clamp(maxVelocity * (path->kCurveSlowdown / curvature), 0.0, maxVelocity);
    double velocity = std::clamp(maxVelocity, 0.0, maxVelocity);

    // Velocity slew
    static double lastVelocity = 0;
    double max_increase = sampleRate.convert(okapi::second) / slewTime.convert(okapi::second);
    double desired_change = velocity - lastVelocity;

    double clamped_increase = std::clamp(desired_change, -max_increase, max_increase);

    velocity = lastVelocity + clamped_increase;
    lastVelocity = velocity;

    if (path->reversed) {
        angleDelta = 180_deg - angleDelta;
        velocity *= -1;
    }

    // Follow an arc algorithm from Team 1712 paper
    // L = V * (2 + CT)/2
    // R = V * (2 − CT)/2
    /*
    redacted :)
    */

    // Probably get feedback control dealt with using Tank Controller wrapper thing
    // std::shared_ptr<FeedbackController> forwardVelocityController;
    // forwardVelocityController->setTarget(velocity);
    // double power = (Kv * velocity) + (Ka * accel) + (Kp * (target - current));
    output = { velocity, angleDelta.convert(okapi::radian) };
    return output;
}
// Algorithm from "Implementation of the Adaptive Pure Pursuit Controller" by Team 1712
//  Get the curvature of the arc between the robot, and the lookahead point
float PurePursuitController::getCurvatureToPoint(Pose botPosition) {
    // Redacted :)
    return 0.0;
}

bool PurePursuitController::isSettled() {
    // This does mean that we will settle while still in motion, but it's not this class's responsibility
    std::valarray<float> lastPoint = path->controlPoints.back();
    Point lastPoint_p = Point{ lastPoint[0] * okapi::meter, lastPoint[1] * okapi::meter };

    return path->isAtEnd() && (lastPoint_p.distance(lookAheadPoint) < 0.01_m);
};

void PurePursuitController::setMaxVelocity(double imaxVelocity) {
    maxVelocity = imaxVelocity;
};

void PurePursuitController::setSlewTime(okapi::QTime islewTIme) {
    slewTime = islewTIme;
};

}
