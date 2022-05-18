#include "tomobo/drivetrain/XDriveController.hpp"

namespace tomobo {

XDriveController::XDriveController(ChassisScales itrackingScales, ChassisScales ichassisScales,
    Odometry* iodometry,
    std::unique_ptr<FeedbackController> iforwardDistanceController,
    std::unique_ptr<FeedbackController> iangleController,
    std::unique_ptr<FeedbackController> istrafeController) :
    TankDriveController(
        itrackingScales, ichassisScales, iodometry,
        std::move(iforwardDistanceController), std::move(iangleController)),

    strafeController(std::move(istrafeController)),
    XDrivePursuitController(odometry, 0 * okapi::meter) {}

// We create a second pure pursuit controller to deal with holonomic purusit

XDriveController::~XDriveController() {
    stopTask();
};

void XDriveController::wait() {
    while (!isSettled()) {
        pros::delay(10);
    }
    reset();
    updateMotors();
};

void XDriveController::reset() {
    // Sometimes we *may* want to switch movement algorithm while in motion.
    // We dont update motors here, so a reset wont cause a jolt, however wait will.
    // We can probably get a new motor power in before the motors get actually updated though.
    leftPower = 0;
    rightPower = 0;
    strafePower = 0;
    XPower = 0;
    YPower = 0;
    TankDriveController::reset();
    strafeController->reset();

    midStart = midEncoder;
};

void XDriveController::step() {
    updateSensors();
    // Reset the powers every step, so that they will be zeroed out if they aren't explicitely set.
    leftPower = 0, rightPower = 0, strafePower = 0, XPower = 0, YPower = 0;
    if (controlState == "DRIVE_STRAIGHT")
        driveStraightHandler();
    else if (controlState == "TURN")
        turnHandler();
    else if (controlState == "LOOK_AT")
        lookAtHandler();
    else if (controlState == "SET_HEADING")
        setHeadingHandler();
    else if (controlState == "STRAFE")
        strafeHandler();
    else if (controlState == "VECTOR_DRIVE")
        vectorDriveHandler();
    else if (controlState == "DRIVE_VECTOR_ANGLE")
        driveVectorAngleHandler();
    else if (controlState == "SIMUL_DRIVE")
        simulDriveHandler();
    else if (controlState == "ARC")
        arcHandler();
    else if (controlState == "PURE_PURSUIT")
        purePursuitHandler();
    else if (controlState == "HOLONOMIC_PURSUIT")
        holonomicPursuitHandler();

    updateMotors();
};

bool XDriveController::isSettled() {
    if (controlState == "IDLE")
        return true;
    else if (controlState == "DRIVE_STRAIGHT")
        return forwardDistanceController->isSettled() && angleController->isSettled();
    else if (controlState == "TURN")
        return angleController->isSettled();
    else if (controlState == "LOOK_AT")
        return angleController->isSettled();
    else if (controlState == "SET_HEADING")
        return angleController->isSettled();
    else if (controlState == "STRAFE")
        return strafeController->isSettled();
    else if (controlState == "VECTOR_DRIVE")
        return forwardDistanceController->isSettled() && strafeController->isSettled();
    else if (controlState == "DRIVE_VECTOR_ANGLE")
        return forwardDistanceController->isSettled();
    else if (controlState == "SIMUL_DRIVE")
        return forwardDistanceController->isSettled() && strafeController->isSettled()
            && angleController->isSettled();
    else if (controlState == "ARC")
        return forwardDistanceController->isSettled() && angleController->isSettled();
    else if (controlState == "PURE_PURSUIT")
        return purePursuitController.isSettled();
    else if (controlState == "HOLONOMIC_PURSUIT")
        return XDrivePursuitController.isSettled();

    return true;
};

XDriveController* const XDriveController::strafe(QLength distance) {
    reset();
    setState("STRAFE");
    strafeTarget = distance;
    return this;
};

XDriveController* const XDriveController::vectorDrive(Point point) {
    reset();
    setState("VECTOR_DRIVE");
    vectorTarget = point;
    return this;
};

XDriveController* const XDriveController::simulDrive(Point point, QAngle heading, SimulDriveType type) {
    reset();
    simulDriveLinDisabled = false;
    simulDriveType = type;
    setState("SIMUL_DRIVE");
    vectorTarget = point;
    headingTarget = heading;
    return this;
};

XDriveController* const XDriveController::driveVectorAngle(Point target) {
    reset();
    setState("DRIVE_VECTOR_ANGLE");
    vectorTarget = target;
    return this;
};

XDriveController* const XDriveController::holonomicPursuit(const Path& ipath) {
    reset();
    XDrivePursuitController.setPath(ipath);
    setState("HOLONOMIC_PURSUIT");
    return this;
}

void XDriveController::strafeHandler() {
    double left = (leftEncoder - leftStart) / trackingScales.straight;
    double right = (rightEncoder - rightStart) / trackingScales.straight;
    double mid = (midEncoder - midStart) / trackingScales.middle;

    double distanceError = strafeTarget.convert(meter) - mid;
    double forwarddistanceError = -((left + right) / (2.0));
    QAngle angle = ((left - right) / trackingScales.wheelTrack.convert(okapi::meter)) * radian;

    double strafeOut = strafeController->step(distanceError);
    double straightOut = forwardDistanceController->step(forwarddistanceError);
    double angleOut = angleController->step((-angle).convert(radian));

    // printf("graph:%d,%f\n", pros::millis(), ((left + right) / (2.0)));

    strafePower = strafeOut;
    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;
};

void XDriveController::driveStraightHandler() {
    double left = (leftEncoder - leftStart) / trackingScales.straight;
    double right = (rightEncoder - rightStart) / trackingScales.straight;
    double mid = (midEncoder - midStart) / trackingScales.middle;

    QAngle angle = ((left - right) / trackingScales.wheelTrack.convert(okapi::meter)) * radian;
    double distanceError = driveStraightTarget.convert(meter) - ((left + right) / (2.0));
    double strafeDistanceError = strafeTarget.convert(meter) - mid;

    double strafeOut = strafeController->step(strafeDistanceError);
    double straightOut = forwardDistanceController->step(distanceError);
    double angleOut = angleController->step((-angle).convert(radian));

    strafePower = strafeOut;
    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;
};

void XDriveController::vectorDriveHandler() {
    Pose botPose = odometry->getPose();

    double left = (leftEncoder - leftStart) / trackingScales.straight;
    double right = (rightEncoder - rightStart) / trackingScales.straight;

    QAngle angle = ((left - right) / trackingScales.wheelTrack.convert(okapi::meter)) * radian;

    QLength distance = botPose.distance(vectorTarget);
    QAngle angleTo = botPose.angleTo(vectorTarget);

    QLength strafeDistance = sin(angleTo.convert(okapi::radian)) * distance;
    QLength forwardDistance = cos(angleTo.convert(okapi::radian)) * distance;

    double strafeOut = strafeController->step(strafeDistance.convert(okapi::meter));
    double straightOut = forwardDistanceController->step(forwardDistance.convert(okapi::meter));
    double angleOut = angleController->step((-angle).convert(radian));

    strafePower = strafeOut;
    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;
};

void XDriveController::simulDriveHandler() {
    Pose botPose = odometry->getPose();

    QLength distance = botPose.distance(vectorTarget);
    QAngle angleTo = botPose.angleTo(vectorTarget);

    QLength strafeDistance = sin(angleTo.convert(okapi::radian)) * distance;
    QLength forwardDistance = cos(angleTo.convert(okapi::radian)) * distance;

    double angleError = (headingTarget - odometry->getPose().theta).convert(radian);
    angleError = atan2(sin(angleError), cos(angleError));

    double strafeOut = strafeController->step(strafeDistance.convert(okapi::meter));
    double straightOut = forwardDistanceController->step(forwardDistance.convert(okapi::meter));
    double angleOut = angleController->step(angleError);

    double linEffort = std::hypot(strafeOut, straightOut);
    // Equal effort
    if (simulDriveType == SimulDriveType::EQUAL) {
        if ((forwardDistanceController->isSettled() && strafeController->isSettled()) || simulDriveLinDisabled) {
            straightOut = 0;
            strafeOut = 0;
            simulDriveLinDisabled = true;
        }
        if (std::abs(linEffort) + std::abs(angleOut) > 1) {
            angleOut = std::clamp(angleOut, -0.5, 0.5);
            double linScaler = linEffort * (1 / 0.5);
            strafeOut /= linScaler;
            straightOut /= linScaler;
        }
    }

    // Always linear
    if (simulDriveType == SimulDriveType::ALWAYS_LINEAR) {
        if (std::abs(linEffort) + std::abs(angleOut) > 1) {
            angleOut = std::copysign(1 - std::abs(linEffort), angleOut);
        }
    }
    // This algorithm may be too agressive, needs testing.
    /*
    if (std::abs(linEffort) + std::abs(angleOut) > 1){
        QAngle angleDelta = okapi::radian * atan2(sin(angleTo.convert(okapi::radian)), cos(angleTo.convert(okapi::radian)));
        double angleDelta_d = angleDelta.convert(okapi::degree);
        double kOffAngle = (std::abs(std::fmod(angleDelta_d, 90)) - 45) / 45.0;
        //const double kAngleWeight = 1;
        //kOffAngle *= kAngleWeight;

        double linScaler = linEffort / std::max(std::numeric_limits<double>::min(),kOffAngle);
        printf("%f, %f, %f, %f\n", angleDelta_d, std::fmod(angleDelta_d, 90), std::abs(std::fmod(angleDelta_d, 90) - 45), kOffAngle);

        strafeOut /= linScaler;
        straightOut /= linScaler;

        //printf("%f,%f,%f,%f\n", angleTo.convert(okapi::degree), strafeOut, straightOut, angleOut);
        angleOut = std::copysign(1 - std::abs(kOffAngle), angleOut);

    }
    */

    strafePower = strafeOut;
    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;
};

void XDriveController::driveVectorAngleHandler() {
    Pose botPose = odometry->getPose();

    QLength distance = botPose.distance(vectorTarget);
    QAngle angleTo = botPose.angleTo(vectorTarget);

    double linearOut = forwardDistanceController->step(distance.convert(meter));
    angleLinearPower(linearOut, angleTo);
};

void XDriveController::holonomicPursuitHandler() {
    std::pair<double, double> output = XDrivePursuitController.step();
    double angleError = atan2(sin(output.second), cos(output.second));

    angleLinearPower(output.first, radian * output.second);
}

void XDriveController::angleLinearPower(double linearPower, QAngle angle, double angularPower) {
    double theta = angle.convert(radian);
    // The loss of speed at theta
    // Divide by this to attempt to get the same velocity at every possible theta
    // but also still be able to get max velocity.
    double penalty = std::max(abs(sin(theta + 0.25_pi)), abs(sin(theta + 0.75_pi)));
    // Multiply the power by this to get the same linear speed at angle
    // double penalty = (sqrt(2) / 2) + 1 - linearise;

    XPower = (linearPower / penalty) * sin(angle.convert(radian) + 0.75_pi);
    YPower = (linearPower / penalty) * sin(angle.convert(radian) + 0.25_pi);
    leftPower = angularPower;
    rightPower = -angularPower;
};

}
