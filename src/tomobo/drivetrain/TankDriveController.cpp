#include "tomobo/drivetrain/TankDriveController.hpp"

namespace tomobo {

TankDriveController::TankDriveController(ChassisScales itrackingScales, ChassisScales ichassisScales,
    Odometry* iodometry,
    std::unique_ptr<FeedbackController> iforwardDistanceController,
    std::unique_ptr<FeedbackController> iangleController) :
    trackingScales(itrackingScales),
    chassisScales(ichassisScales),
    odometry(iodometry),
    forwardDistanceController(std::move(iforwardDistanceController)),
    angleController(std::move(iangleController)),
    purePursuitController(odometry, ichassisScales.wheelTrack){};

TankDriveController::~TankDriveController() {
    stopTask();
};

// Relative movements need to reset everything before running
TankDriveController* const TankDriveController::driveStraight(QLength distance) {
    reset();
    setState("DRIVE_STRAIGHT");
    driveStraightTarget = distance;
    return this;
};

TankDriveController* const TankDriveController::turnAngle(QAngle angle) {
    reset();
    setState("TURN");
    angleTarget = angle;
    return this;
};

TankDriveController* const TankDriveController::lookAt(Point point, QAngle offset) {
    reset();
    setState("LOOK_AT");
    lookAtTarget = point;
    angleOffset = offset;
    return this;
};

TankDriveController* const TankDriveController::driveDeltaDistance(Point point, bool reversed) {
    reset();
    QLength distance = odometry->getPose().distance(point);
    if (reversed)
        distance *= -1;
    driveStraight(distance);
    return this;
};

TankDriveController* const TankDriveController::setHeading(QAngle heading, QAngle offset) {
    reset();
    setState("SET_HEADING");
    headingTarget = heading;
    angleOffset = offset;
    return this;
};

TankDriveController* const TankDriveController::arc(Point point) {
    reset();
    setState("ARC");
    arcTarget = point;
    return this;
};

TankDriveController* const TankDriveController::purePursuit(const Path& ipath) {
    reset();
    purePursuitController.setPath(ipath);

    setState("PURE_PURSUIT");
    return this;
}

TankDriveController* const TankDriveController::motionProfile(std::unique_ptr<MotionProfileGenerator::TrajectoryPair> ipath) {
    reset();
    // purePursuitController.setPath(ipath);
    trajectory = std::move(ipath);
    useVelocity.store(true);
    pathIndex = 0;
    lastMPMovement = pros::millis();

    setState("MOTION_PROFILE");
    return this;
}

// Start the background task that runs the drive controller
void TankDriveController::startTask() {
    if (task)
        return;

    runTask.store(true);
    task = std::make_unique<pros::Task>(
        [=] {
            loop();
        },
        TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "DriveController_Task");
};

void TankDriveController::stopTask() {
    if (!task)
        return; // TODO: Log

    task->notify();
    runTask.store(false);
    while (taskRunning.load())
        pros::delay(5);

    task.reset();
};

void TankDriveController::loop() {
    taskRunning.store(true);
    std::uint32_t now = pros::millis();
    while (runTask.load() && !task->notify_take(true, 0)) {
        step();
        pros::Task::delay_until(&now, updateRate.convert(okapi::millisecond));
    };
    taskRunning.store(false);
};

void TankDriveController::reset() {
    leftPower = 0;
    rightPower = 0;

    setState("IDLE");
    forwardDistanceController->reset();
    angleController->reset();

    leftStart = leftEncoder;
    rightStart = rightEncoder;
    useVelocity.store(false);
};
void TankDriveController::setDisabled(bool disabled) {
    isDisabled.store(disabled);
};

void TankDriveController::step() {
    // On each update, we read new numbers from the sensors, we then run the
    // step an algorithm according to the state of our state machine.
    // We then update the motors (write new power values to them)
    updateSensors();

    if (controlState == "DRIVE_STRAIGHT")
        driveStraightHandler();
    else if (controlState == "TURN")
        turnHandler();
    else if (controlState == "LOOK_AT")
        lookAtHandler();
    else if (controlState == "SET_HEADING")
        setHeadingHandler();
    else if (controlState == "ARC")
        arcHandler();
    else if (controlState == "PURE_PURSUIT")
        purePursuitHandler();
    else if (controlState == "MOTION_PROFILE")
        motionProfileHandler();

    updateMotors();
};

// All the functions that actually handle running the algorithms

void TankDriveController::driveStraightHandler() {
    double left = (leftEncoder - leftStart) / trackingScales.straight;
    double right = (rightEncoder - rightStart) / trackingScales.straight;

    QAngle angle = ((left - right) / trackingScales.wheelTrack.convert(okapi::meter)) * radian;
    double distanceError = driveStraightTarget.convert(meter) - ((left + right) / (2.0));

    double straightOut = forwardDistanceController->step(distanceError);
    double angleOut = angleController->step((-angle).convert(radian)) / 2;

    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;
};
void TankDriveController::turnHandler() {
    double left = (leftEncoder - leftStart) / trackingScales.straight;
    double right = (rightEncoder - rightStart) / trackingScales.straight;

    QAngle angle = ((left - right) / trackingScales.wheelTrack.convert(okapi::meter)) * radian;

    double angleOut = angleController->step((angleTarget - angle).convert(radian));

    leftPower = angleOut;
    rightPower = -angleOut;
};
void TankDriveController::lookAtHandler() {
    double angleError = odometry->getPose().angleTo(lookAtTarget).convert(radian) + angleOffset.convert(radian);
    angleError = atan2(sin(angleError), cos(angleError));

    double angleOut = angleController->step(angleError);

    leftPower = angleOut;
    rightPower = -angleOut;
};
void TankDriveController::setHeadingHandler() {
    double angleError = (headingTarget - odometry->getPose().theta).convert(radian);
    angleError = atan2(sin(angleError), cos(angleError));

    double angleOut = angleController->step(angleError);

    leftPower = angleOut;
    rightPower = -angleOut;
};
void TankDriveController::arcHandler() {
    Pose botPose = odometry->getPose();

    double x0 = arcTarget.x.convert(meter);
    double y0 = arcTarget.y.convert(meter);
    double theta = botPose.theta.convert(radian);

    double x1 = x0;
    double y1 = y0;

    double angleError = botPose.angleTo(arcTarget).convert(radian);

    if (angleError >= (1_deg).convert(radian) || angleError <= (-1_deg).convert(radian)) {
        // X coordinate of the closest point the robot could get to if it kept going forward.
        x1 = (y0 + tan(theta) * x0) / (tan(-theta + 0.5_pi) - tan(-theta));
        y1 = tan(-theta + 0.5_pi) * x1;
    }

    double distanceError = botPose.distance({ x1 * meter, y1 * meter }).convert(meter);
    double angleToLookPoint = botPose.angleTo({ x1 * meter, y1 * meter }).convert(radian);

    angleError = atan2(sin(angleError), cos(angleError));
    angleToLookPoint = atan2(sin(angleToLookPoint), cos(angleToLookPoint));

    double direction = (angleToLookPoint >= -0.5_pi && angleToLookPoint <= 0.5_pi) ? 1.0 : -1.0;
    distanceError *= direction;

    if (botPose.distance(arcTarget) < 10_cm) {
        angleError = 0;
    } else if (direction == -1) {
        angleError = angleError - (180_deg).convert(radian);
    }

    if (botPose.distance(arcTarget) < arcTarget.distance({ x1 * meter, y1 * meter })) {
        distanceError = 0;
    }

    double straightOut = forwardDistanceController->step(distanceError);
    double angleOut = angleController->step(angleError);

    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;
}

void TankDriveController::purePursuitHandler() {
    std::pair<double, double> output = purePursuitController.step();
    double angleError = atan2(sin(output.second), cos(output.second));

    double angleDampen = purePursuitController.path->angleDampen;
    double angleOutMax = purePursuitController.path->angleOutMax;

    double angleOut = angleController->step(angleError);
    angleOut = std::clamp(angleOut, -angleOutMax, angleOutMax);
    double straightOut = output.first - std::abs(angleOut / angleDampen);

    if (purePursuitController.path->mirrored)
        angleOut *= -1;

    // val, min, max
    // clamps straight out to only let maximum velocity be between -1 and 1 TODO:
    angleOut = std::clamp(angleOut, -1.0, 1.0);
    straightOut = std::clamp(straightOut, -(1 - abs(angleOut)), 1 - abs(angleOut));

    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;

    /*
    const auto leftRPM = convertLinearToRotational(output.first * mps, chassisScales).convert(rpm);
    const auto rightRPM = convertLinearToRotational(output.second * mps, chassisScales).convert(rpm);
    leftVelocity = leftRPM;
    rightVelocity = rightRPM;
    */
}

QAngularSpeed convertLinearToRotational(QSpeed linear, ChassisScales scales) {
    return (linear * (360_deg / (scales.wheelDiameter * 1_pi)));
}

void TankDriveController::motionProfileHandler() {
    // trajectory;
    // pathIndex = 0;
    // lastMPMovement = pros::millis();

    int reversed = 1;
    bool followMirrored = false;

    if (pathIndex >= trajectory->length)
        return;

    const auto segDT = trajectory->left.get()[pathIndex].dt * second;
    const uint32_t time = pros::millis();

    if (time + segDT.convert(millisecond) > lastMPMovement) {
        pathIndex++;
        if (pathIndex >= trajectory->length)
            return;
        lastMPMovement = time;
    }

    const auto leftRPM = convertLinearToRotational(trajectory->left.get()[pathIndex].velocity * mps, chassisScales).convert(rpm);
    const auto rightRPM = convertLinearToRotational(trajectory->right.get()[pathIndex].velocity * mps, chassisScales).convert(rpm);

    // const double rightSpeed = rightRPM / toUnderlyingType(pair.internalGearset) * reversed;
    const double rightSpeed = rightRPM * reversed;
    const double leftSpeed = leftRPM * reversed;

    printf("pathIndex: %d, len: %d, left: %d, right: %d\n", pathIndex, trajectory->length, leftSpeed, rightSpeed);

    if (followMirrored) {
        leftVelocity = rightSpeed;
        rightVelocity = leftSpeed;

    } else {
        leftVelocity = leftSpeed;
        rightVelocity = rightSpeed;
    }
}
/*
void TankDriveController::straightVelocityHandler() {
    double left = (leftEncoder - leftStart) / chassisScales.straight;
    double right = (rightEncoder - rightStart) / chassisScales.straight;

    QAngle angle = ((left - right) / chassisScales.wheelTrack.convert(okapi::meter)) * radian;
    double distanceError = driveStraightTarget.convert(meter) - ((left + right) / (2.0));

    double straightOut = forwardDistanceController->step(distanceError);

    leftPower = straightOut;
    rightPower = straightOut;
};
*/

bool TankDriveController::isSettled() {

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
    else if (controlState == "ARC")
        return forwardDistanceController->isSettled() && angleController->isSettled();
    else if (controlState == "PURE_PURSUIT")
        return purePursuitController.isSettled();
    else if (controlState == "MOTION_PROFILE")
        return pathIndex >= trajectory->length;
    return true;
};

void TankDriveController::wait(int timeout) {
    int time = pros::millis();
    if (timeout == 0) {
        while (!isSettled())
            pros::delay(10);
    } else {
        while (!isSettled() && !(time + timeout < pros::millis()))
            pros::delay(10);
    }
    reset();
    updateMotors();
};

void TankDriveController::setState(std::string icontrolState) {
    controlState = icontrolState;
};

std::string TankDriveController::getState() {
    return controlState;
};

void TankDriveController::setMaxSpeed(QSpeed speed){};
}
