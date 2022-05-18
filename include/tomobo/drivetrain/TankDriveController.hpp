#pragma once

#include "tomobo/MotionProfileGenerator.hpp"
#include "tomobo/controller/FeedbackController.hpp"
#include "tomobo/odometry/Odometry.hpp"
#include "tomobo/odometry/Point.hpp"
#include "tomobo/pure_pursuit/PurePursuitController.hpp"
#include "tomobo/units.hpp"

#include <atomic>
#include <map>

using namespace okapi;
using namespace okapi::literals;
using namespace tomobo;

namespace tomobo {
class TankDriveController {
public:
    TankDriveController(ChassisScales itrackingScales, ChassisScales ichassisScales,
        Odometry* iodometry,
        std::unique_ptr<FeedbackController> iforwardDistanceController,
        std::unique_ptr<FeedbackController> iangleController);

    ~TankDriveController();

    virtual void wait(int timeout = 0);

    TankDriveController* const driveStraight(QLength distance);
    TankDriveController* const turnAngle(QAngle angle);

    TankDriveController* const lookAt(Point point, QAngle offset = 0_deg);
    TankDriveController* const driveDeltaDistance(Point point, bool reversed = false);
    TankDriveController* const setHeading(QAngle heading, QAngle offset = 0_deg);

    TankDriveController* const arc(Point point);

    TankDriveController* const purePursuit(const Path& ipath);
    TankDriveController* const motionProfile(std::unique_ptr<MotionProfileGenerator::TrajectoryPair> ipath);

    // void driveArc(tomobo::Point point);

    void setMaxSpeed(QSpeed speed);

    void startTask();
    void stopTask();

    virtual void reset();
    void setDisabled(bool disabled);
    bool isSettled();

    std::string getState();

    virtual void step();

    virtual void updateSensors() = 0;
    virtual void updateMotors() = 0;

    okapi::QTime updateRate = 10_ms;

    std::unique_ptr<FeedbackController> forwardDistanceController;
    std::unique_ptr<FeedbackController> angleController;
    std::unique_ptr<FeedbackController> forwardVelocityController;
    int leftStart = 0, rightStart = 0, midStart = 0;

    okapi::ChassisScales trackingScales;
    okapi::ChassisScales chassisScales;

    int leftEncoder{ 0 }, rightEncoder{ 0 }, midEncoder{ 0 };
    std::atomic<double> leftPower{ 0 }, rightPower{ 0 };
    std::atomic<double> leftVelocity{ 0 }, rightVelocity{ 0 };

protected:
    std::string controlState = "IDLE";

    void loop();

    virtual void driveStraightHandler();
    void turnHandler();
    void lookAtHandler();
    void setHeadingHandler();
    void arcHandler();
    void purePursuitHandler();
    void straightVelocityHandler();
    void motionProfileHandler();

    virtual void setState(std::string icontrolState);

    Odometry* odometry;
    PurePursuitController purePursuitController;

    std::unique_ptr<pros::Task> task;
    std::atomic_bool runTask{ false };
    std::atomic_bool taskRunning{ false };

    std::atomic_bool isDisabled{ false };

    // Control state variables
    std::atomic_bool useVelocity{ false };
    int pathIndex = 0; // how far along the mp path we are
    uint32_t lastMPMovement = 0;
    std::unique_ptr<MotionProfileGenerator::TrajectoryPair> trajectory;

    QLength driveStraightTarget = 0_m;
    QAngle angleTarget = 0_deg;

    Point lookAtTarget = { 0_m, 0_m };
    Point arcTarget = { 0_m, 0_m };

    QAngle angleOffset = 0_deg;
    QAngle headingTarget = 0_deg;
};
}
