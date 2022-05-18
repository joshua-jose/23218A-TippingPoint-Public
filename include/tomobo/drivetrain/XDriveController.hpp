#pragma once

#include "tomobo/drivetrain/TankDriveController.hpp"

namespace tomobo {

class XDriveController : public TankDriveController {
public:
    XDriveController(ChassisScales itrackingScales, ChassisScales ichassisScales,
        Odometry* iodometry,
        std::unique_ptr<FeedbackController> iforwardDistanceController,
        std::unique_ptr<FeedbackController> iangleController,
        std::unique_ptr<FeedbackController> istrafeController);

    ~XDriveController();

    enum class SimulDriveType {
        NO_HANDLER,
        ALWAYS_LINEAR,
        EQUAL
    };

    XDriveController* const strafe(QLength distance);
    XDriveController* const vectorDrive(Point point);
    XDriveController* const simulDrive(Point point, QAngle heading,
        SimulDriveType type = SimulDriveType::NO_HANDLER);
    XDriveController* const driveVectorAngle(Point target); // Mostly testing algorithm
    XDriveController* const holonomicPursuit(const Path& ipath);

    void step() override;

    void reset() override;
    bool isSettled();
    void wait();

    std::unique_ptr<FeedbackController> strafeController;

    PurePursuitController XDrivePursuitController;

protected:
    void strafeHandler();
    void driveStraightHandler() override;

    void vectorDriveHandler();
    void simulDriveHandler();
    void driveVectorAngleHandler();
    void holonomicPursuitHandler();

    void angleLinearPower(double linearPower, QAngle angle, double angularPower = 0);

    std::atomic<double> strafePower{ 0 };
    // Power for diagonal pairs of wheels.
    std::atomic<double> XPower{ 0 }, YPower{ 0 };

    QLength strafeTarget = 0_m;

    SimulDriveType simulDriveType = SimulDriveType::NO_HANDLER;
    Point vectorTarget{ 0_m, 0_m };
    bool simulDriveLinDisabled = false;
};
}
