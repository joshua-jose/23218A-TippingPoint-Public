#pragma once

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "pros/rtos.hpp"
#include "tomobo/odometry/Pose.hpp"
#include "tomobo/units.hpp"

#include <atomic>

namespace tomobo {
class Odometry {
public:
    ~Odometry();

    virtual void step() = 0;

    void startTask();
    void stopTask();

    Pose getPose();
    void setPose(const Pose& ipose);

    void setSampleTime(okapi::QTime isampleTime);
    okapi::QTime getSampleTime();

    // only public so the skills run can tap into it
    pros::Mutex mutex;
    Pose pose{ 0 * okapi::meter, 0 * okapi::meter, 0 * okapi::degree };

protected:
    virtual void loop();

    okapi::QTime sampleTime = 10 * okapi::millisecond;
    okapi::Timer timer;

    std::unique_ptr<pros::Task> task;
    // Run task says whether we *want* the task running
    std::atomic_bool runTask{ false };
    // taskRunning is set by the task itself, it definitively tells us if
    // the task is running or not.
    std::atomic_bool taskRunning{ false };
};
}
