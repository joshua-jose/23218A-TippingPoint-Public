#include "tomobo/odometry/Odometry.hpp"

namespace tomobo {

// If the Odometry class is destroyed, make sure we end the task as well.
Odometry::~Odometry() {
    if (task)
        stopTask();
};

Pose Odometry::getPose() {
    return pose;
};

void Odometry::setPose(const Pose& ipose) {
    mutex.take(TIMEOUT_MAX);
    pose = ipose;
    mutex.give();
};

void Odometry::setSampleTime(okapi::QTime isampleTime) {
    sampleTime = isampleTime;
};
okapi::QTime Odometry::getSampleTime() {
    return sampleTime;
};

void Odometry::startTask() {
    if (task)
        return; // TODO: Log

    runTask.store(true);
    task = std::make_unique<pros::Task>(
        [=] {
            taskRunning.store(true);
            loop();
            taskRunning.store(false);
        },
        TASK_PRIORITY_MAX - 3, TASK_STACK_DEPTH_DEFAULT, "Odometry_Task");
};

void Odometry::stopTask() {
    if (!task)
        return; // TODO: Log

    task->notify();
    runTask.store(false);

    while (taskRunning.load())
        pros::delay(5);

    task.reset();
    mutex.give(); // Is this needed? try remove in testing.
};
// Background task that runs the odometry.
void Odometry::loop() {

    std::uint32_t now = pros::millis();
    while (runTask.load() && !task->notify_take(true, 0)) {
        mutex.take(TIMEOUT_MAX);
        step();
        mutex.give();
        pros::Task::delay_until(&now, sampleTime.convert(okapi::millisecond));
    };
};

}
