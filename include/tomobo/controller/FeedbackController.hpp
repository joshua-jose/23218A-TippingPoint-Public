#pragma once

#include "okapi/api/control/util/settledUtil.hpp"

#include "pros/rtos.h"
#include "tomobo/units.hpp"

#include <algorithm>
#include <limits>

using namespace okapi::literals;

namespace tomobo {

class FeedbackController {

public:
    virtual double step(double reading) = 0;
    virtual void reset() = 0;

    void setTarget(double newTarget);
    double getTarget();

    void setTargetLimits(double min, double max);

    double getProcessVariable();
    double getOutput();
    double getError();

    void setMaxOutput(double newMaxOutput);
    void setMinOutput(double newMinOutput);
    void setOutputLimits(double min, double max);

    bool isDisabled();
    void setDisabled(bool newDisabled);

    void setSettledUtil(std::unique_ptr<okapi::SettledUtil> newSettledUtil);

    void setSampleRate(okapi::QTime newSampleRate);
    okapi::QTime getSampleRate();

    bool isSettled();

    // get Dt from last step.
    okapi::QTime peekDt();

    bool slewEnabled;

protected:
    //update Dt and return the new value.
    okapi::QTime getDt();

    double slew(double input);

    double target = 0, processVariable = 0, error = 0;
    double output = 0;
    double maxOutput = 1, minOutput = -1;
    double minTarget = std::numeric_limits<double>::min();
    double maxTarget = std::numeric_limits<double>::max();

    bool disabled = false;
    std::unique_ptr<okapi::SettledUtil> settledUtil;
    okapi::QTime sampleRate = 10_ms;

    uint32_t lastDtUpdate = pros::c::millis();
    okapi::QTime Dt = sampleRate;

    okapi::QTime slewTime;
    double slewLastInput;
};

}
