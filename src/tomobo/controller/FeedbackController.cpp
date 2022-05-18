#include "tomobo/controller/FeedbackController.hpp"

namespace tomobo {

/*  The idea is that a lot of these values are driven by the step function.
    These are mostly just naive getter/setter functions.
*/
void FeedbackController::setTarget(double newTarget) {
    target = std::clamp(newTarget, minTarget, maxTarget);
};
double FeedbackController::getTarget() {
    return target;
};

void FeedbackController::setTargetLimits(double min, double max) {
    minTarget = min;
    maxTarget = max;
};

double FeedbackController::getProcessVariable() {
    return processVariable;
};

double FeedbackController::getError() {
    //error = target - processVariable;
    return error;
};

double FeedbackController::getOutput() {
    return output;
};

void FeedbackController::setMaxOutput(double newMaxOutput) {
    maxOutput = newMaxOutput;
    output = std::clamp(output, minOutput, maxOutput);
};

void FeedbackController::setMinOutput(double newMinOutput) {
    minOutput = newMinOutput;
    output = std::clamp(output, minOutput, maxOutput);
};

void FeedbackController::setOutputLimits(double min, double max) {

    if (min > max) {
        double tmin = min;
        min = max;
        max = tmin;
    }

    minOutput = min;
    maxOutput = max;
    output = std::clamp(output, minOutput, maxOutput);
};

bool FeedbackController::isDisabled() {
    return disabled;
};
void FeedbackController::setDisabled(bool newDisabled) {
    disabled = newDisabled;
};

void FeedbackController::setSettledUtil(std::unique_ptr<okapi::SettledUtil> newSettledUtil) {
    settledUtil = std::move(newSettledUtil);
    // write our own?
    // Fits with the long term plan of having many different settle conditions.
};

void FeedbackController::setSampleRate(okapi::QTime newSampleRate) {
    if (newSampleRate <= 0_ms)
        return; //TODO: Log error.
    sampleRate = newSampleRate;
};
okapi::QTime FeedbackController::getSampleRate() {
    return sampleRate;
};

bool FeedbackController::isSettled() {
    return isDisabled() ? true : settledUtil->isSettled(error);
};
// Get delta time, aka get the time between now and the last time we asked for Dt
okapi::QTime FeedbackController::getDt() {
    uint32_t now = pros::c::millis();
    int32_t Dt_ms = now - lastDtUpdate;
    lastDtUpdate = now;
    Dt = okapi::millisecond * Dt_ms;

    return Dt;
};
// Get Dt without resetting the timer
okapi::QTime FeedbackController::peekDt() {
    uint32_t now = pros::c::millis();
    uint32_t Dt_ms = now - lastDtUpdate;

    return okapi::millisecond * Dt_ms;
};
// Simple function to limit the rate of change of the number passed to the slew function
double FeedbackController::slew(double input) {

    if (!slewEnabled)
        return input;

    double max_increase = sampleRate.convert(okapi::second) / slewTime.convert(okapi::second);
    double desired_change = input - slewLastInput;

    double clamped_increase = desired_change;

    if (std::abs(input) > std::abs(slewLastInput))
        clamped_increase = std::clamp(desired_change, -max_increase, max_increase);

    double output = slewLastInput + clamped_increase;
    slewLastInput = output;
    //printf("%f, %f, %f, %f\n", input, output, desired_change, max_increase);
    return output;
}

}
