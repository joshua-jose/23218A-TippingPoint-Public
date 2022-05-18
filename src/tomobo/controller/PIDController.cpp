#include "tomobo/controller/PIDController.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/timer.hpp"

namespace tomobo {

PIDController::PIDController(double ikP, double ikI, double ikD,
    bool iresetOnCross, bool islewEnabled, okapi::QTime islewTime) {

    std::unique_ptr<okapi::AbstractTimer> timer = std::make_unique<okapi::Timer>();
    settledUtil = std::make_unique<okapi::SettledUtil>(std::move(timer));

    kP = ikP;
    kI = ikI;
    kD = ikD;

    resetOnCross = iresetOnCross;
    slewEnabled = islewEnabled;
    slewTime = islewTime;
};

PIDController::PIDController(struct Gains gains) {
    std::unique_ptr<okapi::AbstractTimer> timer = std::make_unique<okapi::Timer>();
    settledUtil = std::make_unique<okapi::SettledUtil>(std::move(timer));

    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;
};

PIDController::PIDController(const PIDController& other) {
    std::unique_ptr<okapi::AbstractTimer> timer = std::make_unique<okapi::Timer>();
    settledUtil = std::make_unique<okapi::SettledUtil>(std::move(timer));

    kP = other.kP;
    kI = other.kI;
    kD = other.kD;
};

// Perform an iteration of calculations, reading inputs, and setting outputs
double PIDController::step(double reading) {
    if (disabled)
        return 0;

    okapi::QTime DtCurrent = peekDt();

    // If enough tmie has passed and we are ready to run an iteration...
    if (DtCurrent >= sampleRate) {
        // reset the Dt timer.
        getDt();
        error = reading;

        // If we want to reset the integral when we cross 0, and we have crossed 0
        // reset integral.
        if (resetOnCross && (std::copysign(1.0, error) != std::copysign(1.0, lastError))) {
            integral = 0;
        }

        bool shouldIntegrate = true;

        // Set shouldIntegrate to false if we don't want to integrate on "saturation" (when we are outputting the max output)
        // and we are saturating the output
        shouldIntegrate = shouldIntegrate && !(integrateOnSaturation && std::abs(output) == maxOutput);
        // Set shouldIntegrate to false if the previous statement was false, or...
        // the error is outside the bounds of integralMinSum and integralMaxSum.
        shouldIntegrate = shouldIntegrate && (std::abs(error) >= integralMinSum || std::abs(error) <= integralMaxSum);

        // Calculate integral and derivative
        if (shouldIntegrate)
            integral += error * sampleRate.convert(okapi::second);
        double derivative = (lastError - error) / sampleRate.convert(okapi::second);
        integral = std::min(integralMax, integral);

        lastError = error;

        // Calculate output value
        output = (kP * error) + (kI * integral) - (kD * derivative);
        // Limit the output to minOutput and maxOutput
        output = std::clamp(output, minOutput, maxOutput);
        // Limit the rate of change of output
        output = slew(output);
        // Also update the settled util with the error calculated from this iteration
        // It will use this number to determine if we are "settled", aka we are done with the movement.
        settledUtil->isSettled(error);
    }
    return output;
};

void PIDController::reset() {
    error = 0;
    lastError = 0;
    integral = 0;
    output = 0;
    slewLastInput = 0;
    settledUtil->reset();
};

// Functions to set/get different variables stored in the class.

void PIDController::setGains(double ikP, double ikI, double ikD) {
    kP = ikP;
    kI = ikI;
    kD = ikD;
};
void PIDController::setGains(struct Gains gains) {
    kP = gains.kP;
    kI = gains.kI;
    kD = gains.kD;
};

void PIDController::setResetOnCross(bool input) {
    resetOnCross = input;
};

void PIDController::setIntegrateOnSaturation(bool input) {
    integrateOnSaturation = input;
};

void PIDController::setIntegralSumBounds(double errorMin, double errorMax) {
    integralMinSum = errorMin;
    integralMaxSum = errorMax;
};

void PIDController::setIntegralMax(double input) {
    integralMax = input;
};

PIDController::Gains PIDController::getGains() {
    return { kP, kI, kD };
};

}
