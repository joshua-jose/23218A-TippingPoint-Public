#include "tomobo/controller/PIDFController.hpp"

namespace tomobo {
PIDFController::PIDFController(double ikP, double ikI, double ikD, double ikF,
    bool iresetOnCross, bool islewEnabled, okapi::QTime islewTime) :
    PIDController(ikP, ikI, ikD, iresetOnCross, islewEnabled, islewTime) {
    setGains(ikP, ikI, ikD, ikF);

    resetOnCross = iresetOnCross;
    slewEnabled = islewEnabled;
    slewTime = islewTime;
};

PIDFController::PIDFController(struct Gains gains) :
    PIDFController(gains.kP, gains.kI, gains.kD, gains.kF){};
PIDFController::PIDFController(const PIDFController& other) :
    PIDFController(other.kP, other.kI, other.kD, other.kF){};

double PIDFController::step(double reading) {
    if (disabled)
        return 0;

    okapi::QTime DtCurrent = peekDt();

    if (DtCurrent >= sampleRate) {
        getDt();
        error = reading;

        if (resetOnCross && (std::copysign(1.0, error) != std::copysign(1.0, lastError))) {
            integral = 0;
        }

        bool shouldIntegrate = true;

        shouldIntegrate = shouldIntegrate && !(!integrateOnSaturation && std::abs(output) == maxOutput);

        shouldIntegrate = shouldIntegrate && (std::abs(error) >= integralMinSum || std::abs(error) <= integralMaxSum);

        if (shouldIntegrate)
            integral += error * Dt.convert(okapi::second);
        double derivative = (lastError - error) / Dt.convert(okapi::second);
        integral = std::min(integralMax, integral);

        lastError = error;

        output = (kF * target) + (kP * error) + (kI * integral) - (kD * derivative);
        output = std::clamp(output, minOutput, maxOutput);
        output = slew(output);
        settledUtil->isSettled(error);
    }
    return output;
};

void PIDFController::setGains(double ikP, double ikI, double ikD, double ikF) {
    kP = ikP;
    kI = ikI;
    kD = ikD;
    kF = ikF;
};
void PIDFController::setGains(struct Gains gains) {
    setGains(gains.kP, gains.kI, gains.kD, gains.kF);
};

PIDFController::Gains PIDFController::getGains() {
    return { kP, kI, kD, kF };
};

}
